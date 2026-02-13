from machine import UART, Pin
import time

# --- Pin Definitions ---
UART_TX_PIN = 16 # GP16 (Connects to ESP RX)
UART_RX_PIN = 17 # GP17 (Connects to ESP TX)
ESP_RST_PIN = 29 # GP29 (ESP Reset)

def safe_decode(data_bytes):
    """Helper to safely decode bytes to string, filtering binary garbage."""
    if not data_bytes:
        return ""
    try:
        # Try standard decoding with ignore
        return data_bytes.decode('utf-8', 'ignore')
    except Exception:
        # Fallback: Filter for printable ASCII characters only
        # Keeps readable text and removes binary noise causing decode errors
        return "".join([chr(b) for b in data_bytes if 32 <= b <= 126 or b in (10, 13)])

def test_wifi_uart():
    print(f"--- Testing ESP-01S on UART (RX=GP{UART_RX_PIN}) ---")
    print("Safety: TX Pin (GP16) is set to Input (High-Z) initially.")
    
    # 1. Safety: Set TX pin to Input (High Impedance) to avoid conflict
    # if the ESP is not present and the pin is used for something else.
    tx_safe = Pin(UART_TX_PIN, Pin.IN)
    
    # 2. Initialize Reset Pin
    rst = Pin(ESP_RST_PIN, Pin.OUT)
    rst.value(1) 
    
    # 3. Initialize UART for Boot Logs (RX ONLY, 74880 baud)
    # We pass tx=None to try and prevent the UART from driving the TX pin yet.
    # Note: On some ports, if tx is not specified, it might default. 
    # But setting the Pin object above helps.
    print("1. Phase 1: Passive Listen (74880 baud)...")
    try:
        uart = UART(0, baudrate=74880, rx=Pin(UART_RX_PIN), tx=None, timeout=2000)
    except Exception:
        # Fallback if tx=None is not supported
        uart = UART(0, baudrate=74880, rx=Pin(UART_RX_PIN), timeout=2000)
    
    # 4. Perform Hardware Reset
    print("   Resetting ESP-01S...")
    rst.value(0) # Hold Reset
    time.sleep(0.1)
    rst.value(1) # Release Reset
    time.sleep(2.5) # Wait for boot logs
    
    # 5. Check for Boot Signature
    esp_detected = False
    if uart.any():
        raw_boot = uart.read()
        boot_msg = safe_decode(raw_boot)
        print(f"   Boot Output: {boot_msg.strip()}")
        
        # Check for common ESP8266 boot keywords
        # "ets Jan 8" = Standard Bootloader date
        # "csum" or "chksum" = Checksum verification
        if "ets Jan 8" in boot_msg or "csum" in boot_msg or "chksum" in boot_msg:
            print("   >>> VALID SIGNATURE DETECTED: ESP-01S is present.")
            esp_detected = True
        else:
            print("   >>> WARNING: Data received, but signature unclear.")
            # We assume detected if we got substantial data, but flag it
            if len(boot_msg) > 10:
                print("   Assuming detection based on data length.")
                esp_detected = True
    else:
        print("   >>> NO DATA: ESP-01S not detected or silent.")
    
    # 6. Phase 2: Active AT Command Test (Only if detected)
    if esp_detected:
        print("\n2. Phase 2: Active AT Test (115200 baud)...")
        print(f"   Configuring TX Pin (GP{UART_TX_PIN}) as UART Output now.")
        
        # Re-init UART with BOTH pins at 115200
        uart = UART(0, baudrate=115200, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN), timeout=2000)
        
        # Send AT Command
        cmd = "AT\r\n"
        print(f"   Sending command: {cmd.strip()}")
        uart.write(cmd)
        
        time.sleep(0.5)
        
        # Read Response
        if uart.any():
            response_bytes = uart.read()
            decoded = safe_decode(response_bytes).strip()
            print(f"   Response: {decoded}")
            
            if "OK" in decoded:
                print(">>> SUCCESS: ESP-01S responded with OK.")
            else:
                print(">>> WARNING: Received data, but not 'OK'. Check baud rate?")
        else:
            print(">>> FAIL: No response to AT command.")
            
    else:
        print("\n>>> SKIPPING PHASE 2: UART TX will not be enabled because ESP-01S was not confirmed.")
        print("    Test Finished safely.")

if __name__ == "__main__":
    test_wifi_uart()