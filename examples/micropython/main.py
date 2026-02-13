import machine
from machine import Pin, PWM, I2C, UART, ADC
import time
import math
import _thread
import sys

# ==========================================
# PART 1: CYTRON BASE BOARD TEST (Startup)
# ==========================================

# --- Configuration ---
INTRO_MELODY_NOTE = [392, 392, 440, 392, 523, 494, 392, 392, 440, 392, 587, 523]
INTRO_MELODY_DURATION = [0.3, 0.1, 0.4, 0.4, 0.4, 0.8, 0.3, 0.1, 0.4, 0.4, 0.4, 0.8]

PIEZO_PIN = 22      # GP22 for Piezo Buzzer
NEOPIXEL_PIN = 23   # GP23 for 2 Neopixel RGB LEDs
NUM_PIXELS = 2

# All GPIOs for the "Running Light" effect
pins_top = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8, 11, 10]
pins_bot = [16, 17, 18, 19, 26, 27, 28, 29, 12, 13, 14, 15]

def mp_tone(pwm_pin, frequency, duration):
    """Generates a tone using PWM."""
    if frequency == 0:
        time.sleep(duration)
        return
    pwm = PWM(Pin(pwm_pin))
    try:
        pwm.freq(int(frequency))
        pwm.duty_u16(32768) 
        time.sleep(duration)
    finally:
        pwm.deinit()

def run_cytron_startup():
    print("\n=== PHASE 1: CYTRON BASE BOARD TEST ===")
    
    # 1. Neopixel Setup
    try:
        from neopixel import NeoPixel
        pixels = NeoPixel(Pin(NEOPIXEL_PIN, Pin.OUT), NUM_PIXELS)
        pixels.fill((0,0,0))
    except ImportError:
        print("Warning: neopixel module missing.")
        pixels = None

    # 2. GPIO LED Setup
    # Note: This will briefly drive HAT pins. This is expected.
    leds_top = [Pin(p, Pin.OUT) for p in pins_top]
    leds_bot = [Pin(p, Pin.OUT) for p in pins_bot]

    print("Playing Melody & Running LEDs...")
    
    # Play Melody & Chase LEDs
    for i in range(len(INTRO_MELODY_DURATION)):
        # LEDs On
        if i < len(leds_top): leds_top[i].value(1)
        if i < len(leds_bot): leds_bot[i].value(1)
        
        # Tone
        mp_tone(PIEZO_PIN, INTRO_MELODY_NOTE[i], INTRO_MELODY_DURATION[i])
        
        # LEDs Off
        if i < len(leds_top): leds_top[i].value(0)
        if i < len(leds_bot): leds_bot[i].value(0)

    # RGB Flash
    if pixels:
        print("Flashing RGB LEDs...")
        for color in [(50,0,0), (0,50,0), (0,0,50)]: # R, G, B
            pixels.fill(color)
            pixels.write()
            time.sleep(0.2)
        pixels.fill((0,0,0))
        pixels.write()

    # Cleanup: Ensure all GPIOs used are reset to Input to not interfere with HAT
    for p in pins_top + pins_bot:
        Pin(p, Pin.IN)
        
    print("Base Board Test Complete.\n")
    time.sleep(1)


# ==========================================
# PART 2: CUSTOM HAT INTEGRATION TEST
# ==========================================

# --- Hardware Constants ---
I2C_ID = 1
I2C_SDA = 18
I2C_SCL = 19
OLED_ADDR = 0x3C

UART_TX = 16
UART_RX = 17
ESP_RST = 29

THERM_0 = 26
THERM_1 = 27

STEP_1_STEP = 0
STEP_1_DIR = 1
STEP_2_STEP = 2
STEP_2_DIR = 3

LOAD_0 = 6
LOAD_1 = 7

SHARED_PIN = 28 # Enable (Low) / Button (High)
ENC_A = 4
ENC_B = 5

# --- Globals ---
oled = None
encoder_lib = None
esp_status = "Not Checked"
thread_running = True

def safe_decode(data_bytes):
    if not data_bytes: return ""
    try:
        return data_bytes.decode('utf-8', 'ignore')
    except:
        return "".join([chr(b) for b in data_bytes if 32 <= b <= 126])

def read_thermistor(adc_obj):
    # Logic: >64000=Open, <500=Closed(Switch), else=Thermistor
    THRESH_OPEN = 64000 
    THRESH_SHORT = 500
    R_PULLUP = 10000.0
    R_NOMINAL = 100000.0
    BETA = 3950.0
    TEMP_NOMINAL = 25.0
    
    raw = adc_obj.read_u16()
    
    if raw > THRESH_OPEN:
        return "OPEN"
    elif raw < THRESH_SHORT:
        return "SW-CLOSE"
    else:
        try:
            r_ntc = R_PULLUP * (raw / (65535 - raw))
            steinhart = math.log(r_ntc / R_NOMINAL) / BETA
            steinhart += 1.0 / (TEMP_NOMINAL + 273.15)
            temp = (1.0 / steinhart) - 273.15
            return f"{temp:.1f}C"
        except:
            return "ERR"

def init_oled():
    global oled
    print("Scanning I2C for OLED...")
    try:
        from ssd1306 import SSD1306_I2C
        i2c = I2C(I2C_ID, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
        if OLED_ADDR in i2c.scan():
            oled = SSD1306_I2C(128, 32, i2c, OLED_ADDR)
            oled.fill(0); oled.text("HAT Test Start", 0, 0); oled.show()
            print("  -> OLED Initialized.")
        else:
            print("  -> OLED not found.")
    except Exception as e:
        print(f"  -> OLED Error: {e}")

def update_display(l1, l2, l3):
    if not oled: return
    oled.fill(0)
    oled.text(l1, 0, 0)
    oled.text(l2, 0, 10)
    oled.text(l3, 0, 20)
    oled.show()

def check_esp():
    global esp_status
    print("Checking ESP-01S...")
    
    # Reset Pin
    rst = Pin(ESP_RST, Pin.OUT)
    rst.value(1)
    
    # Phase 1: Passive Listen (74880 baud)
    # Configure TX as Input first for safety
    tx_safe = Pin(UART_TX, Pin.IN)
    
    try:
        uart = UART(0, baudrate=74880, rx=Pin(UART_RX), tx=None, timeout=1500)
    except:
        uart = UART(0, baudrate=74880, rx=Pin(UART_RX), timeout=1500)

    # Pulse Reset
    rst.value(0); time.sleep(0.1); rst.value(1)
    time.sleep(2.0)
    
    detected = False
    if uart.any():
        boot_log = safe_decode(uart.read())
        if "ets Jan 8" in boot_log or "csum" in boot_log:
            detected = True
            print("  -> Boot Signature Detected.")
    
    # Phase 2: Active Test (115200 baud)
    if detected:
        print("  -> Sending AT Command...")
        uart = UART(0, baudrate=115200, tx=Pin(UART_TX), rx=Pin(UART_RX), timeout=1000)
        uart.write("AT\r\n")
        time.sleep(0.5)
        if uart.any():
            resp = safe_decode(uart.read())
            if "OK" in resp:
                esp_status = "OK"
                print("  -> ESP Response: OK")
            else:
                esp_status = "BadResp"
                print(f"  -> ESP Response: {resp.strip()}")
        else:
            esp_status = "NoResp"
            print("  -> ESP: No Response to AT")
    else:
        esp_status = "NotFnd"
        print("  -> ESP: Not Detected")

def actuator_thread():
    print("[Thread] Motor & Load Driver Thread Started")
    
    # PWM for Steppers (Continuous Spin)
    pwm_s1 = PWM(Pin(STEP_1_STEP))
    pwm_s2 = PWM(Pin(STEP_2_STEP))
    pwm_s1.freq(1000); pwm_s1.duty_u16(32768)
    pwm_s2.freq(1000); pwm_s2.duty_u16(32768)
    
    dir1 = Pin(STEP_1_DIR, Pin.OUT)
    dir2 = Pin(STEP_2_DIR, Pin.OUT)
    load0 = Pin(LOAD_0, Pin.OUT)
    load1 = Pin(LOAD_1, Pin.OUT)
    
    state = 0
    
    while thread_running:
        # Toggle Directions and Loads every 2 seconds
        val = state % 2
        dir1.value(val)
        dir2.value(val)
        load0.value(val)
        load1.value(1 - val) # Alternate loads
        
        state += 1
        time.sleep(2.0)
        
    # Safety Cleanup
    pwm_s1.deinit()
    pwm_s2.deinit()
    load0.value(0)
    load1.value(0)

def main():
    global encoder_lib, thread_running
    
    # 1. Run Base Board Test
    run_cytron_startup()
    
    print("=== PHASE 2: HAT INTEGRATION TEST ===")
    
    # 2. Init HAT Peripherals
    init_oled()
    check_esp()
    
    # 3. Init Encoder
    try:
        from QEnc_Pio_4 import QEnc_Pio_4
        encoder_lib = QEnc_Pio_4((Pin(ENC_A, Pin.IN), Pin(ENC_B, Pin.IN)), sm_id=0, freq=machine.freq())
        print("Encoder Initialized.")
    except:
        print("Encoder Lib Missing.")

    # 4. Init Sensors
    adc0 = ADC(THERM_0)
    adc1 = ADC(THERM_1)
    btn = Pin(SHARED_PIN, Pin.IN, Pin.PULL_DOWN)

    # 5. Start Background Thread (Motors/Loads)
    _thread.start_new_thread(actuator_thread, ())
    
    print("\n[Main Loop Running] Press Ctrl+C to Stop.")
    print("Motors are spinning. Encoder/Button/Thermistors active.")
    
    try:
        while True:
            # Poll Inputs
            enc_val = encoder_lib.read() if encoder_lib else 0
            btn_val = btn.value()
            t0_str = read_thermistor(adc0)
            t1_str = read_thermistor(adc1)
            
            btn_str = "PRS" if btn_val else "RLS"
            
            # Console Update
            status = f"Enc:{enc_val:<6} Btn:{btn_str} | T0:{t0_str:<6} T1:{t1_str:<6} | WiFi:{esp_status}"
            sys.stdout.write("\r" + status)
            
            # OLED Update
            update_display(f"WiFi:{esp_status} Btn:{btn_str}", f"Enc: {enc_val}", f"T0:{t0_str} T1:{t1_str}")
            
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        thread_running = False
        print("\nStopping...")
        if encoder_lib and hasattr(encoder_lib, 'deinit'):
            encoder_lib.deinit()
        time.sleep(1) # Wait for thread to finish
        print("Test Complete.")

if __name__ == "__main__":
    main()