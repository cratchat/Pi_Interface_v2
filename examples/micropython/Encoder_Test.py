from machine import Pin
import machine
import time
import sys

# Attempt to import the PIO Encoder driver 
# Ensure your library file is named 'QEnc_Pio_4.py' on the board
try:
    from QEnc_Pio_4 import QEnc_Pio_4
except ImportError:
    print("FATAL ERROR: QEnc_Pio_4.py (Encoder Library) not found.")
    sys.exit()

# --- Pin Definitions ---
# Shared Control Pins
# GP28 is shared between Stepper Enable (Active LOW) and Encoder Button (Active HIGH)
ENCODER_BTN_PIN = 28  

# Encoder Pins (for PIO)
ENCODER_A_PIN = 4   # GP4 (A Phase)
ENCODER_B_PIN = 5   # GP5 (B Phase)

# --- Globals ---
btn_pin = None
encoder_pio = None

def setup_encoder_button():
    """Initializes the Encoder Button and the PIO Encoder."""
    global btn_pin, encoder_pio
    
    print(f"\n--- Setting up Encoder (GP{ENCODER_A_PIN}/GP{ENCODER_B_PIN}) and Button (GP{ENCODER_BTN_PIN}) ---")
    
    # 1. Initialize Encoder Button
    # GP28 is shared: Needs to be configured as Input with PULL_DOWN 
    # because the button connects to 3.3V (Active HIGH) when pressed.
    btn_pin = Pin(ENCODER_BTN_PIN, Pin.IN, Pin.PULL_DOWN)
    print(f"Encoder Button initialized on GP{ENCODER_BTN_PIN} (Active HIGH, Pull-Down).")
    
    # 2. Initialize PIO Encoder 
    try:
        # Pins must be passed as Pin objects for the library initialization
        pin_a = Pin(ENCODER_A_PIN, Pin.IN)
        pin_b = Pin(ENCODER_B_PIN, Pin.IN)
        
        # Pass Pin objects and use the system clock frequency
        # sm_id=0 uses State Machine 0
        encoder_pio = QEnc_Pio_4((pin_a, pin_b), sm_id=0, freq=machine.freq())
        print("PIO Encoder initialized successfully.")
    except Exception as e:
        print(f"ERROR: Failed to initialize QEnc_Pio_4. Error: {e}")
        return False
        
    return True

def test_encoder_button():
    """Reads and prints the button state and encoder count loop."""
    global encoder_pio
    
    if not setup_encoder_button():
        return
        
    print("\nStarting Test Loop (Continuous)...")
    print("ACTION: Rotate the encoder knob and press the button.")
    print("To stop the test, press Ctrl+C or the Stop button.")
    
    last_print_time = 0
    
    try:
        # Run indefinitely until user interrupts
        while True: 
            
            # Read button state (1 when pressed, 0 when released)
            is_pressed = btn_pin.value()
            
            # Read encoder count using the library method
            count = encoder_pio.read() 
            
            # Determine status string
            btn_status = "PRESSED" if is_pressed else "RELEASED"
            
            # Print status every 100ms to avoid flooding console too fast
            current_time = time.ticks_ms()
            if time.ticks_diff(current_time, last_print_time) > 100:
                print(f"Encoder Count: {count:<6} | Button: {btn_status}")
                last_print_time = current_time
                
            time.sleep(0.01) # Short delay to prevent CPU hogging
            
    except KeyboardInterrupt:
        print("\nTest Stopped by User.")
        
    finally:
        # Cleanup PIO resources
        if encoder_pio:
            # Check if the library has a deinit method (future proofing)
            if hasattr(encoder_pio, 'deinit'):
                encoder_pio.deinit()
            # Fallback: Access the internal StateMachine 'qenc' and stop it
            elif hasattr(encoder_pio, 'qenc'):
                encoder_pio.qenc.active(0)
            
            print("PIO Deinitialized.")

# --- Main Test Execution ---
if __name__ == "__main__":
    test_encoder_button()
    print("\nNext feature to test: Load Drivers (MOSFETs).")