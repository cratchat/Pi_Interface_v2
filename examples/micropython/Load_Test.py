from machine import Pin
import time

# --- Pin Definitions ---
# Load Drivers (N-Channel MOSFETs, Active HIGH)
LOAD_0_PIN = 6  # GP6
LOAD_1_PIN = 7  # GP7

def test_load_drivers():
    print(f"\n--- Testing Load Drivers (MOSFETs) on GP{LOAD_0_PIN} and GP{LOAD_1_PIN} ---")
    
    # Initialize Pins
    # Active HIGH: 1 = ON (MOSFET Closed), 0 = OFF (MOSFET Open)
    load0 = Pin(LOAD_0_PIN, Pin.OUT)
    load1 = Pin(LOAD_1_PIN, Pin.OUT)
    
    # Ensure they start OFF
    load0.value(0)
    load1.value(0)
    print("Initialization: Both Loads OFF.")
    time.sleep(1)

    # Test Loop
    print("\nStarting Cycle Test...")
    print("Ensure you have a load (e.g., LED, Fan) or Multimeter connected to verify operation.")
    print("Press Ctrl+C to stop.")
    
    try:
        while True:
            # 1. Load 0 ON
            print(f"Load 0 (GP{LOAD_0_PIN}) -> ON")
            load0.value(1)
            time.sleep(1)
            
            # 2. Load 0 OFF
            print(f"Load 0 (GP{LOAD_0_PIN}) -> OFF")
            load0.value(0)
            time.sleep(0.5)
            
            # 3. Load 1 ON
            print(f"Load 1 (GP{LOAD_1_PIN}) -> ON")
            load1.value(1)
            time.sleep(1)
            
            # 4. Load 1 OFF
            print(f"Load 1 (GP{LOAD_1_PIN}) -> OFF")
            load1.value(0)
            time.sleep(0.5)
            
            # 5. Both ON
            print("Both Loads -> ON")
            load0.value(1)
            load1.value(1)
            time.sleep(1)
            
            # 6. Both OFF
            print("Both Loads -> OFF")
            load0.value(0)
            load1.value(0)
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nTest Stopped by User.")
    finally:
        # Safety: Turn off loads on exit
        load0.value(0)
        load1.value(0)
        print("Safety: All Loads turned OFF.")

if __name__ == "__main__":
    test_load_drivers()