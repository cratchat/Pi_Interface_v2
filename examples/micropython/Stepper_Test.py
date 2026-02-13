from machine import Pin
import time

# --- Pin Definitions ---
# Stepper 1
STEP_1_PIN = 0  # GP0
DIR_1_PIN = 1   # GP1

# Stepper 2
STEP_2_PIN = 2  # GP2
DIR_2_PIN = 3   # GP3

# Shared Enable Pin (Active LOW to Enable)
# Also the Encoder Button (Active HIGH)
ENABLE_PIN = 28 

def setup_steppers():
    print(f"--- Setting up Stepper Drivers ---")
    print(f"M1: Step=GP{STEP_1_PIN}, Dir=GP{DIR_1_PIN}")
    print(f"M2: Step=GP{STEP_2_PIN}, Dir=GP{DIR_2_PIN}")
    print(f"Shared Enable: GP{ENABLE_PIN} (Active LOW)")
    
    # Initialize Pins
    s1_step = Pin(STEP_1_PIN, Pin.OUT)
    s1_dir = Pin(DIR_1_PIN, Pin.OUT)
    
    s2_step = Pin(STEP_2_PIN, Pin.OUT)
    s2_dir = Pin(DIR_2_PIN, Pin.OUT)
    
    # Initialize Enable Pin
    # Start with HIGH (Disabled) for safety
    en_pin = Pin(ENABLE_PIN, Pin.OUT)
    en_pin.value(1) 
    
    return s1_step, s1_dir, s2_step, s2_dir, en_pin

def step_motor(step_pin, delay_us=500):
    """Generates a single step pulse."""
    step_pin.value(1)
    time.sleep_us(delay_us)
    step_pin.value(0)
    time.sleep_us(delay_us)

def run_motor_test(name, step_pin, dir_pin, steps=200, delay_us=800):
    """Moves a specific motor forward and backward."""
    print(f"\nTesting {name}...")
    
    # Forward
    print("  -> Forward")
    dir_pin.value(1) # Direction 1
    for _ in range(steps):
        step_motor(step_pin, delay_us)
    
    time.sleep(0.5)
    
    # Reverse
    print("  <- Reverse")
    dir_pin.value(0) # Direction 0
    for _ in range(steps):
        step_motor(step_pin, delay_us)
        
    time.sleep(0.5)

def test_steppers():
    s1_step, s1_dir, s2_step, s2_dir, en_pin = setup_steppers()
    
    print("\nStarting Stepper Test...")
    print("Enabling Motors (GP28 -> LOW)...")
    en_pin.value(0) # Enable Drivers
    time.sleep(0.5) # Allow drivers to wake up
    
    try:
        # Loop a few times
        for cycle in range(1, 4):
            print(f"\n=== Cycle {cycle} ===")
            
            # Test Motor 1
            # Assuming 1/16 microstepping, 3200 steps = 1 rev
            # We'll do a partial move for testing (e.g. 1/4 rev)
            run_motor_test("Motor 1", s1_step, s1_dir, steps=800, delay_us=400)
            
            # Test Motor 2
            run_motor_test("Motor 2", s2_step, s2_dir, steps=800, delay_us=400)
            
            # Test Simultaneous (Interleaved - Software approach)
            print("\n  -> Both Motors Forward (Simulated)")
            s1_dir.value(1)
            s2_dir.value(1)
            for _ in range(400):
                step_motor(s1_step, 400)
                step_motor(s2_step, 400)
            
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest Stopped by User.")
        
    finally:
        print("\nDisabling Motors (GP28 -> HIGH)...")
        en_pin.value(1)
        print("Test Complete.")

if __name__ == "__main__":
    test_steppers()