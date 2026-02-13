from machine import ADC, Pin
import time
import math

# --- Pin Definitions ---
THERM_0_PIN = 26  # GP26 (ADC0)
THERM_1_PIN = 27  # GP27 (ADC1)

# --- Constants for Logic ---
# ADC is 16-bit (0-65535)
ADC_MAX = 65535
V_REF = 3.3

# Thresholds
# If ADC > 64000, it's likely just the 10k pull-up (Open Circuit)
# If ADC < 500, it's likely grounded (Limit Switch Closed)
THRESH_OPEN = 64000 
THRESH_SHORT = 500

# Thermistor Params (Approximate for 100k NTC Beta 3950)
R_PULLUP = 10000.0 # 10k Ohm
R_NOMINAL = 100000.0 # 100k Ohm at 25C
TEMP_NOMINAL = 25.0
BETA = 3950.0

def adc_to_resistance(adc_val):
    """Calculates resistance of the device connected to ground."""
    if adc_val == 0 or adc_val >= ADC_MAX:
        return None
    
    # Voltage Divider: V_adc = V_ref * R_ntc / (R_ntc + R_pullup)
    # R_ntc = R_pullup * (V_adc / (V_ref - V_adc))
    # Using raw ADC: R_ntc = R_pullup * (adc / (65535 - adc))
    
    try:
        r_ntc = R_PULLUP * (adc_val / (ADC_MAX - adc_val))
        return r_ntc
    except ZeroDivisionError:
        return float('inf')

def get_approx_temp(resistance):
    """Simple Steinhart-Hart calculation for NTC temp."""
    if resistance is None or resistance <= 0:
        return None
        
    try:
        steinhart = resistance / R_NOMINAL     # (R/Ro)
        steinhart = math.log(steinhart)        # ln(R/Ro)
        steinhart /= BETA                      # 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMP_NOMINAL + 273.15) # + (1/To)
        steinhart = 1.0 / steinhart            # Invert
        steinhart -= 273.15                    # Convert to C
        return steinhart
    except Exception:
        return None

def test_thermistors():
    print(f"\n--- Testing Thermistors/Switches on GP{THERM_0_PIN} and GP{THERM_1_PIN} ---")
    
    # Initialize ADCs
    # We pass the pin number (int) directly to avoid "Pin doesn't have ADC capabilities" error
    adc0 = ADC(THERM_0_PIN)
    adc1 = ADC(THERM_1_PIN)
    
    # --- ADDED DELAY FOR CAPACITOR CHARGING ---
    print("Initializing ADCs...")
    print("Waiting 2 seconds for signal capacitors (100nF) to stabilize...")
    time.sleep(2)
    
    print("Logic:")
    print(f"  > {THRESH_OPEN}  : Limit Switch OPEN / No Sensor")
    print(f"  < {THRESH_SHORT}    : Limit Switch CLOSED")
    print("  In-between : Thermistor detected (Calc Temp)")
    print("\nStarting readings... Press Ctrl+C to stop.\n")
    
    try:
        while True:
            # Read Raw Values
            val0 = adc0.read_u16()
            val1 = adc1.read_u16()
            
            # --- Process Channel 0 ---
            status0 = "UNKNOWN"
            temp0_str = "---"
            
            if val0 > THRESH_OPEN:
                status0 = "SWITCH OPEN"
            elif val0 < THRESH_SHORT:
                status0 = "SWITCH CLOSED"
            else:
                # Calculate Temp
                res0 = adc_to_resistance(val0)
                temp0 = get_approx_temp(res0)
                if temp0 is not None:
                    status0 = "THERMISTOR"
                    temp0_str = f"{temp0:.1f}C"
                else:
                    status0 = "RANGE ERR"

            # --- Process Channel 1 ---
            status1 = "UNKNOWN"
            temp1_str = "---"
            
            if val1 > THRESH_OPEN:
                status1 = "SWITCH OPEN"
            elif val1 < THRESH_SHORT:
                status1 = "SWITCH CLOSED"
            else:
                # Calculate Temp
                res1 = adc_to_resistance(val1)
                temp1 = get_approx_temp(res1)
                if temp1 is not None:
                    status1 = "THERMISTOR"
                    temp1_str = f"{temp1:.1f}C"
                else:
                    status1 = "RANGE ERR"

            # Print
            # Overwrite line (using \r) for clean output if terminal supports it, 
            # otherwise just print new lines. Using standard print for safety.
            print(f"T0 (GP{THERM_0_PIN}): {val0:<5} [{status0:<13} {temp0_str:>6}]  |  T1 (GP{THERM_1_PIN}): {val1:<5} [{status1:<13} {temp1_str:>6}]")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest Stopped.")

if __name__ == "__main__":
    test_thermistors()