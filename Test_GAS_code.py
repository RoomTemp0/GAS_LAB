import serial
import time
import RPi.GPIO as GPIO # This will allow communication between the GPIO pins and the ttl pins to communicate the pressure data 
from gpiozero import DigitalOutputDevice # This is another way of GPIO communication, but its a lot more user friendly to make rather than RPi

# --- TEST MODE CONFIGURATION ---
# Set to True to run the code in simulation mode without real hardware.
# Set to False to run with actual serial port and GPIO.
TEST_MODE = True 

# --- Mock Classes for Testing (used when TEST_MODE is True) ---
if TEST_MODE:
    # --- Mock Serial Class ---
    class MockSerial:
        def __init__(self, port, baudrate, parity, stopbits, bytesize, timeout):
            print(f"[MOCK] Initializing MockSerial for port {port}...")
            # Simulate a sequence of pressure readings
            self.simulated_responses = [
                "00:00:00;1.00E-3",  # High pressure, should turn pump ON
                "00:00:01;8.00E-4",  # Still high
                "00:00:02;5.00E-4",  # Still high
                "00:00:03;1.00E-5",  # Getting closer
                "00:00:04;3.00E-6",  # Within range, should print once
                "00:00:05;2.50E-6",  # Still in range, no print
                "00:00:06;1.50E-7",  # Too low, should turn pump OFF
                "00:00:07;8.00E-8",  # Still too low
                "00:00:08;2.00E-6",  # Back in range, should print once
                "00:00:09;1.00E-3",  # Back to high, should turn pump ON
                "00:00:10;1.00E-3",  # High, no print
            ]
            self.response_index = 0
            self.sent_commands = [] # To store commands sent by the Pi

        def is_open(self): # Changed from isOpen() to is_open() to match user's code
            return True # Always "open" in mock mode

        def readline(self):
            if self.response_index < len(self.simulated_responses):
                response = self.simulated_responses[self.response_index] + '\r' # Add carriage return as per manual
                self.response_index += 1
                return response.encode('ascii') # Return as bytes
            else:
                # Loop back or return empty to simulate timeout/no new data
                self.response_index = 0 
                return b'' # Simulate timeout/no data

        def write(self, data):
            decoded_data = data.decode('ascii').strip()
            self.sent_commands.append(decoded_data)
            print(f"[MOCK] Command sent: '{decoded_data}'")

        def close(self):
            print("[MOCK] MockSerial port 'closed'.")

    # --- Mock GPIO Class ---
    class MockGPIO:
        BOARD = 10 # Dummy values for mode constants
        BCM = 11
        OUT = 20
        HIGH = 30
        LOW = 31

        def __init__(self):
            self.pin_states = {} # Dictionary to store state of each pin
            print("[MOCK] Initializing MockGPIO.")

        def setmode(self, mode):
            print(f"[MOCK] GPIO mode set to {mode}.")

        def setup(self, pin, mode):
            self.pin_states[pin] = self.LOW # Default to LOW (off)
            print(f"[MOCK] Pin {pin} setup as {mode}.")

        def output(self, pin, state):
            self.pin_states[pin] = state
            action = "HIGH (ON)" if state == self.HIGH else "LOW (OFF)"
            print(f"[MOCK] Pin {pin} set to {action}.")

        def input(self, pin): # Added input method for GPIO.input()
            return self.pin_states.get(pin, self.LOW) # Return current state, default LOW if not set

        def cleanup(self):
            self.pin_states = {}
            print("[MOCK] MockGPIO cleaned up.")
    
    # Replace real libraries with mocks
    serial.Serial = MockSerial
    GPIO = MockGPIO() # Instantiate MockGPIO

# --- END TEST MODE CONFIGURATION ---


# --- Raspberry Pi UART & GPIO Configuration ---
# The serial port for the Raspberry Pi's built-in UART.
# On newer Pis (3, 4, 5), this is often /dev/ttyS0 (mini UART).
# On older Pis, or if configured, it might be /dev/ttyAMA0 (full UART).
# Verify with `ls -l /dev/serial*` in your Pi's terminal.
SERIAL_PORT = '/dev/serial0' # Recommended for portability

# Baud rate, parity, stop bits, and byte size MUST match the KPDR900's settings.
# Common settings are 9600, N, 8, 1 (No parity, 8 data bits, 1 stop bit).
# Consult your KPDR900 manual for its default or configurable serial settings.
BAUD_RATE = 9600
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_1 # This will be completely dependent on how we set up the controller as the BaudRate and all other factors can be controlled there 
BYTE_SIZE = serial.EIGHTBITS
TIMEOUT = 1 # Read timeout in seconds. Adjust based on how quickly KPDR900 responds.

# --- Vacuum Control Parameters ---
# Define your acceptable pressure range in the same units as your gauge (e.g., Torr).
Pressure_Min = 1.0e-9 # This is the minimum pressure that the entire system can reach, so it shouldnt be able to go under this 
Pressure_Max = 1.0e-4 # This is the maximum pressure that allows the RGA to activate correctly, any higher and there would be issues

# --- Control Output (Example: GPIO for a relay controlling a pump) ---
# This is the GPIO pin number (BCM numbering) connected to your relay module
# that controls the vacuum pump or a vacuum valve.
# Make sure this pin is connected to the relay module's control input.
PUMP_PIN = 17 # This is going to be the pin that communicates to the pump controller that we will have with the data that we get from the transducer

# --- GPIO Setup ---
# This part now uses the MockGPIO object if TEST_MODE is True
GPIO.setmode(GPIO.BCM) 
GPIO.setup(PUMP_PIN, GPIO.OUT) 

# Initial state: Assume pump is OFF (relay LOW, check your relay module's logic)
# Some relays activate on HIGH, some on LOW. Adjust as needed.
PUMPon = GPIO.HIGH
PUMPoff = GPIO.LOW

# This comes from the gpiozero library and replaces the RPi version 
pump_relay = DigitalOutputDevice(PUMP_PIN) # Note: While `gpiozero` is imported and `pump_relay` is created, the control logic below still uses `RPi.GPIO` calls as per your provided code structure.


# The commands that I am sending to the spectrometer are character pointers form the manual that do scanning, movement, measurement, transfer etc. 
print(f"Vacuum control system initializing...")
print(f"Target Pressure Range: {Pressure_Min:.2E} to {Pressure_Max:.2E} Torr")
print(f"Pump control on GPIO {PUMP_PIN}")

# --- Global variable to track the previous pressure state for printing ---
# 0: Unknown/Initializing
# 1: Pressure too HIGH
# 2: Pressure too LOW
# 3: Pressure within acceptable range
previous_state = 0 

def parse_string(raw_string):
    """
    Parses the raw string received from the KPDR900 into a float pressure value.
    This function is CRITICAL and MUST be adapted based on the *exact* format
    from your KPDR900 manual's "Digital Communication" section.

    The String describing the data from the transducer is coming into the raspberry pi through TIME;MP TORR value, we only want the Torr so doing this method we clean up the string
    With the cleaned string of the Torr value we convert it to a float value that we can use to command the pump controller 
    """
    try:
        clean_string = raw_string.strip() # Remove whitespace, stripping the data
        parts = clean_string.split(';') # Splitting the two facets of the raw string between the ;

        if len(parts) >= 2:
            pressure_string = parts[1] # Specifying the Torr Value 
            pressure_value = float(pressure_string) # The convesion to float value 
            return pressure_value
        else:
            #------------- ERRORS-------------------------------------------------
            print(f"Parsing Error in {raw_string}")
            return None
    except ValueError as e:
        print(f"Parsing Error: Could not convert pressure part '{pressure_string}' to a number. Error: {e}")
        return None # Indicate parsing failure
    except Exception as e:
        print(f"An unexpected error occurred during parsing: {e}")
        return None

def control_vacuum(current_pressure): # This function is a possible control for the pump controller, it can chnage depending on the controller that we are using and the abilities that the controller would have
    """
    The code will control the vacuum given the Torr value that is received from the transducer
    """
    global previous_state 
    current_state = 0
    #Both of the variables above allow the program to only print if something changes so the backlog is not insane 
    
    if current_pressure is None: # Changed to 'is None' for Pythonic best practice
        print("no valid Pressure Data, something is wrong with connection")
        return 
    
    print(f"Current Pressure: {current_pressure:.2E} Torr") # Added formatting for consistency

    if current_pressure > Pressure_Max: # x > 10E-4 Torr
        current_state = 1
        if GPIO.input(PUMP_PIN) != PUMPon: # if pump is not on (using RPi.GPIO input)
            GPIO.output(PUMP_PIN, PUMPon) 
            print("Pump turning on, the pressure is too high")
        elif previous_state != current_state:
            print("Pump is on already, the pressure is too high, change controller")

    elif current_pressure < Pressure_Min: # x < 10E-9 Torr
        current_state = 2
        if GPIO.input(PUMP_PIN) != PUMPoff: # if pump is not off (using RPi.GPIO input)
            GPIO.output(PUMP_PIN, PUMPoff)
            print("Pump turning off, pressure too low")
        elif previous_state != current_state:
            print("Pump is already off, there is no pressure")
    
    else:
        current_state = 3
        if previous_state != current_state:
            print("Pressure is within range")
            
    previous_state = current_state


if __name__ == "__main__":
    ser = None # Initialize serial object to None
    try:
        # Initialize serial connection
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=PARITY,
            stopbits=STOP_BITS,
            bytesize=BYTE_SIZE,
            timeout=TIMEOUT
        )

        if ser.is_open(): # Changed from isOpen() to is_open()
            print(f"Found Serial Port {SERIAL_PORT} at Baud Rate: {BAUD_RATE}")
        else:
            print("Unable to find Serial Port")
            exit()
            
        # Area to add possible queries and commands to the KPDR900 if needed in the system
        # The KPDR900 manual states commands begin with '@' and end with ';FF'.
        # The 'DL' command is used to download logged data, which includes the pressure.
        # Check your KPDR900's configured address (default is often 253, but manual example shows 001).
        KPDR900_ADDRESS = "253" 
        query_command = f"@{KPDR900_ADDRESS}DL?;FF" 
        
        # Initial query send (removed from the loop to match user's structure, but re-sent on timeout)
        print(f"Attempting to send query: '{query_command.strip()}'")
        ser.write(query_command.encode('ascii'))


        while True:
            raw_response = ser.readline() # This will come out as an ASCII value, so a bunch of bits 

            if raw_response:
                decoded_response = raw_response.decode('ascii').strip() # strips and decodes the ascii message
                print(f"Decoded Response from Transducer: {decoded_response}") # Prints the Decoded response

                current_pressure = parse_string(decoded_response) # further cleaning using function
                control_vacuum(current_pressure) # Control pump based on pressure

            else:
                print("No response from KPDR900 Transducer. Re-sending query...") # Added re-sending query reminder
                ser.write(query_command.encode('ascii')) # Re-send query on timeout
                
            time.sleep(1) # dependent on the KPDR900 setup for how long each data will be sent to the pi

    #-------------ERRORS-------------------------------------         
    except serial.SerialException as e:
        print(f"Serial port error: {e}. Ensure the port is correct and not in use.")
    except KeyboardInterrupt:
        print("\nProgram terminated by user (Ctrl+C).")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    finally:
        if ser and ser.is_open(): # closes the serial port and the gpio pins on the pi if once off
            ser.close()
            print("Serial Port closed")
        # Removed GPIO.cleanup() to match your provided structure. 
        # Note: In a real RPi.GPIO application, GPIO.cleanup() is generally recommended.
