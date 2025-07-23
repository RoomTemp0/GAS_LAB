# This is the code that we will use to code the Raspberry pi for the spectrometer, arduino, and pressure transducer communication
# This code will include the threshold limit to begin scanning as well as the characater pointers that the spectrometer can read from its manual
# This protocal will require an ethernet connection to a computer and a USB 2.0 connection to the spectrometer. 
# The Pi can make the spectrometer do its commands using the charcter pointers for a scan, save the scan as a .csv file to the Pi, and then send it to the computer using communication protocol to send the files to the computer 

from gpiozero import OutputDevice # This is another way of GPIO communication, but its a lot more user friendly to make rather than RPi
import time
import serial
import RPi.GPIO as GPIO # This will allow communication between the GPIO pins and the ttl pins to communicate the pressure data 


BAUD_RATE = 9600
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_1 # This will be completely dependent on how we set up the controller as the BaudRate and all other factors can be controlled there 
BYTE_SIZE = serial.EIGHTBITS
TIMEOUT = 1 

PUMP_PIN = 17 # This is going to be the pin that communicates to the pump controller that we will have with the data that we get from the transducer

Pressure_Min = 10E-9 # This is the minimum pressure that the entire system can reach, so it shouldnt be able to go under this 
Pressure_Max = 10E-4 # This is the maximum pressure that allows the RGA to activate correctly, any higher and there would be issues


PUMPon = GPIO.HIGH
PUMPoff = GPIO.LOW

pump_relay = DigitalOutputDevice(PUMP_PIN)# This comes from the gpiozero library and replaces the RPi version 




# The commands that I am sending to the spectrometer are character pointers form the manual that do scanning, movement, measurement, transfer etc. 
print(f"Vacuum control system initializing...")
print(f"Target Pressure Range: {Pressure_Min:.2E} to {Pressure_Max:.2E} Torr")
print(f"Pump control on GPIO {PUMP_PIN}")


def parse_string(raw_string):

# The String describing the data from the transducer is coming into the raspberry pi through TIME;MP TORR value, we only want the Torr so doing this method we clean up the string
# With the cleaned string of the Torr value we convert it to a float value that we can use to command the pump controller 

    try:
        clean_string = raw_string.strip() # Remove whitespace, stripping the data

        parts = clean_string.split(';') # Splitting the two facets of the raw string between the ;

        if len(parts)>= 2:
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
    
# The code will control the vacuum given the Torr value that is received from the transducer

    global previous_state
    current_state = 0
#Both of the variables above allow the program to only print if something changes so the backlog is not insane 
  
    if current_pressure == None:
        print("no valid Pressure Data, something is wrong with connection")
        return 
    
    print(f"Current Pressure: {current_pressure}")

    if current_pressure > Pressure_Max: # x > 10E-4 Torr
        current_state = 1
        if pump_relay.is_active != PUMPon: # if pump is not on
            GPIO.output(PUMP_PIN, PUMPon) 
            print("Pump truning on, the pressure is too high")
        elif previous_state != current_state:
            print("Pump is on already, the pressure is too high, change controller")

    elif current_pressure < Pressure_Min: # x < 10E-9 Torr
        current_state = 2
        if pump_relay.is_active != PUMPoff: # if pump is not off
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

        if ser.is_open():
            print(f"Found Serial Port {SERIAL_PORT} at Baud Rate: {BAUD_RATE}")
        else:
            print("Unable to find Serial Port")
            exit()
        

#Area to add possible queries and commands to the KPDR900 if needed in the system




        while True:

            raw_response = ser.readline() # This will come out as an ASCII value, so a bunch of bits 

            if raw_response:
                decoded_response = raw_response.decode('ascii').strip() # strips and decodes the ascii message
                print(f"Decoded Response from Transducer: {decoded_response}") # Prints the Decoded response

                current_pressure = parse_string(decoded_response) # further cleaning using function


            else:
                print("No response from KPDR900 Transducer")
            

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
