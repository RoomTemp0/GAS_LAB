#include <Wire.h>             // Required for I2C communication (even if OLED is removed, if other I2C devices are intended)

// --- PID Library (COMMENTED OUT - UNCOMMENT WHEN READY FOR PID CONTROL) ---
// You will need to install this library via Arduino IDE's Library Manager:
// Search for "PID_v1" by Brett Beauregard or a similar well-regarded PID library.
// #include <PID_v1.h>

// --- MKS Transducer Pin Definitions ---
const int PRESSURE_ANALOG_PIN = A0; // Arduino analog input pin connected to MKS transducer analog output (+)
// Note: MKS transducer Analog Output (-) should be connected to Arduino GND

// --- Spectrometer/RGA Control Pin Definition ---
const int SPECTROMETER_CONTROL_PIN = 7; // Arduino digital output pin connected to Spectrometer/RGA enable input

// --- Voltage Divider Resistor Values (REQUIRED IF USING HARDWARE VOLTAGE DIVIDER) ---
// IMPORTANT: Replace R1_VALUE and R2_VALUE with the actual resistor values you use.
// R1 is from transducer output to Arduino input. R2 is from Arduino input to GND.
// Example: If you use 10kOhm for R1 and 10kOhm for R2 (halving the voltage):
const float R1_VALUE = 10000.0; // Resistor connected from transducer output to analog pin
const float R2_VALUE = 10000.0; // Resistor connected from analog pin to GND

// --- MKS Transducer Calibration/Reference Values ---
// IMPORTANT: These values are CRITICAL for accurate pressure conversion.
// You MUST obtain these from a detailed MKS transducer manual or by empirical calibration.
// For demonstration, let's use a hypothetical reference (ADJUST THESE!):
// Hypothetical Example: Let's say at 0.5 VDC (V_REF), the pressure is 1.0e-8 Torr (P_REF).
const float V_REF = 0.5;      // Voltage (VDC) at P_REF
const float P_REF = 1.0e-8;   // Pressure (Torr) corresponding to V_REF
const float VOLTS_PER_DECADE = 0.5; // From MKS manual: 0.5 VDC/decade

// --- Spectrometer/RGA Activation Pressure Threshold ---
// This is the specific pressure where the Spectrometer/RGA should turn ON.
const float SPECTROMETER_ON_PRESSURE_TORR = 0.05; // Example: 0.05 Torr (50 mTorr)


// --- PID Control Variables (COMMENTED OUT - UNCOMMENT WHEN READY) ---
/*
// Define the PID input, output, and setpoint variables
double pressureInput;     // Input to PID: Current pressure from transducer
double pidOutput;         // Output from PID: Control signal for actuator
double pressureSetpoint;  // Setpoint for PID: Desired constant pressure

// Define PID tuning parameters (These need to be tuned for your specific system!)
// Start with Ki and Kd at 0, then tune Kp, then Ki, then Kd.
double Kp = 2.0;  // Proportional gain
double Ki = 0.5;  // Integral gain
double Kd = 0.1;  // Derivative gain

// Specify the PID control object
// Arguments: &Input, &Output, &Setpoint, Kp, Ki, Kd, Direction (DIRECT or REVERSE)
// Use DIRECT if increasing output increases pressure (e.g., opening a gas bleed valve).
// Use REVERSE if increasing output decreases pressure (e.g., opening a throttle valve).
PID myPID(&pressureInput, &pidOutput, &pressureSetpoint, Kp, Ki, Kd, REVERSE);

// Define the pin for the PID-controlled actuator (e.g., a PWM pin for a valve driver)
const int PID_ACTUATOR_PIN = 9; // Example: Digital pin 9, which supports PWM
// Define the range of the PID output that will map to the actuator
const int PID_OUTPUT_MIN = 0;   // Minimum PWM value (0-255) or analog voltage (0-255 mapped to 0-5V)
const int PID_OUTPUT_MAX = 255; // Maximum PWM value (0-255)
*/

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  // --- Initialize Spectrometer Control Pin ---
  pinMode(SPECTROMETER_CONTROL_PIN, OUTPUT);
  digitalWrite(SPECTROMETER_CONTROL_PIN, LOW); // Ensure Spectrometer/RGA is OFF initially

  // --- PID Setup (COMMENTED OUT - UNCOMMENT WHEN READY) ---
  /*
  // Initialize the PID actuator pin
  pinMode(PID_ACTUATOR_PIN, OUTPUT);

  // Set the PID output limits to match the analogWrite() range (0-255)
  myPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);

  // Set the PID sample time (how often the PID algorithm is computed)
  // A shorter sample time makes the PID react faster, but can cause instability if too short.
  myPID.SetSampleTime(100); // 100 milliseconds

  // Turn on the PID controller
  myPID.SetMode(AUTOMATIC);

  // Set your desired constant pressure setpoint here (e.g., 500 Torr)
  pressureSetpoint = 500.0; // Example: Set desired pressure to 500 Torr
  Serial.print("PID Setpoint: ");
  Serial.print(pressureSetpoint, 3);
  Serial.println(" Torr");
  */
}

void loop() {
  // --- Read Transducer Analog Output ---
  int analogReadValue = analogRead(PRESSURE_ANALOG_PIN);

  // Convert raw analog value to voltage at the Arduino pin (this is the scaled-down voltage)
  // Assuming Arduino's Vcc is 5.0V for analog reference (adjust if using 3.3V or EXTERNAL_AREF)
  float scaledVoltage = analogReadValue * (5.0 / 1023.0);

  // --- Undo the voltage division to get the actual transducer output voltage ---
  // Formula: V_transducer = V_scaled * ((R1_VALUE + R2_VALUE) / R2_VALUE)
  float actualTransducerVoltage = scaledVoltage * ((R1_VALUE + R2_VALUE) / R2_VALUE);

  // --- Convert Voltage to Pressure (Torr) ---
  // P = 10^(((V_out - V_REF) / VOLTS_PER_DECADE) + log10(P_REF))
  float pressureTorr = pow(10, ((actualTransducerVoltage - V_REF) / VOLTS_PER_DECADE) + log10(P_REF));

  // --- Spectrometer/RGA Control Logic ---
  // The goal is to turn the spectrometer ON when pressure reaches SPECTROMETER_ON_PRESSURE_TORR
  // and keep it on. If pressure goes too high, it might be turned off for safety.
  // This is a simple ON/OFF based on a single threshold.
  if (pressureTorr <= SPECTROMETER_ON_PRESSURE_TORR) {
    digitalWrite(SPECTROMETER_CONTROL_PIN, HIGH); // Turn Spectrometer ON
    // You might want to add a flag here if you only want it to turn on once and stay on
    // until explicitly reset or turned off by another condition.
    // For now, it will turn HIGH every loop if condition is met.
    Serial.println("--- Pressure below Spectrometer ON threshold. TURNING SPECTROMETER ON ---");
  } else {
    digitalWrite(SPECTROMETER_CONTROL_PIN, LOW); // Turn Spectrometer OFF (e.g., if pressure rises too high)
    Serial.println("--- Pressure above Spectrometer ON threshold. TURNING SPECTROMETER OFF ---");
  }


  // --- PID Control Loop (COMMENTED OUT - UNCOMMENT WHEN READY) ---
  /*
  // Assign the current pressure reading to the PID input variable
  pressureInput = pressureTorr;

  // Compute the PID output
  myPID.Compute();

  // Map the PID output to the actuator's control range
  // For a PWM-controlled proportional valve:
  analogWrite(PID_ACTUATOR_PIN, (int)pidOutput);

  // --- PID Debugging Output ---
  Serial.print("SP: "); Serial.print(pressureSetpoint, 3);
  Serial.print(" | PV: "); Serial.print(pressureInput, 6);
  Serial.print(" | PID Out: "); Serial.println((int)pidOutput);
  */
  
  // --- Serial Monitor Debugging ---
  Serial.print("Current Pressure: ");
  Serial.print(pressureTorr, 6);
  Serial.print(" Torr | Actual Voltage: ");
  Serial.print(actualTransducerVoltage, 3);
  Serial.print(" V | Spectrometer State: ");
  Serial.println(digitalRead(SPECTROMETER_CONTROL_PIN) == HIGH ? "ON" : "OFF");


  delay(500); // Update readings and serial monitor every 0.5 seconds
}
