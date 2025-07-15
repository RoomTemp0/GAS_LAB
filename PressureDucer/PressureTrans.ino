#include <wire.h>

const int Pressurepin1 = A0;
const int pumpcontrolpin1 = 7;


//These values are to be obtained by calibration of the design by hooking up everything and powering the transducer to see the VDC being outputted at the lowest Torr in the system
// There is no need for the highest to be obtained by the system as there is no limit, just a slope 
//const float V_REF = 0.5; // the Voltage being outputted at the lowest pressure in the system || example 
//const float P_REF = 1.0e-8; // the measured lowest pressure in the system || example
const float VOLTS_PER_DECADE = 0.5; // per decade of pressure the VDC goes +.5V as stated on the manual 


//const float Threshold_Torr = 500.0; // Example: 500 Torr, this is the threshold that we wan teverything to be based on
//const float Hysteresis_Torr = 10.0;       // Example: 10 Torr hysteresis to prevent chattering as the torr can go from >500 to <500 on the dime

//Voltage Divider for protection against the input analog voltage being over 5V
//const float R1 = 10000;
//const float R2 = 10000;


bool isPumpOn = false;


// Possiblility in using an OLED Screen to look at the values | Using Adafruit libraries 


void setup() {
  Serial.begin(9600);
  pinMode(pumpcontrolpin1, OUTPUT);
  digitalWrite(pumpcontrolpin1, LOW);
  // put your setup code here, to run once:

}

void loop() {

  int analogValue = analogRead(Pressurepin1);

  // Convert the analog value to voltage (assuming Arduino's VCC is 5V, and 10-bit ADC)
  // If you use analogReference(EXTERNAL) or internal reference, adjust 5.0 accordingly.

  float protected_voltage = analogValue * (5.0 / 1023.0); // Voltage at the Arduino pin
  //float Actual_Voltage = voltageArd * ((R1 + R2)/ R2); // This is for the voltage protection if the analog voltage at the max caps out the protected range for the arduino 


  float pressureTorr = pow(10, ((protected_voltage - V_REF) / VOLTS_PER_DECADE) + log10(P_REF));

  // --- Display Values for Debugging ---
  Serial.print("Analog Value: ");
  Serial.print(analogValue);
  Serial.print(", Voltage: ");
  Serial.print(protected_voltage, 3); // 3 decimal places
  Serial.print(" V, Pressure: ");
  Serial.print(pressureTorr, 6); // 6 decimal places for scientific notation
  Serial.println(" Torr");



  // --- Implement Pressure Threshold Logic with Hysteresis ---
  if (!pumpOn) { // If pump is currently off, check to turn it ON
    if (pressureTorr >= Threshold_Torr) {
      digitalWrite(pumpcontrolpin1, HIGH); // Turn pump ON (adjust HIGH/LOW based on your relay module)
      pumpOn = true;
      Serial.println("--- TURNING PUMP ON ---");
    }
  } else { // If pump is currently on, check to turn it OFF
    if (pressureTorr <= Threshold_Torr - Hysteresis_Torr) {
      digitalWrite(pumpcontrolpin1, LOW); // Turn pump OFF (adjust HIGH/LOW based on your relay module)
      pumpOn = false;
      Serial.println("--- TURNING PUMP OFF ---");
    }
  }

  delay(100); // Small delay to prevent too rapid readings/actions

}
