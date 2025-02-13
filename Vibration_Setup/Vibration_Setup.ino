const int sensorPin = 4;  // SW-18020P connected to GPIO 21
volatile bool vibrationDetected = false; // Flag for vibration

void IRAM_ATTR vibrationISR() {
  vibrationDetected = true;  // Set flag when vibration occurs
}

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP);  // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(sensorPin), vibrationISR, FALLING);  // Detect LOW signal
}

void loop() {
  if (vibrationDetected) {
    Serial.println("Vibration Detected!");
    vibrationDetected = false;  // Reset flag after printing
  }
}
