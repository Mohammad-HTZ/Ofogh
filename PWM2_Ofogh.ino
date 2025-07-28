// Pin settings
const int inputPin = 4;  // Input pin for PWM from function generator (GPIO4)
const int outputPin = 5; // Output pin to replicate PWM signal (GPIO5)
const int ledChannel = 0; // PWM channel (0 to 15 on ESP32)
const int resolution = 10; // PWM resolution (10-bit = 0 to 1023)

// Variables for measuring frequency and duty cycle
unsigned long lastTime = 0;
unsigned long highTime = 0;
unsigned long lowTime = 0;
unsigned long lastEdgeTime = 0;
bool lastState = false;
int pulseCount = 0;

void setup() {
  // Configure input and output pins
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  
  // Initialize Serial for displaying information
  Serial.begin(115200);
  Serial.println("PWM Passthrough Started");
}

void loop() {
  // Read the PWM input signal
  bool currentState = digitalRead(inputPin);
  
  // Pass the input signal directly to the output pin
  digitalWrite(outputPin, currentState);
  
  // Detect rising edge for frequency and duty cycle measurement
  if (lastState == false && currentState == true) {
    // Calculate period from the last rising edge
    unsigned long period = micros() - lastEdgeTime;
    lastEdgeTime = micros();
    pulseCount++;
    
    // Calculate duty cycle (high time / period)
    if (highTime > 0 && period > 0) {
      float dutyCycle = (float)highTime / period * 100.0;
      highTime = 0; // Reset high time
    }
  }
  
  // Calculate low and high times for duty cycle
  if (currentState == true) {
    highTime += micros() - lastEdgeTime;
    lastEdgeTime = micros();
  }
  
  // Calculate and display frequency and duty cycle every second
  if (millis() - lastTime >= 1000) {
    float measuredFreq = pulseCount; // Frequency = pulses per second
    float measuredDutyCycle = 0;
    if (highTime > 0 && (highTime + lowTime) > 0) {
      measuredDutyCycle = (float)highTime / (highTime + lowTime) * 100.0;
    }
    
    // Display on Serial
    Serial.print("Frequency: ");
    Serial.print(measuredFreq);
    Serial.println(" Hz");
    Serial.print("Duty Cycle: ");
    Serial.print(measuredDutyCycle);
    Serial.println(" %");
    
    // Reset counters
    pulseCount = 0;
    highTime = 0;
    lowTime = 0;
    lastTime = millis();
  }
  
  lastState = currentState;
}