#include <WiFi.h>
#include <WebServer.h>

// --- WiFi Credentials ---
// Replace with your network details!
const char* ssid = "Home";
const char* password = "m76283175M@";

// --- Web Server Object---
WebServer server(80);

// --- Global variables to hold data for the web page ---
// Using String type to make it easy to display
String webVoltage = "N/A";
String webHighTime = "N/A";
String webLowTime = "N/A";
String webDuty = "N/A";
String webFrequency = "N/A";
// Pin settings
const int inputPin = 4;      // Input pin for PWM signal (must be an ADC pin, GPIO4 is ADC1_CH10)
const int outputPin = 5;     // Output pin to replicate PWM signal (GPIO5)
const int PWM_OUTPUT_PIN = 23; // *** FIX: Define a pin for the PWM output ***
const int VOLTAGE_PIN = 34;  // A separate pin for analog voltage reading

// PWM Generator settings
int PWMGenChannel = 0; // Use channel 0 for the generator
int freq = 100;
int res = 8;

// Variables for measurement (must be volatile for use in ISR)
volatile unsigned long pulseHighTime = 0;
volatile unsigned long pulseLowTime = 0;
volatile unsigned long lastEdgeTime = 0;
volatile bool newMeasurementAvailable = false;

// Variables for timed display
unsigned long lastPrintTime = 0;
int adcValue = 0;
float voltage = 0;
// This function builds and sends the HTML web page
void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<meta http-equiv=\"refresh\" content=\"2\">";
  html += "<link rel=\"icon\" href=\"data:,\">";
  html += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
  html += "h1 {color: #0F3376; padding: 2vh;} p {font-size: 1.5rem;}</style></head>";
  html += "<body><h1>ESP32 PWM Analyzer</h1>";
  html += "<p><strong>Frequency:</strong> " + webFrequency + " Hz</p>"; 
  html += "<p><strong>Duty Cycle:</strong> " + webDuty + " %</p>";    
  html += "<p><strong>High Duration:</strong> " + webHighTime + " &micro;s</p>";
  html += "<p><strong>Low Duration:</strong> " + webLowTime + " &micro;s</p>";
  html += "</body></html>";
  server.send(200, "text/html", html); 
}
void IRAM_ATTR measurePulse() {
  unsigned long currentTime = micros();
  
  if (digitalRead(inputPin)) {
    pulseLowTime = currentTime - lastEdgeTime;
    newMeasurementAvailable = true;
    // adcValue = analogRead(VOLTAGE_PIN);
    // voltage = (adcValue / 4095.0) * 3.3;
  } else {
    pulseHighTime = currentTime - lastEdgeTime;
  }
  
  lastEdgeTime = currentTime;
}

void setup() {
  Serial.begin(115200);
  Serial.println("PWM Analyzer Started");
  // --- Connect to WiFi and start server ---
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // This is the address for your browser

  server.on("/", handleRoot); // Set up the page handler
  server.begin();             // Start the web server
  Serial.println("HTTP server started");

  // --- Configure the PWM Generator ---
  // ledcSetup(PWMGenChannel, freq, res);
  // // *** FIX: Attach the PWM channel to the output pin ***
  // ledcAttachPin(PWM_OUTPUT_PIN, PWMGenChannel);
  ledcAttach(PWM_OUTPUT_PIN, freq, res);
  // --- Configure the Analyzer Pins ---
  pinMode(inputPin, INPUT_PULLUP);
  pinMode(outputPin, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT); // Simple input for analog reading

  // Attach interrupt to the input pin
  attachInterrupt(digitalPinToInterrupt(inputPin), measurePulse, CHANGE);
}

void loop() {
   server.handleClient();

  // Generate a PWM signal with a ~50% duty cycle (127 out of 255 for 8-bit resolution)
  ledcWrite(PWM_OUTPUT_PIN, 30);
  
  // Pass the input signal directly to the output pin
  digitalWrite(outputPin, digitalRead(inputPin));
  
  if (newMeasurementAvailable) {
    noInterrupts();
    unsigned long highTimeCopy = pulseHighTime;
    unsigned long lowTimeCopy = pulseLowTime;
    newMeasurementAvailable = false;
    interrupts();

    unsigned long period = highTimeCopy + lowTimeCopy;
    if (period > 0) { // Avoid division by zero
    //int adcValue = analogRead(inputPin);
      
      float frequency = 1000000.0 / period;
      float dutyCycle = (float)highTimeCopy * 100.0 / period;
      
      // Only print once per second
      if (millis() - lastPrintTime >= 1000) {
        lastPrintTime = millis();
        webHighTime = String(highTimeCopy);
        webLowTime = String(lowTimeCopy);
        webVoltage = String(voltage, 2); // Format voltage to 2 decimal places
        webDuty = String (dutyCycle);
        webFrequency = String ( frequency);
        Serial.println("--------------------");
        Serial.print("Frequency:  ");
        Serial.print(frequency, 2);
        Serial.println(" Hz");
        Serial.print("Duty Cycle: ");
        Serial.print(dutyCycle, 2);
        Serial.println(" %");
        // *** NEW: Print the measured pulse durations ***
        Serial.print("High Duration: ");
        Serial.print(highTimeCopy);
        Serial.println(" us"); // us = microseconds

        Serial.print("Low Duration:  ");
        Serial.print(lowTimeCopy);
        Serial.println(" us");
        // Serial.print("Input Voltage: ");
        // Serial.print(voltage, 2);
        // Serial.println(" V");
      }
    }
  }
}