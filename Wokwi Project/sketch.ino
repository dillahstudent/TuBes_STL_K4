#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <Keypad.h>

// Define LED pin (default is 13 for built-in LED on Arduino Mega)
const int ledPin = 13;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 49;
const int LOADCELL_SCK_PIN = 51;

// Potentiometer pin
const int POTENTIOMETER_PIN = A0;

// Ultrasonic sensor pins
const int TRIG_PIN = 9;
const int ECHO_PIN = 8;

// Buzzer pin
const int BUZZER_PIN = 18;

// Button pins
const int BUTTON1_PIN = 2;
const int BUTTON2_PIN = 3;

// Initialize the LCD library with the I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the HX711 library
HX711 scale;

// Define fixed multiplier for the load cell
const float loadCellMultiplier = 0.00238; // Replace this with your actual calibration factor

// Define variable for the ultrasonic sensor calibration
float ultrasonicMultiplier = 0.034245 / 2; // Initial calibration factor

// Define the keypads' pins and keys
const byte ROWS = 4; // Four rows
const byte COLS = 4; // Four columns

// Keypad 1
char keys1[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins1[ROWS] = {23, 22, 25, 24}; // R1, R2, R3, R4
byte colPins1[COLS] = {27, 26, 29, 28}; // C1, C2, C3, C4

Keypad keypad1 = Keypad(makeKeymap(keys1), rowPins1, colPins1, ROWS, COLS);

// Keypad 2
char keys2[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins2[ROWS] = {31, 30, 33, 32}; // R1, R2, R3, R4
byte colPins2[COLS] = {35, 34, 37, 36}; // C1, C2, C3, C4

Keypad keypad2 = Keypad(makeKeymap(keys2), rowPins2, colPins2, ROWS, COLS);

// Function prototypes
void TaskLoadCell(void *pvParameters);
void TaskPotentiometer(void *pvParameters);
void TaskUltrasonic(void *pvParameters);
void TaskBuzzer(void *pvParameters);
void TaskButton(void *pvParameters);
void TaskKeypad(void *pvParameters);

volatile bool ledState = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Set the buzzer pin as an output
  pinMode(BUZZER_PIN, OUTPUT);

  // Set the button pins as inputs with internal pull-up resistors
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // Attach interrupts to the buttons
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), handleButton1Press, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), handleButton2Press, CHANGE);

  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("FreeRTOS Demo");

  // Initialize the HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Initialize the ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Create tasks
  xTaskCreate(TaskLoadCell, "LoadCell", 256, NULL, 1, NULL);
  xTaskCreate(TaskPotentiometer, "Potentiometer", 256, NULL, 1, NULL);
  xTaskCreate(TaskUltrasonic, "Ultrasonic", 256, NULL, 2, NULL); // Higher priority
  xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, NULL); // Same priority as other tasks
  xTaskCreate(TaskButton, "Button", 128, NULL, 1, NULL); // Task to handle LED based on button press
  xTaskCreate(TaskKeypad, "Keypad", 256, NULL, 1, NULL); // Task to handle keypad input

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // Nothing here. FreeRTOS takes over.
}

// Interrupt service routines for the buttons
void handleButton1Press() {
  if (digitalRead(BUTTON1_PIN) == LOW) {
    ledState = true;
  } else {
    ledState = false;
  }
}

void handleButton2Press() {
  if (digitalRead(BUTTON2_PIN) == LOW) {
    ledState = true;
  } else {
    ledState = false;
  }
}

// Task to handle LED based on button press
void TaskButton(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    if (ledState) {
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to debounce
  }
}

// Task to read keypad inputs and print to serial monitor
void TaskKeypad(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    char key1 = keypad1.getKey();
    if (key1) {
      Serial.print("Keypad 1: ");
      Serial.println(key1);
    }

    char key2 = keypad2.getKey();
    if (key2) {
      Serial.print("Keypad 2: ");
      Serial.println(key2);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Check for key press every 100 ms
  }
}

// Task 3: Read from the load cell and print the value to the serial monitor and LCD
void TaskLoadCell(void *pvParameters) {
  (void) pvParameters;

  // Task loop
  while (1) {
    // Read the load cell value and convert it to weight
    float reading = scale.get_units(10); // Take an average of 10 readings
    float weight = reading * loadCellMultiplier;
    Serial.print("Weight: ");
    Serial.print(weight);
    Serial.println(" kg");
    lcd.setCursor(0, 1);
    lcd.print("                "); // Clear previous reading
    lcd.setCursor(0, 1);
    lcd.print("Weight: ");
    lcd.print(weight);
    lcd.print(" kg");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1000 ms
  }
}

// Task 4: Read the potentiometer value and print the voltage to the serial monitor
void TaskPotentiometer(void *pvParameters) {
  (void) pvParameters;

  // Task loop
  while (1) {
    // Read the potentiometer value
    int sensorValue = analogRead(POTENTIOMETER_PIN);
    float voltage = sensorValue * (5.0 / 1023.0); // Convert the reading to voltage

    Serial.print("Potentiometer Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1000 ms
  }
}

// Task 5: Read the ultrasonic sensor value and print the distance to the serial monitor
void TaskUltrasonic(void *pvParameters) {
  (void) pvParameters;

  // Task loop
  while (1) {
    // Clear the trigPin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Set the trigPin on HIGH state for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH);

    // Calculate the distance
    float distance = duration * ultrasonicMultiplier; // Use the calibration factor

    // Print the distance to the serial monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 500 ms
  }
}

// Task 6: Control the buzzer
void TaskBuzzer(void *pvParameters) {
  (void) pvParameters;

  // Task loop
  while (1) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1000 ms
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1000 ms
  }
}