#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#include <Keypad.h>
#include <queue.h>

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

// State machine states
enum SystemState {
  STATE_LEVEL_1,
  STATE_LEVEL_2,
  STATE_MOVING,
  STATE_STOP
};

// Task communication structures
struct KeypadData {
  char key;
};

struct LoadCellData {
  float weight;
};

struct PotentiometerData {
  float voltage;
};

struct UltrasonicData {
  float distance;
};

struct StateData {
  SystemState state;
};

// Queues to communicate with the LCD task
QueueHandle_t keypadQueue;
QueueHandle_t loadCellQueue;
QueueHandle_t potentiometerQueue;
QueueHandle_t ultrasonicQueue;
QueueHandle_t stateQueue;

// Create task handler
TaskHandle_t potentiometerTaskHandle  = NULL; // Task handle for the potentiometer task

volatile SystemState currentState = STATE_LEVEL_1;

// Function prototypes
void TaskLoadCell(void *pvParameters);
void TaskPotentiometer(void *pvParameters);
void TaskUltrasonic(void *pvParameters);
void TaskBuzzer(void *pvParameters);
void TaskButton(void *pvParameters);
void TaskKeypad(void *pvParameters);
void TaskLCD(void *pvParameters);
void TaskStateMachine(void *pvParameters);

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

  // Initialize the HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Initialize the ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Create queues
  keypadQueue = xQueueCreate(5, sizeof(KeypadData));
  loadCellQueue = xQueueCreate(5, sizeof(LoadCellData));
  potentiometerQueue = xQueueCreate(5, sizeof(PotentiometerData));
  ultrasonicQueue = xQueueCreate(5, sizeof(UltrasonicData));
  stateQueue = xQueueCreate(5, sizeof(StateData));

  // Create tasks
  xTaskCreate(TaskLoadCell, "LoadCell", 256, NULL, 1, NULL);
  xTaskCreate(TaskPotentiometer, "Potentiometer", 256, NULL, 1, &potentiometerTaskHandle);
  xTaskCreate(TaskUltrasonic, "Ultrasonic", 256, NULL, 2, NULL); // Higher priority
  // xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, NULL); // Same priority as other tasks
  xTaskCreate(TaskButton, "Button", 128, NULL, 1, NULL); // Task to handle LED based on button press
  xTaskCreate(TaskKeypad, "Keypad", 256, NULL, 1, NULL); // Task to handle keypad input
  xTaskCreate(TaskLCD, "LCD", 256, NULL, 1, NULL); // Task to handle LCD display
  xTaskCreate(TaskStateMachine, "StateMachine", 256, NULL, 1, NULL); // Task to handle state machine

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

// In TaskKeypad, after reading the key, send it via queue
void TaskKeypad(void *pvParameters) {
  (void) pvParameters;
  KeypadData data;

  while (1) {
    char key1 = keypad1.getKey();
    if (key1) {
      data.key = key1;
      xQueueSend(keypadQueue, &data, portMAX_DELAY);
    }

    char key2 = keypad2.getKey();
    if (key2) {
      data.key = key2;
      xQueueSend(keypadQueue, &data, portMAX_DELAY);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Check for key press every 100 ms
  }
}

// Task to read from the load cell and send the value to the LCD task
void TaskLoadCell(void *pvParameters) {
  (void) pvParameters;
  LoadCellData data;

  // Task loop
  while (1) {
    // Read the load cell value and convert it to weight
    float reading = scale.get_units(10); // Take an average of 10 readings
    data.weight = reading * loadCellMultiplier;

    xQueueSend(loadCellQueue, &data, portMAX_DELAY);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1000 ms
  }
}

// Task to read the potentiometer value and send the voltage to the LCD task
void TaskPotentiometer(void *pvParameters) {
  (void) pvParameters;
  PotentiometerData data;

  // Task loop
  while (1) {
    // Read the potentiometer value
    int sensorValue = analogRead(POTENTIOMETER_PIN);
    data.voltage = sensorValue * (5.0 / 1023.0); // Convert the reading to voltage

    xQueueSend(potentiometerQueue, &data, portMAX_DELAY);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1000 ms
  }
}

// Task to read the ultrasonic sensor value and send the distance to the LCD task
void TaskUltrasonic(void *pvParameters) {
  (void) pvParameters;
  UltrasonicData data;

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
    data.distance = duration * ultrasonicMultiplier; // Use the calibration factor

    xQueueSend(ultrasonicQueue, &data, portMAX_DELAY);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 500 ms
  }
}

// Task to control the buzzer
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

// Task to handle LCD updates
void TaskLCD(void *pvParameters) {
  (void) pvParameters;
  LoadCellData loadCellData;
  StateData stateData;

  // Task loop
  while (1) {
    // Check for new state data
    if (xQueueReceive(stateQueue, &stateData, 0) == pdPASS) {
      lcd.setCursor(0, 0);
      switch (stateData.state) {
        case STATE_LEVEL_1:
          lcd.print("State: LEVEL 1  ");
          break;
        case STATE_LEVEL_2:
          lcd.print("State: LEVEL 2  ");
          break;
        case STATE_MOVING:
          lcd.print("State: MOVING   ");
          break;
        case STATE_STOP:
          lcd.print("State: STOP     ");
          break;
        default:
          lcd.print("State: UNKNOWN  ");
          break;
      }
    }

    // Check for new load cell data
    if (xQueueReceive(loadCellQueue, &loadCellData, 0) == pdPASS) {
      lcd.setCursor(0, 1);
      lcd.print("Weight: ");
      lcd.print(loadCellData.weight);
      lcd.print(" kg    ");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Update the LCD every 1000 ms
  }
}

void TaskStateMachine(void *pvParameters) {
  (void) pvParameters;
  StateData stateData;
  KeypadData keypadData;
  PotentiometerData potentiometerData;
  UltrasonicData ultrasonicData;

  TaskHandle_t buzzerTaskHandle = NULL; // Task handle for the buzzer task

  // Task loop
  while (1) {
    // Check for new keypad data
    if (xQueueReceive(keypadQueue, &keypadData, 0) == pdPASS) {
      Serial.print("Keypad: ");
      Serial.println(keypadData.key);
      xQueueReset(keypadQueue); // Empty the keypad queue
    }

    // Check for new potentiometer data
    if (xQueueReceive(potentiometerQueue, &potentiometerData, 0) == pdPASS) {
      Serial.print("Voltage: ");
      Serial.print(potentiometerData.voltage);
      Serial.println(" V     ");
      xQueueReset(potentiometerQueue); // Empty the potentiometer queue
    }

    // Check for new ultrasonic sensor data
    if (xQueueReceive(ultrasonicQueue, &ultrasonicData, 0) == pdPASS) {
      Serial.print("Distance: ");
      Serial.print(ultrasonicData.distance);
      Serial.println(" cm    ");
      xQueueReset(ultrasonicQueue); // Empty the ultrasonic queue
    }

    switch (currentState) {
      case STATE_LEVEL_1:
        // Perform actions for IDLE state
        Serial.println("State: LEVEL 1");
        stateData.state = STATE_LEVEL_1;
        xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        // Transition to MEASURE state after some condition
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 2 seconds
        currentState = STATE_LEVEL_2;
        break;

      case STATE_LEVEL_2:
        // Perform actions for MEASURE state
        Serial.println("State: LEVEL 2");
        stateData.state = STATE_LEVEL_2;
        xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        // Trigger sensor reading tasks
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Simulate measurement time
        currentState = STATE_MOVING;
        break;

      case STATE_MOVING:
        // Perform actions for DISPLAY state
        Serial.println("State: MOVING");
        stateData.state = STATE_MOVING;
        xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        // Trigger display update task
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Simulate display update time
        currentState = STATE_STOP;
        break;

      case STATE_STOP:
        // Perform actions for DISPLAY state
        Serial.println("State: STOP");
        stateData.state = STATE_STOP;
        xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        
        // Check if buzzer task is not already created
        if (buzzerTaskHandle == NULL) {
          // Create the buzzer task
          xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
        }

        // Change potentiometer task priority to 3
        vTaskPrioritySet(potentiometerTaskHandle , 3);

        // Trigger display update task
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Simulate display update time
        currentState = STATE_LEVEL_1; // Transition to next state

        break;

      default:
        currentState = STATE_LEVEL_1;
        break;
    }

    // Check if the system state is no longer STATE_STOP and the buzzer task is created
    if (currentState != STATE_STOP && buzzerTaskHandle != NULL) {
      // Delete the buzzer task
      vTaskDelete(buzzerTaskHandle);
      buzzerTaskHandle = NULL; // Reset task handle
    }

    // Check if the system state is no longer STATE_STOP 
    if (currentState != STATE_STOP) {
      // Change the priority of the potentiometer task back to 1
      vTaskPrioritySet(potentiometerTaskHandle , 1);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to prevent task hogging
  }
}
