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
  STATE_MOVING_UP,
  STATE_MOVING_DOWN,
  STATE_STOP,
  STATE_DOOR_OPEN,
  STATE_DOOR_CLOSING,
  STATE_DOOR_CLOSE
};

// Additional message types for LCD task
enum AdditionalMessageType {
  MESSAGE_WEIGHT = 0, 
  MESSAGE_PROXIMITY,
  MESSAGE_QUAKE,
  MESSAGE_EMERGENCY,
  MESSAGE_CLEAR_WEIGHT, 
  MESSAGE_CLEAR_PROXIMITY, 
  MESSAGE_CLEAR_QUAKE, 
  MESSAGE_CLEAR_EMERGENCY 
  // Add more message types as needed
};

// Task communication structures
struct KeypadData {
  char key;
  int keypadId; // Add this line
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
QueueHandle_t buttonQueue;
QueueHandle_t additionalMessageQueue;

// Create task handler
TaskHandle_t potentiometerTaskHandle  = NULL; // Task handle for the potentiometer task

volatile SystemState currentState = STATE_LEVEL_1;
volatile SystemState prevState = STATE_STOP;
volatile SystemState doorState = STATE_DOOR_CLOSE;

// Function prototypes
void TaskLoadCell(void *pvParameters);
void TaskPotentiometer(void *pvParameters);
void TaskUltrasonic(void *pvParameters);
void TaskBuzzer(void *pvParameters);
void TaskKeypad(void *pvParameters);
void TaskLCD(void *pvParameters);
void TaskStateMachine(void *pvParameters);

volatile bool ledStateButton1 = false;
volatile bool ledStateButton2 = false;

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
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), handleButton1Press, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), handleButton2Press, FALLING);

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
  buttonQueue = xQueueCreate(10, sizeof(int));
  additionalMessageQueue = xQueueCreate(5, sizeof(AdditionalMessageType));

  // Create tasks
  xTaskCreate(TaskLoadCell, "LoadCell", 256, NULL, 1, NULL);
  xTaskCreate(TaskPotentiometer, "Potentiometer", 256, NULL, 1, &potentiometerTaskHandle);
  xTaskCreate(TaskUltrasonic, "Ultrasonic", 256, NULL, 2, NULL); // Higher priority
  // xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, NULL); // Same priority as other tasks
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
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int buttonId = 1;
  xQueueSendFromISR(buttonQueue, &buttonId, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR();
}

void handleButton2Press() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int buttonId = 2;
  xQueueSendFromISR(buttonQueue, &buttonId, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR();
}

// In TaskKeypad, after reading the key, send it via queue
void TaskKeypad(void *pvParameters) {
  (void) pvParameters;
  KeypadData data;

  while (1) {
    // Check if any key is pressed for keypad 1
    char key1 = keypad1.getKey();
    if (key1 != NO_KEY) {
      data.key = key1;
      data.keypadId = 1; // Add this line
      xQueueSend(keypadQueue, &data, portMAX_DELAY);
      // Clear the keypad buffer
      keypad1.getKeys();
    }

    // Check if any key is pressed for keypad 2
    char key2 = keypad2.getKey();
    if (key2 != NO_KEY) {
      data.key = key2;
      data.keypadId = 2; // Add this line
      xQueueSend(keypadQueue, &data, portMAX_DELAY);
      // Clear the keypad buffer
      keypad2.getKeys();
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
  int timingCounterState = 0, timingCounterWarning = 0;
  StateData stateData;
  StateData systemData, doorData;
  AdditionalMessageType additionalMessage;
  int additionalMessageArray[4] = {0,0,0,0};

  // Task loop
  while (1) {
    // Check for new state data
    if (xQueueReceive(stateQueue, &stateData, 0) == pdPASS) {
      if(stateData.state >= STATE_DOOR_OPEN){
        doorData.state = stateData.state;
      } else {
        systemData.state = stateData.state;
      }
    }

    // Check for new additional messages
    if (xQueueReceive(additionalMessageQueue, &additionalMessage, 0) == pdPASS) {
      switch (additionalMessage){
        case MESSAGE_WEIGHT:
          additionalMessageArray[MESSAGE_WEIGHT] = 1;
          break;
        case MESSAGE_CLEAR_WEIGHT:
          additionalMessageArray[MESSAGE_WEIGHT] = 0;
          break;
        case MESSAGE_PROXIMITY:
          additionalMessageArray[MESSAGE_PROXIMITY] = 1;
          break;
        case MESSAGE_CLEAR_PROXIMITY:
          additionalMessageArray[MESSAGE_PROXIMITY] = 0;
          break;
        case MESSAGE_QUAKE:
          additionalMessageArray[MESSAGE_QUAKE] = 1;
          break;
        case MESSAGE_CLEAR_QUAKE:
          additionalMessageArray[MESSAGE_QUAKE] = 0;
          break;
        case MESSAGE_EMERGENCY:
          additionalMessageArray[MESSAGE_EMERGENCY] = 1;
          break;
        case MESSAGE_CLEAR_EMERGENCY:
          additionalMessageArray[MESSAGE_EMERGENCY] = 0;
          break;
      }
      
      xQueueReset(additionalMessageQueue); // Empty the potentiometer queue
    }

    lcd.setCursor(0, 0);
    if(timingCounterState == 0){
      switch (systemData.state) {
        case STATE_LEVEL_1:
          lcd.print("State: LEVEL 1  ");
          break;
        case STATE_LEVEL_2:
          lcd.print("State: LEVEL 2  ");
          break;
        case STATE_MOVING_UP:
          lcd.print("State: MOVING UP");
          break;
        case STATE_MOVING_DOWN:
          lcd.print("State: MOVING DN");
          break;
        case STATE_STOP:
          lcd.print("State: STOP     ");
          break;
        default:
          break;
      }
    } else if(timingCounterState == 1){
      switch (doorData.state) {
        case STATE_DOOR_OPEN:
          lcd.print("State: OPEN     ");
          break;
        case STATE_DOOR_CLOSING:
          lcd.print("State: CLOSING  ");
          break;
        case STATE_DOOR_CLOSE:
          lcd.print("State: CLOSED   ");
          break;
        default:
          break;
      }
    }

    lcd.setCursor(0, 1); // Set cursor to second line
    if(additionalMessageArray[timingCounterWarning] == 1){
      switch(timingCounterWarning){
        case MESSAGE_WEIGHT:
          lcd.print("Overload        ");
          break;
        case MESSAGE_PROXIMITY:
          lcd.print("Proximity       ");
          break;
        case MESSAGE_QUAKE:
          lcd.print("Earthquake      ");
          break;
        case MESSAGE_EMERGENCY:
          lcd.print("EMERGENCY       ");
          break;
        default:
          break;
      } 
    } else if(additionalMessageArray[timingCounterWarning] == 0){
      switch(timingCounterWarning){
        case MESSAGE_WEIGHT:
          lcd.print("Load is OK      ");
          break;
        case MESSAGE_PROXIMITY:
          lcd.print("Door is Clear   ");
          break;
        case MESSAGE_QUAKE:
          lcd.print("No Earthquake   ");
          break;
        case MESSAGE_EMERGENCY:
          lcd.print("No Emergency    ");
          break;
        default:
          break;
      }
    }

    timingCounterState++;
    if(timingCounterState == 2) timingCounterState = 0;

    timingCounterWarning++;
    if(timingCounterWarning == 4) timingCounterWarning = 0;

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Update the LCD every 1000 ms
  }
}

void TaskStateMachine(void *pvParameters) {
  (void) pvParameters;
  StateData stateData;
  KeypadData keypadData;
  PotentiometerData potentiometerData;
  UltrasonicData ultrasonicData;
  LoadCellData loadCellData;
  AdditionalMessageType additionalMessage;
  int buttonId;
  int timingCounter = 0;

  TaskHandle_t buzzerTaskHandle = NULL; // Task handle for the buzzer task

  // Task loop
  while (1) {
    //Check for new keypad data
    if (xQueueReceive(keypadQueue, &keypadData, 0) == pdPASS) {
      // Serial.print("Keypad ");
      // Serial.print(keypadData.keypadId); // Print keypad identifier
      // Serial.print(": ");
      // Serial.println(keypadData.key);
      xQueueReset(keypadQueue); // Empty the keypad queue
    }

    // Check for new potentiometer data
    if (xQueueReceive(potentiometerQueue, &potentiometerData, 0) == pdPASS) {
      // Serial.print("Voltage: ");
      // Serial.print(potentiometerData.voltage);
      // Serial.println(" V     ");
      xQueueReset(potentiometerQueue); // Empty the potentiometer queue
    }

    // Check for new ultrasonic sensor data
    if (xQueueReceive(ultrasonicQueue, &ultrasonicData, 0) == pdPASS) {
      // Serial.print("Distance: ");
      // Serial.print(ultrasonicData.distance);
      // Serial.println(" cm    ");
      // xQueueReset(ultrasonicQueue); // Empty the ultrasonic queue
    }

    // Check for new load cell data
    if (xQueueReceive(loadCellQueue, &loadCellData, 0) == pdPASS) {
      // Serial.print("Weight: ");
      // Serial.print(loadCellData.weight);
      // Serial.println(" kg    ");
      // xQueueReset(loadCellQueue); // Empty the load cell queue
    }

    // Check for new button data
    if (xQueueReceive(buttonQueue, &buttonId, 0) == pdPASS) {
      if (buttonId == 1) {
        ledStateButton1 = !ledStateButton1;
        Serial.println("Button 1 pressed");
      } else if (buttonId == 2) {
        ledStateButton2 = !ledStateButton2;
        Serial.println("Button 2 pressed");
      }
      xQueueReset(buttonQueue); // Empty the button queue
    }

    switch (currentState) {
      case STATE_LEVEL_1:
        if(prevState != STATE_LEVEL_1){
          Serial.println("State: LEVEL 1");

          prevState = STATE_LEVEL_1;
          stateData.state = STATE_LEVEL_1;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);

          doorState = STATE_DOOR_OPEN;
          stateData.state = STATE_DOOR_OPEN;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }

        switch (doorState) {
          case STATE_DOOR_OPEN:
            if(loadCellData.weight > 4){
              AdditionalMessageType warningMessage = MESSAGE_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              AdditionalMessageType warningMessage = MESSAGE_CLEAR_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*5){ //delay for 5 second (each loop is 500mS)
              timingCounter = 0;
              doorState = STATE_DOOR_CLOSING;
              stateData.state = STATE_DOOR_CLOSING;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;
          
          case STATE_DOOR_CLOSING:
            if(ultrasonicData.distance < 200){
              AdditionalMessageType warningMessage = MESSAGE_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              AdditionalMessageType warningMessage = MESSAGE_CLEAR_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*5){ //delay for 5 second (each loop is 500mS)
              timingCounter = 0;
              doorState = STATE_DOOR_CLOSE;
              stateData.state = STATE_DOOR_CLOSE;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;
          
          case STATE_DOOR_CLOSE:
            if((keypadData.keypadId != 0) && (keypadData.key == '2')){
              timingCounter = 0;
              keypadData.keypadId = 0;
              keypadData.key = 0;
              prevState = STATE_LEVEL_1;
              currentState = STATE_MOVING_UP;
              stateData.state = STATE_MOVING_UP;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            } else if ((keypadData.keypadId == 1) && (keypadData.key == '*')) {
              timingCounter = 0;
              keypadData.keypadId = 0;
              keypadData.key = 0;
              doorState = STATE_DOOR_OPEN;
              stateData.state = STATE_DOOR_OPEN;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;

          default:
            break;        
        }
        break;

      case STATE_LEVEL_2:
        if(prevState != STATE_LEVEL_2){
          Serial.println("State: LEVEL 2");

          prevState = STATE_LEVEL_2;
          stateData.state = STATE_LEVEL_2;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);

          doorState = STATE_DOOR_OPEN;
          stateData.state = STATE_DOOR_OPEN;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }

        switch (doorState) {
          case STATE_DOOR_OPEN:
            if(loadCellData.weight > 4){
              AdditionalMessageType warningMessage = MESSAGE_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              AdditionalMessageType warningMessage = MESSAGE_CLEAR_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*5){ //delay for 5 second (each loop is 500mS)
              timingCounter = 0;
              doorState = STATE_DOOR_CLOSING;
              stateData.state = STATE_DOOR_CLOSING;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;
          
          case STATE_DOOR_CLOSING:
            if(ultrasonicData.distance < 200){
              AdditionalMessageType warningMessage = MESSAGE_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              AdditionalMessageType warningMessage = MESSAGE_CLEAR_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*5){ //delay for 5 second (each loop is 500mS)
              timingCounter = 0;
              doorState = STATE_DOOR_CLOSE;
              stateData.state = STATE_DOOR_CLOSE;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;
          
          case STATE_DOOR_CLOSE:
            if((keypadData.keypadId != 0) && (keypadData.key == '1')){
              timingCounter = 0;
              keypadData.keypadId = 0;
              keypadData.key = 0;
              prevState = STATE_LEVEL_2;
              currentState = STATE_MOVING_DOWN;
              stateData.state = STATE_MOVING_DOWN;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            } else if ((keypadData.keypadId == 2) && (keypadData.key == '*')) {
              timingCounter = 0;
              keypadData.keypadId = 0;
              keypadData.key = 0;
              doorState = STATE_DOOR_OPEN;
              stateData.state = STATE_DOOR_OPEN;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;

          default:
            break;        
        }
        break;

      case STATE_MOVING_UP:
        if(prevState != STATE_MOVING_UP){
          Serial.println("State: MOVING UP");

          prevState = STATE_MOVING_UP;
          stateData.state = STATE_MOVING_UP;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);

          doorState = STATE_DOOR_CLOSE;
          stateData.state = STATE_DOOR_CLOSE;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }

        timingCounter++;

        if(timingCounter == 2*10){ //delay for 10 second (each loop is 500mS)
          timingCounter = 0;
          prevState = STATE_MOVING_UP;
          currentState = STATE_LEVEL_2;
          stateData.state = STATE_LEVEL_2;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }
        break;

      case STATE_MOVING_DOWN:
        if(prevState != STATE_MOVING_DOWN){
          Serial.println("State: MOVING DOWN");

          prevState = STATE_MOVING_DOWN;
          stateData.state = STATE_MOVING_DOWN;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);

          doorState = STATE_DOOR_CLOSE;
          stateData.state = STATE_DOOR_CLOSE;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }

        timingCounter++;

        if(timingCounter == 2*10){ //delay for 10 second (each loop is 500mS)
          timingCounter = 0;
          prevState = STATE_MOVING_DOWN;
          currentState = STATE_LEVEL_1;
          stateData.state = STATE_LEVEL_1;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }
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
    // if (currentState != STATE_STOP && buzzerTaskHandle != NULL) {
    //   // Delete the buzzer task
    //   vTaskDelete(buzzerTaskHandle);
    //   buzzerTaskHandle = NULL; // Reset task handle
    // }

    // Check if the system state is no longer STATE_STOP 
    // if (currentState != STATE_STOP) {
    //   // Change the priority of the potentiometer task back to 1
    //   vTaskPrioritySet(potentiometerTaskHandle , 1);
    // }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Small delay to prevent task hogging
  }
}
