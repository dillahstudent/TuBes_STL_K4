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
  STATE_EARTHQUAKE,
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

struct ButtonData {
  int buttonId;
  bool isRisingEdge; // Indicates whether it's a rising edge (true) or falling edge (false)
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

  // Attach interrupts to the buttons with CHANGE mode
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), handleButton1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), handleButton2Change, CHANGE);

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
  buttonQueue = xQueueCreate(10, sizeof(ButtonData));
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

void handleButton1Change() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonData buttonData;
  buttonData.buttonId = 1;
  buttonData.isRisingEdge = digitalRead(BUTTON1_PIN) == HIGH; // Check if it's a rising edge
  xQueueSendFromISR(buttonQueue, &buttonData, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR();
}

void handleButton2Change() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonData buttonData;
  buttonData.buttonId = 2;
  buttonData.isRisingEdge = digitalRead(BUTTON2_PIN) == HIGH; // Check if it's a rising edge
  xQueueSendFromISR(buttonQueue, &buttonData, &xHigherPriorityTaskWoken);
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
  BaseType_t xStatus;
  int additionalMessageArray[4] = {0,0,0,0};

  systemData.state = STATE_LEVEL_1;
  doorData.state = STATE_DOOR_CLOSE;

  // Task loop
  while (1) {
    // Check for new state data
    do {
        xStatus = xQueueReceive(stateQueue, &stateData, 0);
        if (xStatus == pdPASS) {
          if(stateData.state >= STATE_DOOR_OPEN){
            doorData.state = stateData.state;
          } else {
            systemData.state = stateData.state;
          }
        }
    } while (xStatus == pdPASS);

    // if (xQueueReceive(stateQueue, &stateData, 0) == pdPASS) {
    //   if(stateData.state >= STATE_DOOR_OPEN){
    //     doorData.state = stateData.state;
    //   } else {
    //     systemData.state = stateData.state;
    //   }
    // }

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
    switch (systemData.state) {
      case STATE_LEVEL_1:
        lcd.print("LEVEL 1 ");
        break;
      case STATE_LEVEL_2:
        lcd.print("LEVEL 2 ");
        break;
      case STATE_MOVING_UP:
        lcd.print("MOVE UP ");
        break;
      case STATE_MOVING_DOWN:
        lcd.print("MOVE DN ");
        break;
      case STATE_STOP:
        lcd.print("STOP    ");
        break;
      case STATE_EARTHQUAKE:
        lcd.print("QUAKE   ");
        break;
      default:
        break;
    }
        
    lcd.setCursor(8, 0);
    switch (doorData.state) {
      case STATE_DOOR_OPEN:
        lcd.print("OPEN    ");
        break;
      case STATE_DOOR_CLOSING:
        lcd.print("CLOSING ");
        break;
      case STATE_DOOR_CLOSE:
        lcd.print("CLOSED  ");
        break;
      default:
        break;
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
          lcd.print("EARTHQUAKE      ");
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
  AdditionalMessageType warningMessage;
  ButtonData buttonData;
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
      
      // Process potentiometer based on voltage
      if ((potentiometerData.voltage > 1) && (currentState != STATE_EARTHQUAKE)) {
        prevState = currentState;
        currentState = STATE_EARTHQUAKE;
        stateData.state = STATE_EARTHQUAKE;
        xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        warningMessage = MESSAGE_QUAKE;
        xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
        
        // Check if buzzer task is not already created
        if (buzzerTaskHandle == NULL) {
          // Create the buzzer task
          xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
        }
      }
    }

    // Check for new ultrasonic sensor data
    if (xQueueReceive(ultrasonicQueue, &ultrasonicData, 0) == pdPASS) {
      // Serial.print("Distance: ");
      // Serial.print(ultrasonicData.distance);
      // Serial.println(" cm    ");
      xQueueReset(ultrasonicQueue); // Empty the ultrasonic queue
    }

    // Check for new load cell data
    if (xQueueReceive(loadCellQueue, &loadCellData, 0) == pdPASS) {
      // Serial.print("Weight: ");
      // Serial.print(loadCellData.weight);
      // Serial.println(" kg    ");
      xQueueReset(loadCellQueue); // Empty the load cell queue
    }

    // Check for new button data
    if (xQueueReceive(buttonQueue, &buttonData, 0) == pdPASS) {
      // Serial.print("Button: ");
      // Serial.println(buttonData.buttonId);
      // Serial.print("is Rising?: ");
      // Serial.println(buttonData.isRisingEdge);
      
      // Process button press events based on button ID and edge
      if ((buttonData.isRisingEdge == false) && (currentState != STATE_EARTHQUAKE)) {
        prevState = currentState;
        currentState = STATE_STOP;
        stateData.state = STATE_STOP;
        xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        warningMessage = MESSAGE_EMERGENCY;
        xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
        
        // Check if buzzer task is not already created
        if (buzzerTaskHandle == NULL) {
          // Create the buzzer task
          xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
        }
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
              warningMessage = MESSAGE_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              warningMessage = MESSAGE_CLEAR_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*10){ //delay for 10 second (each loop is 500mS)
              timingCounter = 0;
              doorState = STATE_DOOR_CLOSING;
              stateData.state = STATE_DOOR_CLOSING;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;
          
          case STATE_DOOR_CLOSING:
            if(ultrasonicData.distance < 200){
              warningMessage = MESSAGE_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              warningMessage = MESSAGE_CLEAR_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*4){ //delay for 4 second (each loop is 500mS)
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
              warningMessage = MESSAGE_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              warningMessage = MESSAGE_CLEAR_WEIGHT;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*10){ //delay for 10 second (each loop is 500mS)
              timingCounter = 0;
              doorState = STATE_DOOR_CLOSING;
              stateData.state = STATE_DOOR_CLOSING;
              xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            }
            break;
          
          case STATE_DOOR_CLOSING:
            if(ultrasonicData.distance < 200){
              warningMessage = MESSAGE_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle == NULL) {
                xTaskCreate(TaskBuzzer, "Buzzer", 128, NULL, 1, &buzzerTaskHandle);
              }

              timingCounter = 0;
            } else {
              warningMessage = MESSAGE_CLEAR_PROXIMITY;
              xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
              
              if (buzzerTaskHandle != NULL) {
                // Delete the buzzer task
                vTaskDelete(buzzerTaskHandle);
                buzzerTaskHandle = NULL; // Reset task handle
              }
              timingCounter++;
            }

            if(timingCounter == 2*4){ //delay for 4 second (each loop is 500mS)
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

        if (buttonData.isRisingEdge == true) {
          xQueueReset(buttonQueue); // Empty the button queue

          warningMessage = MESSAGE_CLEAR_EMERGENCY;
          xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
        
          // Delete the buzzer task
          vTaskDelete(buzzerTaskHandle);
          buzzerTaskHandle = NULL; // Reset task handle

          currentState = prevState; // Transition to next state
          prevState = STATE_STOP; // Transition to next state
          stateData.state = currentState;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        } 
        break;

      case STATE_EARTHQUAKE:
        // Perform actions for DISPLAY state
        Serial.println("State: EARTHQUAKE");

        // Change potentiometer task priority to 3
        vTaskPrioritySet(potentiometerTaskHandle , 3);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if(prevState != STATE_LEVEL_1) {
          warningMessage = MESSAGE_QUAKE;
          xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);

          if(doorState != STATE_DOOR_CLOSE){
            doorState = STATE_DOOR_CLOSING;
            stateData.state = STATE_DOOR_CLOSING;
            xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            vTaskDelay(4000 / portTICK_PERIOD_MS);

            doorState = STATE_DOOR_CLOSE;
            stateData.state = STATE_DOOR_CLOSE;
            xQueueSend(stateQueue, &stateData, portMAX_DELAY);
            vTaskDelay(10000 / portTICK_PERIOD_MS);
          }

          warningMessage = MESSAGE_QUAKE;
          xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
          
          stateData.state = STATE_MOVING_DOWN;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
          vTaskDelay(10000 / portTICK_PERIOD_MS);
          
          warningMessage = MESSAGE_QUAKE;
          xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);

          stateData.state = STATE_LEVEL_1;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS);

          prevState = STATE_LEVEL_1;
        }

        if (prevState == STATE_LEVEL_1) {
          stateData.state = STATE_EARTHQUAKE;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
          doorState = STATE_DOOR_OPEN;
          stateData.state = STATE_DOOR_OPEN;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }

        if (potentiometerData.voltage <= 1) {
          xQueueReset(potentiometerQueue);

          warningMessage = MESSAGE_CLEAR_QUAKE;
          xQueueSend(additionalMessageQueue, &warningMessage, portMAX_DELAY);
        
          // Delete the buzzer task
          vTaskDelete(buzzerTaskHandle);
          buzzerTaskHandle = NULL; // Reset task handle

          // Change potentiometer task priority to 1
          vTaskPrioritySet(potentiometerTaskHandle , 1);

          currentState = prevState; // Transition to next state
          prevState = STATE_EARTHQUAKE; // Transition to next state
          stateData.state = currentState;
          xQueueSend(stateQueue, &stateData, portMAX_DELAY);
        }
        break;

      default:
        currentState = STATE_STOP;
        break;
    }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Small delay to prevent task hogging
  }
}
