#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// STM32 I2C Configuration
#define I2C_SDA PB9  // Customize for your board
#define I2C_SCL PB8

// Hardware Pins
#define LED1_PIN    PC13  // Built-in LED (common on STM32 boards)
#define LED2_PIN    PA1
#define BUTTON_PIN  PA0

// TSL2561 Configuration
#define TSL2561_ADDR 0x39
#define MIN_LUX      0.0
#define MAX_LUX  40000.0

// FreeRTOS Objects
QueueHandle_t xButtonQueue;
SemaphoreHandle_t xSerialMutex;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR);

// ISR: Button Press Handler
void buttonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xButtonQueue, NULL, &xHigherPriorityTaskWoken);
  if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// Task 1: LED Blinking
void TaskBlink(void* pvParameters) {
  pinMode(LED1_PIN, OUTPUT);
  while(1) {
    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task 2: Button Handling
void TaskButton(void* pvParameters) {
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  while(1) {
    if(xQueueReceive(xButtonQueue, NULL, portMAX_DELAY) == pdTRUE) {
      digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
      
      if(xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(50))) {
        Serial.print("[BUTTON] LED2: ");
        Serial.println(digitalRead(LED2_PIN) ? "ON" : "OFF");
        xSemaphoreGive(xSerialMutex);
      }
    }
  }
}

// Task 3: Light Sensor
void TaskSensor(void* pvParameters) {
  sensors_event_t event;
  
  while(1) {
    if(tsl.getEvent(&event)) {
      float lux = constrain(event.light, MIN_LUX, MAX_LUX);
      float pct = (lux - MIN_LUX) * 100.0 / (MAX_LUX - MIN_LUX);
      
      if(xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(50))) {
        Serial.print("[SENSOR] ");
        Serial.print(lux, 1);
        Serial.print(" lx (");
        Serial.print(pct, 1);
        Serial.println("%)");
        xSemaphoreGive(xSerialMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  // Initialize TSL2561
  if(!tsl.begin()) {
    Serial.println("TSL2561 initialization failed!");
    while(1);
  }
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);

  // Create RTOS objects
  xButtonQueue = xQueueCreate(5, 0);
  xSerialMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreate(TaskBlink,  "Blink",  128, NULL, 1, NULL);
  xTaskCreate(TaskButton, "Button", 128, NULL, 2, NULL);
  xTaskCreate(TaskSensor, "Sensor", 256, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {}