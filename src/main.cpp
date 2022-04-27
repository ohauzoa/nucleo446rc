#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SerialCommands.h>

//HardwareSerial Serial2(USART2);
#ifndef LED_BUILTIN
  #define LED_BUILTIN PC13
#endif

char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");


TaskHandle_t Task_Handel1;
TaskHandle_t Task_Handel2;

void Task_Print1(void *pvParameters);
void Task_Print2(void *pvParameters);

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}


void setup()
{
  Serial2.begin(115200);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  xTaskCreate(Task_Print1,"Task1",100,NULL,1,&Task_Handel1);
  xTaskCreate(Task_Print2,"Task2",100,NULL,1,&Task_Handel2);


  Serial2.println("On");
    // start scheduler
  vTaskStartScheduler();
  Serial.println(F("Insufficient RAM"));
  while(1);

}

void Task_Print1(void *pvParameters)

{

  (void) pvParameters;

  while(1)

  {
  Serial2.println("TASK1");
  vTaskDelay(1000/portTICK_PERIOD_MS);
  }

}

void Task_Print2(void *pvParameters)

{

  (void) pvParameters;

  while(1){
  Serial2.println("TASK2");
  vTaskDelay(1000/portTICK_PERIOD_MS);

  }

}


void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(1000);
}
