#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_log.h"

#define INPUT_PIN 4
#define LED_PIN 2

static const char *TAG = "switch_led:";

volatile long interruptCount = 0;

//int state = 0;
#define LED_MASK 1

static TaskHandle_t recvTaskHandle = NULL;
//static TaskHandle_t testTaskHandle = NULL;

static SemaphoreHandle_t mutexPtr;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    interruptCount++;
    if (xSemaphoreTake(mutexPtr,portMAX_DELAY) == pdPASS){
 
      if(gpio_get_level(INPUT_PIN))
        gpio_set_level(LED_PIN, 1);
      else
        gpio_set_level(LED_PIN, 0);

      xSemaphoreGive(mutexPtr);
    }
}

/*
void Test_Task(void *params) {

  while (1) {
    ESP_LOGI(TAG, "Test_Task: ");

    if (xSemaphoreTake(mutexPtr,portMAX_DELAY) == pdPASS){ //(10*1000) / portTICK_RATE_MS);//portMAX_DELAY)) 
      ESP_LOGI(TAG, "Test_Task: acquired mutex");
      xSemaphoreGive(mutexPtr);
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
*/

void LED_Control_Task(void *params) {
  int count = 0;
  
  while (true) {
    if (xSemaphoreTake(mutexPtr,portMAX_DELAY) == pdPASS){ //(10*1000) / portTICK_RATE_MS);//portMAX_DELAY)) 
      if (gpio_get_level(INPUT_PIN)){
        ESP_LOGI(TAG, "GPIO %d is pressed %d. The state is %d", INPUT_PIN, count++, gpio_get_level(INPUT_PIN));
      }
      else{
        ESP_LOGI(TAG, "GPIO %d is NOT pressed %d. The state is %d", INPUT_PIN, count++, gpio_get_level(INPUT_PIN));
      }

      if(!gpio_get_level(INPUT_PIN))
             gpio_set_level(LED_PIN, 0);

//      gpio_set_level(LED_PIN, gpio_get_level(INPUT_PIN));
      xSemaphoreGive(mutexPtr);
    }

    vTaskDelay(3000 / portTICK_RATE_MS);
  }
}

void app_main(void) {
  gpio_pad_select_gpio(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(INPUT_PIN);
  gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
  gpio_pulldown_en(INPUT_PIN);
  gpio_pullup_dis(INPUT_PIN);
  gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);

  mutexPtr = xSemaphoreCreateMutex();
  //assert_param(mutexPtr != NULL);

  xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, &recvTaskHandle);

  vTaskDelay(1000 / portTICK_RATE_MS);
//  xTaskCreate(Test_Task, "Test_Task", 2048, NULL, 1, &testTaskHandle);


  gpio_install_isr_service(0);
  gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
 
  while(1){
     vTaskDelay(3000 / portTICK_RATE_MS);
     if (xSemaphoreTake(mutexPtr,portMAX_DELAY) == pdPASS){ //(10*1000) / portTICK_RATE_MS);//portMAX_DELAY)) 
         ESP_LOGI(TAG, "Interrupt Count = %ld\n", interruptCount);
         xSemaphoreGive(mutexPtr);
     }
  }

}
