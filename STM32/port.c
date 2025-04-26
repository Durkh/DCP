#include "DCP.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_ll_tim.h"

#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_ll_gpio.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <stdarg.h>
#include <stdio.h>

#define GPIO_BANK GPIOB

static char* TAG = "DCP port";

////////////////////////////////////////////////////////////////////////////////////////////////////////

extern DCP_MODE busMode;

extern QueueHandle_t RXmessageQueue;
extern QueueHandle_t TXmessageQueue;
extern QueueHandle_t isrq;

extern TaskHandle_t busTask;

static const float deltaLUT[] = {20, 4, 2.5, 1.25};
extern struct {
    float delta;    //transmission time unit
    float moe;      //transmission margin of error
    uint32_t limits[2];
} configParam;

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
_Noreturn extern void busHandler(void* arg);

////////////////////////////////////////////////////////////////////////////////////////////////////////
extern TIM_HandleTypeDef htim2;

void Log(char const * const tag, char const * const msg, ...){
    (void)tag;

    static char buffer[512];
    va_list args;

    va_start(args, msg);
    int end = vsnprintf(buffer, 512, msg, args);
    va_end(args);

    //end will not point to \0, so if end == 509, the str == 510
    if (end < 510){
        buffer[end++] = '\r';
        buffer[end++] = '\n';
        buffer[end] = '\0';
    }else{
        buffer[509] = '\r';
        buffer[510] = '\n';
        buffer[511] = '\0';
        end = 511;
    }

    CDC_Transmit_FS((uint8_t*)buffer, end);
}

void gpio_set_direction(unsigned int pin, unsigned int dir){
    //TODO modify hardcode
    LL_GPIO_SetPinMode(GPIO_BANK, LL_GPIO_PIN_0, dir == 0? LL_GPIO_MODE_OUTPUT: LL_GPIO_MODE_INPUT);
}

void gpio_set_level(unsigned int pin, unsigned int level){
    HAL_GPIO_WritePin(GPIO_BANK, pin, level);
}

int gpio_get_level(unsigned int pin){
    return HAL_GPIO_ReadPin(GPIO_BANK, pin);
}

void toggle_debug_pin(){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

uint32_t get_clock_speed(){
    return HAL_RCC_GetSysClockFreq();
}

void reset_clock_tick(){
    HAL_TIM_Base_Init(&htim2);
}

uint32_t get_clock_tick(){
    return LL_TIM_GetCounter(htim2.Instance);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
extern DCP_MODE busMode;
extern QueueHandle_t RXmessageQueue;
extern QueueHandle_t TXmessageQueue;
extern QueueHandle_t isrq;

extern TaskHandle_t busTask;

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DCPInit(const unsigned int busPin, const DCP_MODE mode){

    if (mode.addr == 0) return false;

    if (busMode.addr != 0){
        busMode = mode;
        return true;
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

    /*
    HAL_GPIO_WritePin(GPIO_BANK, busPin, GPIO_PIN_RESET);

    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = busPin,
        .Mode = GPIO_MODE_IT_FALLING,
        .Pull = GPIO_PULLUP
    };

    HAL_GPIO_Init(GPIO_BANK, &GPIO_InitStruct);
    */

    RXmessageQueue = xQueueCreate(8, sizeof(uint8_t*));
    if (!RXmessageQueue){
        Log(TAG, "could not create RX message queue");

        return false;
    } 
    Log(TAG, "RX queue created");

    TXmessageQueue = xQueueCreate(8, sizeof(uint8_t*));
    if (!TXmessageQueue){
        Log(TAG, "could not create TX message queue");

        vQueueDelete(RXmessageQueue);

        return false;
    } 
    Log(TAG, "TX queue created");

    busMode = mode;
    configParam.delta = deltaLUT[busMode.speed];
    configParam.moe = .02*configParam.delta;

    configParam.limits[0] = ((configParam.delta - configParam.moe)*1e-6)*get_clock_speed();
    configParam.limits[1] = ((configParam.delta + configParam.moe)*1e-6)*get_clock_speed();

    Log(TAG, "transmission limits: [%lu ~ %lu]ticks", configParam.limits[0], configParam.limits[1]);
    Log(TAG, "transmission limits: [%.2f ~ %.2f]us", configParam.delta - configParam.moe, configParam.delta + configParam.moe);

    isrq = xQueueCreate(3, 0xFF);
    if(isrq == NULL){
        Log(TAG, "could not create ISR queue");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        return false;
    }
    Log(TAG, "ISR queue created");

    //HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    xTaskCreate(busHandler, "DCP bus handler", 2*1024, (void*)busPin, configMAX_PRIORITIES-2, &busTask);

    if (!busTask){
        Log(TAG, "could not create bus arbitrator task");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        //HAL_NVIC_DisableIRQ(EXTI0_IRQn);

        vQueueDelete(isrq);

        return false;
    }
    Log(TAG, "bus arbitrator task created");

    return true;
}

