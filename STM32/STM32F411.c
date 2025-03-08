#include "DCP.h"

#include "stm32f4xx_hal_rcc.h"

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_ll_tim.h"

#include "stm32f4xx_hal_gpio.h";
#include "stm32f4xx_ll_gpio.h";

#include <stdarg.h>
#include <stdio.h>

#define GPIO_BANK GPIOB

TIM_HandleTypeDef* tmr;

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void Log(char const * const tag, char const * const msg, ...){
    (void)tag;

    static char buffer[512];
    va_list args;

    va_start(args, msg);
    int end = snprintf(buffer, 512, msg, args);
    va_end(args);

    //end will not point to \0, so if end == 509, the str == 510
    if (end++ < 510){
        buffer[end++] = '\r';
        buffer[end++] = '\n';
        buffer[end] = '\0';
    }else{
        buffer[510] = '\r';
        buffer[511] = '\n';
        buffer[512] = '\0';
        end = 512;
    }

    CDC_Transmit_FS(msg, end);
}

void inline gpio_set_direction(unsigned int pin, unsigned int dir){
    LL_GPIO_SetPinMode(GPIO_BANK, pin, dir == 0? LL_GPIO_MODE_OUTPUT: LL_GPIO_MODE_INPUT);
}

void inline gpio_set_level(unsigned int pin, unsigned int level){
    HAL_GPIO_WritePin(GPIO_BANK, pin, level);
}

int inline gpio_get_level(unsigned int pin){
    return HAL_GPIO_ReadPin(GPIO_BANK, pin);
}

uint32_t inline get_clock_speed(){
    return HAL_RCC_GetSysClockFreq();
}

void reset_clock_tick(){
    HAL_TIM_Base_Init(tmr);
}

uint32_t get_clock_tick(){
    return LL_TIM_GetCounter(&tmr);
}

extern DCP_MODE busMode;
extern QueueHandle_t RXmessageQueue;
extern QueueHandle_t TXmessageQueue;
extern QueueHandle_t isrq;

extern TaskHandle_t busTask;

bool DCPInit(const unsigned int busPin, const DCP_MODE mode){

    if (mode.addr == 0) return false;

    if (busMode.addr != 0){
        busMode = mode;
        return true;
    }

    gpio_set_direction(2, 0);

    CLOCK_TO_TIME = 1.0/get_clock_speed();

     GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIO_BANK, busPin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_BANK, &GPIO_InitStruct);

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

    configParam.limits[0] = ((configParam.delta - configParam.moe)*1e-6)/CLOCK_TO_TIME;
    configParam.limits[1] = ((configParam.delta + configParam.moe)*1e-6)/CLOCK_TO_TIME;

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

    HAL_NVIC_SetPriority(EXT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXT0_IRQn);

    xTaskCreate(busHandler, "DCP bus handler", 2*1024, &pin, configMAX_PRIORITIES-2, &busTask);

    if (!busTask){
        ESP_LOGE(TAG, "could not create bus arbitrator task");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        gpio_uninstall_isr_service();

        vQueueDelete(isrq);

        return false;
    }
    Log(TAG, "bus arbitrator task created");

    return true;
}

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == GPIO_PIN_13) {
        BusISR(GPIO_Pin);
    } else {
      __NOP();
    }
}
