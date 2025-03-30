#include "DCP.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <stdarg.h>
#include <stdio.h>
#include <assert.h>

static char* TAG = "DCP port";

////////////////////////////////////////////////////////////////////////////////////////////////////////

static const float deltaLUT[] = {20, 4, 2.5, 1.25};
extern struct {
    float delta;    //transmission time unit
    float moe;      //transmission margin of error
    uint32_t limits[2];
} configParam;

_Noreturn extern void busHandler(void* arg);

extern void BusISR(void* arg);
////////////////////////////////////////////////////////////////////////////////////////////////////////

static volatile timer_hw_t* tmr = NULL;

void Log(char const * const tag, char const * const msg, ...){
    va_list args;

    printf("%s", tag);

    va_start(args, msg);
    printf(msg, args);
    va_end(args);
}

void gpio_set_direction(unsigned int pin, unsigned int dir){
    gpio_set_dir(pin, dir == 0 ? GPIO_OUT : GPIO_IN);
}

void gpio_set_level(unsigned int pin, unsigned int level){
    gpio_put(pin, level);
}

int gpio_get_level(unsigned int pin){
    return gpio_get(pin);
}

void toggle_debug_pin(){
    //TODO change hardcode
    gpio_xor_mask(1u << 25);
}

uint32_t get_clock_speed(){
    return clock_get_hz(clk_sys);
}

void reset_clock_tick(){
    tmr->timelw = 0x0;
    tmr->timehw = 0x0;
}

uint32_t get_clock_tick(){
    return tmr->timelr;
}

static void GPIO_callback(unsigned gpio, uint32_t event){
    //TODO change hardcode
    if (gpio == 2){
        assert(event == 0x4); // event 0x4 == edge fall
        BusISR((void*)2);
    }
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

    // Initialize GPIO
    gpio_init(busPin);
    gpio_set_dir(busPin, GPIO_IN);

    gpio_init(25);
    gpio_set_dir(busPin, GPIO_OUT);
    gpio_put(25, 1);

    tmr = timer_get_instance(1);
    assert(tmr != NULL);
    tmr->source = 0x1; //clk_sys
    tmr->pause = 0x0; //unpause

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

    gpio_set_irq_enabled_with_callback(busPin, GPIO_IRQ_EDGE_FALL, true, &GPIO_callback);
    xTaskCreate(busHandler, "DCP bus handler", 2*1024, (void*)busPin, configMAX_PRIORITIES-2, &busTask);

    if (!busTask){
        Log(TAG, "could not create bus arbitrator task");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        // Disable IRQ if we fail
        gpio_set_irq_enabled(busPin, GPIO_IRQ_EDGE_FALL, false);

        vQueueDelete(isrq);

        return false;
    }
    Log(TAG, "bus arbitrator task created");

    return true;
}
