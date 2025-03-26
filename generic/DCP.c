#include "DCP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

//TODO change vvvvv
#include <portmacro.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

extern void Log(char const * const tag, char const * const msg, ...);

extern void gpio_set_direction(unsigned int pin, unsigned int dir);
extern void gpio_set_level(unsigned int pin, unsigned int level);
extern int gpio_get_level(unsigned int pin);

extern uint32_t get_clock_speed();
extern void reset_clock_tick();
extern uint32_t get_clock_tick();
extern void toggle_debug_pin();

static char* TAG = "DCP Driver";

volatile DCP_MODE busMode;

QueueHandle_t RXmessageQueue = NULL;
QueueHandle_t TXmessageQueue = NULL;
QueueHandle_t isrq = NULL;

TaskHandle_t busTask = NULL;

volatile struct {
    float delta;    //transmission time unit
    float moe;      //transmission margin of error
    uint32_t limits[2];
} configParam;

/*!
 * @brief generic definition of function that delays for microsseconds
 * @param ticks = delay in us * frequency in MHz
 */
static __attribute__((always_inline)) inline void Delay(const uint32_t ticks){
    reset_clock_tick();

    taskENTER_CRITICAL();

    //TODO overflow is assumed to not happen, it won't always be the case
    while (get_clock_tick() < ticks)
        asm volatile ("nop");

    taskEXIT_CRITICAL();
}

static inline bool s_ReadBit(const unsigned int pin){
    
    while (gpio_get_level(pin) == 0)
        continue;

    //reading high time
    reset_clock_tick();
    for (uint32_t lim = configParam.limits[1] << 1; gpio_get_level(pin) == 1 && get_clock_tick() < lim;)
        continue;

    return get_clock_tick() <= configParam.limits[1]? 0: 1;
}

static uint8_t s_ReadByte(const unsigned int pin){

    uint8_t byte = 0;

    for (int i = 7; i >= 0; --i){
        byte |= s_ReadBit(pin) << i;
    }

    return byte;
}


void BusISR(void* arg){

    const uint16_t pin = (uint16_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t data[0xFF];

    vTaskNotifyGiveFromISR(busTask, &xHigherPriorityTaskWoken);

    //reading incoming data
    gpio_set_direction(pin, 1);
    toggle_debug_pin();

    //wait for SYNC to end
    while(gpio_get_level(pin) == 0) continue;

    toggle_debug_pin();
    for (reset_clock_tick(); gpio_get_level(pin) == 1; ){
        if(get_clock_tick() > 10*configParam.limits[1]){
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            return;
        }
    }

    if(get_clock_tick() <= 6*configParam.limits[0]){
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        return;
    }

    toggle_debug_pin();
    data[0] = s_ReadByte(pin);
    const uint8_t flag = data[0]? data[0]: sizeof(struct DCP_Message_L3_t)+1;

    for (int i = 1; i < flag; ++i){
        toggle_debug_pin();
        data[i] = s_ReadByte(pin);
        toggle_debug_pin();
    }

    xQueueSendFromISR(isrq, data, &xHigherPriorityTaskWoken);
    toggle_debug_pin();

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static inline bool s_SendBytes(unsigned int const pin, uint8_t const size, uint8_t const data[size], unsigned const delays[restrict 3]){

     for (int i = 0; i < size; ++i){
        for (int j = 7; j >= 0; --j){
            //bus modulation
            // if bit == 0: 1 delta high, 1 delta low
            // else: 2 delta high, 1 delta low

            gpio_set_direction(pin, 1);

            if (((data[i] >> j) & 0x1) == 0){
                Delay(delays[0]);
            }else {
                Delay(delays[1]);
            }

            //collision
            //if any transmission pulled the pin low after
            //the delay started, they should still be
            //keeping the delay low.
            if (gpio_get_level(pin) == 0){
                return true;
            }

            //low side of the bit
            gpio_set_direction(pin, 0);
            gpio_set_level(pin, 0);

            Delay(delays[1] + delays[2]);
            gpio_set_direction(pin, 1);

            //collision
            if(gpio_get_level(pin) == 0){
                return true;

            }
        }
    }

    return false;
}

/*!
 * @brief task that controls the state machine of the control of the bus
 *
 *
 */
_Noreturn void busHandler(void* arg){

    const unsigned int pin = (unsigned int)arg;
    enum {STARTING, LISTENING, SENDING, WAITING, READING, END_} state = WAITING;

    //precalculations
    const uint32_t freqMHz = get_clock_speed()/1e6;

    //TODO change this BS
#ifdef CONFIG_IDF_TARGET_ESP32C3

    //negative skews in us to be added to the timings
    const unsigned int skews[4][4] = {
        //listening, sync, 0, 1
        {0, 20, 3, 3},
        {0, 25, 2, 1},
        {0, 20, 2, 2},
        {0, 20, 1, 0}
    };

#else 

    const unsigned int skews[4][4] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
    };

#endif

    const uint32_t delays[] = {
        (busMode.addr + 6) * configParam.delta/4.0 * freqMHz,
        ((busMode.isController?25:50) * configParam.delta-skews[busMode.speed][1])*freqMHz,
        (configParam.delta-skews[busMode.speed][2])*freqMHz,
        (2*configParam.delta-skews[busMode.speed][3])*freqMHz
    };

    //Log(TAG, "calculated delays:\n\tlistening: %lu cycles\n\tsync: %lu cycles\n\tbit 0: %lu cycles\n\tbit 1: %lu cycles", delays[0], delays[1], delays[2], delays[3]);

    //variables
    uint8_t qItem[0xFF];
    DCP_Data_t message = {0};
    bool collision = false;

    assert(RXmessageQueue != NULL);
    assert(TXmessageQueue != NULL);

    gpio_set_direction(pin, 1);

    while(1){

        //toggle_debug_pin();

        switch(state){
            case LISTENING:
                //listening bus for CSMA
                if(gpio_get_level(pin) == 0){
                    state = WAITING;
                    continue;
                }

                toggle_debug_pin();
                ulTaskNotifyValueClear(busTask, UINT_MAX);
                //protocol piority delay
                //devices with smaller addresses will have the priority
                Delay(delays[0]);

                //while in the delay, did someone take the bus?
                if(ulTaskNotifyTake(pdTRUE, 0)){
                    Log(TAG, "someone took the bus");
                    state = WAITING;
                    continue;
                }

                toggle_debug_pin();
                __attribute__((fallthrough));
            case STARTING:
                //starting communication
                if(gpio_get_level(pin) == 0){
                    taskEXIT_CRITICAL();
                    state = WAITING;
                    continue;
                }

                //sync signal
                gpio_set_direction(pin, 0);

                Delay(delays[1]);

                //bit sync signal
                //high part
                toggle_debug_pin();
                gpio_set_direction(pin, 1);
                Delay((uint32_t)(8 * delays[2]));

                //bit sync signal
                //low part
                gpio_set_direction(pin, 0);
                gpio_set_level(pin, 0);
                Delay((uint32_t)(8 * delays[2]));

                toggle_debug_pin();

                //leaving the bus still low not to interfere in the first bit
                __attribute__((fallthrough));
            case SENDING:
                //sending data
                assert(message.data != NULL);

                toggle_debug_pin();

                collision = s_SendBytes(pin,
                                        message.message->type? message.message->type: sizeof(struct DCP_Message_t),
                                        message.data,
                                        (unsigned[3]){delays[2], delays[3], 150});

                taskEXIT_CRITICAL();

                toggle_debug_pin();
                if (collision){
                    Log(TAG, "Collision detected");
                    state = LISTENING;
                    continue;
                }

                free(message.data);

                Log(TAG, "successfully sent message, going to wait mode");

                ulTaskNotifyValueClear(busTask, UINT_MAX);
                gpio_set_direction(pin, 1);

                __attribute__((fallthrough));
            case WAITING:
                //waiting for messages to send/receive
                state = WAITING;

                //message to read
                if(uxQueueMessagesWaiting(isrq)){
                    state = READING;
                    break;
                }
                 
                //message to send
                if (xQueueReceive(TXmessageQueue, &(message.data), 1) == pdPASS){
                    state = LISTENING;
                    break;
                }

                break;
            case READING:

                xQueueReceive(isrq, &qItem, pdMS_TO_TICKS(5));

                message.data = malloc(qItem[0]? qItem[0]: sizeof(struct DCP_Message_t) * sizeof(uint8_t));
                memmove(message.data, qItem, qItem[0]? qItem[0]: sizeof(struct DCP_Message_t));

                xQueueSend(RXmessageQueue, &(message.data), pdMS_TO_TICKS(15));

                state = WAITING;
                break;
            default:
                Log(TAG, "this code should not be executed, possible corruption");
                break;
        }
    }
}

bool SendMessage(const DCP_Data_t message){

#ifdef ESP_LOGD
    if (message.message->type){
        Log(TAG, "sending message: %s", message.message->generic.payload);
    } else {
        Log(TAG, "sending L3 message");
    }
#endif


    if (xQueueSend(TXmessageQueue, (void*)&(message.data), portMAX_DELAY) != pdTRUE){
        Log(TAG, "could not send message to queue");
        return false;
    }

    return true;
}

struct DCP_Message_t* ReadMessage(){

    DCP_Data_t message = {0};

    if ((busMode.flags.flags & 0x1) == FLAG_Instant){
        if (xQueueReceive(RXmessageQueue, &(message.data), 0) == pdTRUE){

            return message.message;
        }

        return NULL;
    }

    if (xQueueReceive(RXmessageQueue, &(message.data), portMAX_DELAY) == pdTRUE){

        return message.message;
    }

    return NULL;
}
