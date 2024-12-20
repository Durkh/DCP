#include "DCP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <freertos/queue.h>

#include <driver/gpio.h>
#include <esp_private/esp_clk.h>
#include <esp_log.h>
#include <rom/ets_sys.h>

#include <assert.h>
#include <string.h>

static char* TAG = "DCP Driver";

portMUX_TYPE criticalMutex = portMUX_INITIALIZER_UNLOCKED;
volatile DCP_MODE busMode;

QueueHandle_t RXmessageQueue = NULL;
QueueHandle_t TXmessageQueue = NULL;

TaskHandle_t busTask = NULL;

static const float deltaLUT[] = {20, 4, 2.5, 1.25};
volatile struct {
    float delta;
} configParam;

static uint32_t CLOCK_TO_TIME;

/*!
 * @brief generic definition of function that delays for microsseconds
 * @param ticks = delay in us * frequency in MHz
 */
static void Delay(const esp_cpu_cycle_count_t ticks){
    esp_cpu_set_cycle_count(0);

    taskENTER_CRITICAL(&criticalMutex);

    //TODO overflow is assumed to not happen, it won't always be the case
    while (esp_cpu_get_cycle_count() < ticks)
        asm volatile ("nop");

    taskEXIT_CRITICAL(&criticalMutex);
}

static void BusISR(void* arg){
    (void)arg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(busTask, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static bool s_ReadBit(const gpio_num_t pin){
    
    //reading high time
    esp_cpu_set_cycle_count(0);
    //TODO bug possible spinlock
    while (gpio_get_level(pin) == 1)
        continue;

    uint32_t countH = esp_cpu_get_cycle_count();

    //confirming low time
    esp_cpu_set_cycle_count(0);
    while (gpio_get_level(pin) == 1)
        continue;
    uint32_t countL = esp_cpu_get_cycle_count();

    assert(countL*CLOCK_TO_TIME == configParam.delta);

    return countH*CLOCK_TO_TIME == configParam.delta? 0: 1;
}

static uint8_t s_ReadByte(const gpio_num_t pin){

    uint8_t byte = 0;

    //TODO check endianess
    for (int i = 8; i>0; --i){
        byte |= s_ReadBit(pin) << i;
    }

    return byte;
}

/*!
 * @brief task that controls the state machine of the control of the bus
 *
 *
 */
void _Noreturn busHandler(void* arg){

    const gpio_num_t pin = *((gpio_num_t*)arg);
    enum {STARTING, LISTENING, SENDING, WAITING, READING, END_} state = WAITING;

    //precalculations
    const uint32_t freqMHz = esp_clk_cpu_freq()/1e6;

    //TODO change this BS
#ifdef CONFIG_IDF_TARGET_ESP32C3

    const unsigned int skews[4][4] = {
        {0, 20, 4, 5},
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

    ESP_LOGV(TAG, "calculated delays:\n\tlistening: %lu cycles\n\tsync: %lu cycles\n\tbit 0: %lu cycles\n\tbit 1: %lu cycles", delays[0], delays[1], delays[2], delays[3]);

    //variables
    uint8_t senderAddr, receiverAddr;
    DCP_Message_t *message = NULL;

    assert(RXmessageQueue != NULL);
    assert(TXmessageQueue != NULL);

    gpio_set_direction(pin, GPIO_MODE_INPUT);

    while(1){
        ESP_LOGV(TAG, "changing to state %d", state);

        switch(state){
            case LISTENING:
                //listening bus for CSMA
                if(gpio_get_level(pin) == 0){
                    state = WAITING;
                    continue;
                }

                ESP_LOGV(TAG, "delaying");

                //protocol piority delay
                //devices with smaller addresses will have the priority
                Delay(delays[0]);

                //while in the delay, did someone take the bus?
                if(gpio_get_level(pin) == 0){
                    state = WAITING;
                    continue;
                }

                __attribute__((fallthrough));
            case STARTING:
                //starting communication
                if(gpio_get_level(pin) == 0){
                    state = WAITING;
                    continue;
                }

                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                gpio_set_level(pin, 0);

                Delay(delays[1]);

                //leaving the bus still low not to interfere in the first bit
                __attribute__((fallthrough));
            case SENDING:
                //sending actual data
                assert(message != NULL);

                taskENTER_CRITICAL(&criticalMutex);

                switch(message->type){
                    case MESSAGE_SYNC:
                        break;
                    case MESSAGE_L3:
                        break;
                    default:
                        for (int i = 0; i < message->size; ++i){
                            for (int j = 8; j != 0; --j){
                                //bus modulation
                                // if bit == 0: 1 delta high, 1 delta low
                                // else: 2 delta high, 1 delta low

                                gpio_set_direction(pin, GPIO_MODE_INPUT);
                                
                                if (((message->payload[i] >> j) & 0x1) == 0){
                                    Delay(delays[2]);
                                }else {
                                    Delay(delays[3]);
                                }

                                //collision
                                //if any transmission pulled the pin low after
                                //the delay started, they should still be
                                //keeping the delay low.
                                if (gpio_get_level(pin) == 0){
                                    taskEXIT_CRITICAL(&criticalMutex);
                                    ESP_LOGD(TAG, "Collision on high detected, stopping transmission");
                                    state = LISTENING;
                                    //TODO bug continuing the for, not changing
                                    //the state
                                    continue;
                                }

                                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                                gpio_set_level(pin, 0);

                                Delay(delays[2]);
                                gpio_set_direction(pin, GPIO_MODE_INPUT);

                                //collision
                                if(gpio_get_level(pin) == 0){
                                    taskEXIT_CRITICAL(&criticalMutex);
                                    ESP_LOGD(TAG, "Collision on low detected, stopping transmission");
                                    state = LISTENING;
                                    //TODO bug continuing the for, not changing
                                    //the state
                                    continue;
                                }
                            }
                        }
                }

                taskEXIT_CRITICAL(&criticalMutex);

                ESP_LOGV(TAG, "successfully sent message, going to wait mode");

                ulTaskNotifyValueClear(busTask, UINT_MAX);
                gpio_set_direction(pin, GPIO_MODE_INPUT);

                __attribute__((fallthrough));
            case WAITING:
                //waiting for messages to send/receive
                state = WAITING;

                //message to read
                if(ulTaskNotifyTake(pdTRUE, 1)){
                    state = READING;
                    break;
                }
                 
                //message to send
                if (xQueueReceive(TXmessageQueue, &(message), 1) == pdPASS){
                    state = LISTENING;
                    break;
                }

                break;
            case READING:
                //reading the address and/or data

                gpio_set_direction(pin, GPIO_MODE_INPUT);

                //wait for SYNC to end
                //while(gpio_get_level(pin) == 0) continue;

                taskENTER_CRITICAL(&criticalMutex);

                //reading origin address
                for (int i = 0; i<8; ++i){
                    senderAddr |= s_ReadBit(pin) << i;
                }

                //reading destination address?
                for (int i = 0; i<8; ++i){
                    receiverAddr |= s_ReadBit(pin) << i;
                }

                if (receiverAddr != busMode.addr){
                    //TODO (BUG) got to wait to the end of transmission
                    state = WAITING;
                    continue;
                }

                message = malloc(sizeof * message);

                const uint8_t flag = s_ReadByte(pin);
                switch (flag) {
                    case MESSAGE_SYNC:
                        while(gpio_get_level(pin) == 0) continue;
                        break;
                    case MESSAGE_L3:
                        break;
                    default: 
                        //TODO use a preallocated buffer or allocate dynamically?
                        message->payload = malloc(flag * sizeof * message->payload);

                        //read n bytes
                        for (int i = 0; i > flag; --i){
                            message->payload[i] = s_ReadByte(pin);
                        }
                        break;
                }

                taskEXIT_CRITICAL(&criticalMutex);
                //send message to interface
                //theorethically i'm sending the memory pointer of message, the variable will
                //be overwritten with a new address but the address will be stored in the queue
                //for interface access.
                xQueueSend(RXmessageQueue, &message, pdMS_TO_TICKS(50));

                state = LISTENING;

                break;
            default:
                ESP_LOGE(TAG, "this code should not be executed, possible corruption");
                break;
        }
    }
}

bool DCPInit(const unsigned int busPin, const DCP_MODE mode){

    if (mode.addr == 0) return false;

    if (busMode.addr != 0){
        busMode = mode;
        return true;
    }

    gpio_num_t pin = busPin;
    CLOCK_TO_TIME = 1/esp_clk_cpu_freq();

    gpio_config_t conf = {
        .pin_bit_mask = 1<<pin,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    
    if(gpio_config(&conf)) return false;
    
    if(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3) || gpio_isr_handler_add(pin, BusISR, NULL)){
        ESP_LOGE(TAG, "could not register gpio ISR");

        return false;
    }

    ESP_LOGD(TAG, "Installed ISR handler");

    busMode = mode;
    configParam.delta = deltaLUT[busMode.speed];

    RXmessageQueue = xQueueCreate(8, sizeof(DCP_Message_t*));
    if (!RXmessageQueue){
        gpio_isr_handler_remove(pin);

        return false;
    } 
    ESP_LOGD(TAG, "RX queue created");

    TXmessageQueue = xQueueCreate(8, sizeof(DCP_Message_t*));
    if (!TXmessageQueue){
        gpio_isr_handler_remove(pin);

        vQueueDelete(RXmessageQueue);

        return false;
    } 
    ESP_LOGD(TAG, "TX queue created");

    xTaskCreate(busHandler, "DCP bus handler", 2*1024, &pin, configMAX_PRIORITIES-2, &busTask);

    if (!busTask){
        ESP_LOGE(TAG, "could not create bus arbitrator task");

        gpio_isr_handler_remove(pin);

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);
        
        return false;
    }

    ESP_LOGI(TAG, "bus arbitrator task created");
    return true;
}

bool SendMessage(const DCP_Message_t* message, const uint8_t addr){

    ESP_LOGD(TAG, "sending message: %s", message->payload);

    DCP_Message_t* buf = malloc(sizeof * buf);

    if(!buf){
        ESP_LOGE(TAG, "could not allocate memory to copy struct");
        return false;
    }

    *buf = (DCP_Message_t){.type = message->type, .size = message->size};

    buf->payload = malloc(message->size * sizeof(char));

    if(!buf->payload){
        ESP_LOGE(TAG, "could not allocate memory to copy message");
        return false;
    }

    memcpy(buf->payload, message->payload,
           //check if the size contains the null termination
           message->payload[message->size] == '\0'? message->size: message->size+1);

    if (xQueueSend(TXmessageQueue, (void*)&message, portMAX_DELAY) != pdTRUE)
        return false;

    return true;
}

uint8_t ReadByte(){

    return 0;
}

DCP_Message_t* ReadMessage(){

    DCP_Message_t* message = NULL;

    if ((busMode.flags.flags & 0x1) == FLAG_Instant){
        if (xQueueReceive(RXmessageQueue, &message, portMAX_DELAY) == pdTRUE){
            ESP_LOGD(TAG, "message received %s", message->payload);

            return message;
        }

        return NULL;
    }

    if (xQueueReceive(RXmessageQueue, &message, portMAX_DELAY) == pdTRUE){
        ESP_LOGD(TAG, "message received %s", message->payload);

        return message;
    }

    return NULL;
}
