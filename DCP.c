#include "DCP.h"
#include <assert.h>

/*!
 * @brief generic definition of function that delays for microsseconds
 * @param delayUS time to delay in microsseconds 
 */
static void Delay(const uint32_t delayNS);

#include <rom/ets_sys.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <driver/gpio.h>
#include <esp_private/esp_clk.h>
#include <esp_log.h>

static char* TAG = "DCP Driver";

portMUX_TYPE criticalMutex = portMUX_INITIALIZER_UNLOCKED;
volatile DCP_MODE busMode;

QueueHandle_t RXmessageQueue = NULL;
QueueHandle_t TXmessageQueue = NULL;

SemaphoreHandle_t isrSemaphore = NULL;

static const float deltaLUT[] = {20, 4, 2.5, 1.25};
volatile struct {
    float delta;
} configParam;

static uint32_t CLOCK_TO_TIME;

/*
 * THIS FUNCTION SHOULD ONLY BE CALLED IN A CRITICAL SECTION
 *
 */
static __attribute__((always_inline)) void Delay(const uint32_t delayNS){
    esp_cpu_set_cycle_count(0);

    const esp_cpu_cycle_count_t ticks = delayNS/CLOCK_TO_TIME;

    //TODO overflow is assumed to not happen, it won't always be the case
    while (esp_cpu_get_cycle_count() < ticks)
        asm ("nop");
}

static void BusISR(void* arg){
    (void)arg;
    xSemaphoreGiveFromISR(isrSemaphore, NULL);
}

static bool s_ReadBit(const gpio_num_t pin){
    
    //reading high time
    esp_cpu_set_cycle_count(0);
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
    enum {STARTING, LISTENING, SENDING, WAITING, READING, END_} state = STARTING;

    //precalculation
    const float listeningDelayPartial = configParam.delta / 4.0;
    const unsigned int syncDelayPartial = busMode.isController? 
                                            25*configParam.delta:
                                            50*configParam.delta;
    uint8_t senderAddr, receiverAddr;

    DCP_Message_t *message = NULL;

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

                Delay((busMode.addr+6)*listeningDelayPartial);

                __attribute__((fallthrough));
            case STARTING:
                //starting communication
                if(gpio_get_level(pin) == 0){
                    state = WAITING;
                    continue;
                }

                //TODO check return of vvvvvvvv
                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                gpio_set_level(pin, 0);

                Delay(syncDelayPartial);

                gpio_set_direction(pin, GPIO_MODE_INPUT);

                //TODO what if the bus is still low?
                __attribute__((fallthrough));
            case SENDING:
                //sending actual data
                assert(message != NULL);

                gpio_set_direction(pin, GPIO_MODE_INPUT);

                switch(message->type){
                    case MESSAGE_SYNC:
                        break;
                    case MESSAGE_L3:
                        break;
                    default:
                        for (int i = 0; i < message->size; ++i){
                            //TODO check for endianess
                            for (int j = 8; j != 0; --j){
                                gpio_set_direction(pin, GPIO_MODE_INPUT);
                                Delay(2*configParam.delta);

                                //collision
                                if (gpio_get_level(pin) == 0){
                                    state = LISTENING;
                                    continue;
                                }

                                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                                gpio_set_level(pin, 0);

                                //bus modulation
                                // if bit == 0: low for 2 delta
                                // else: low for 1 delta
                                Delay(((message->payload[i] >> j) & 0x1) == 0?
                                        configParam.delta:
                                        2*configParam.delta);
                                gpio_set_direction(pin, GPIO_MODE_INPUT);

                                //collision
                                if(xSemaphoreTake(isrSemaphore, 0) == pdTRUE){
                                    state = LISTENING;
                                    continue;
                                }
                            }
                        }
                }

                gpio_set_direction(pin, GPIO_MODE_INPUT);

                __attribute__((fallthrough));
            case WAITING:
                //waiting for messages to send/receive
                state = WAITING;

                //message to read
                if(xSemaphoreTake(isrSemaphore, 1) == pdTRUE){
                    state = READING;
                }
                 
                //message to send
                if (xQueueReceive(TXmessageQueue, &(message), 1) == pdPASS){
                    state = LISTENING;
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

bool DCPInit(int busPin, DCP_MODE mode){

    CLOCK_TO_TIME = 1/esp_clk_cpu_freq();

    gpio_config_t conf = {
        .pin_bit_mask = 1<<busPin,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    
    if(gpio_config(&conf)) return false;
    
    isrSemaphore = xSemaphoreCreateBinary();
    if (isrSemaphore == NULL){
        ESP_LOGE(TAG, "could not create ISR semaphore");

        return false;
    }

    if(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL6)) return false;
    if(gpio_isr_handler_add(busPin, &BusISR, NULL)){
        ESP_LOGE(TAG, "could not register gpio ISR");

        vSemaphoreDelete(isrSemaphore);

        return false;
    }

    ESP_LOGD(TAG, "Installed ISR handler");

    busMode = mode;
    configParam.delta = deltaLUT[busMode.speed];

    RXmessageQueue = xQueueCreate(8, sizeof(DCP_Message_t));
    if (!RXmessageQueue){
        gpio_isr_handler_remove(busPin);

        vSemaphoreDelete(isrSemaphore);

        return false;
    } 
    ESP_LOGD(TAG, "RX queue created");

    TXmessageQueue = xQueueCreate(8, sizeof(DCP_Message_t));
    if (!TXmessageQueue){
        gpio_isr_handler_remove(busPin);

        vSemaphoreDelete(isrSemaphore);

        vQueueDelete(RXmessageQueue);

        return false;
    } 
    ESP_LOGD(TAG, "TX queue created");

    TaskHandle_t t = NULL;
    xTaskCreate(busHandler, "DCP bus handler", 2*1024, NULL, configMAX_PRIORITIES-2, &t);

    if (!t){
        ESP_LOGE(TAG, "could not create bus arbitrator task");

        gpio_isr_handler_remove(busPin);

        vSemaphoreDelete(isrSemaphore);

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);
        
        return false;
    }

    ESP_LOGI(TAG, "bus arbitrator task created");
    return true;
}

bool SendMessage(const DCP_Message_t message, const uint8_t addr){

    if (xQueueSend(TXmessageQueue, &message, portMAX_DELAY) != pdTRUE)
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
