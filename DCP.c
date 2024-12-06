#include "DCP.h"
#include <assert.h>

/*!
 * @brief generic definition of function that delays for microsseconds
 * @param delayUS time to delay in microsseconds 
 */
void Delay(const unsigned int delayUS);

#ifdef __ESP32__

#include <rom/ets_sys.h>
#include<FreeRTOS/task.h>
#include<FreeRTOS/portmacro.h>
#include<driver/gpio>

portMUX_TYPE criticalMutex = portMUX_INITIALIZER_UNLOCKED;
volatile DCP_MODE busMode;
QueueHandle_t messageQueue = NULL;
SemaphoreHandle_t isrSemaphore = NULL;

static const float deltaLUT[] = {20, 4, 2.5, 1.25};
volatile struct {
    float delta;
} configParam;

/*
 * THIS FUNCTION SHOULD ONLY BE CALLED IN A CRITICAL SECTION
 *
 */
__attribute__((always_inline)) void Delay(const uint32_t delayNS){
    esp_cpu_set_cycle_count(0);

#if CONFIG_IDF_TARGET_ESP32C3
    const esp_cpu_cycle_count_t ticks = delayNS/6.25f;

    //TODO overflow is assumed to not happen, it won't always be the case
    while (esp_cpu_get_cycle_count() < ticks)
        volatile asm ("nop");
#endif
}

static void BusISR(void* arg){
    gpio_num_t pin = (gpio_num_t*)*arg;
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

    //TODO assert countL*K == DELTA

    return countH* == configParam.delta? 0: 1
}

/*!
 * @brief task that controls the state machine of the control of the bus
 *
 *
 */
void _Noreturn busHandler(void* arg){

    const gpio_num_t pin = (gpio_num_t*)*arg;
    enum {STARTING, LISTENING, SENDING, WAITING, READING, END_} state = STARTING;

    //precalculation
    const unsigned float listeningDelayPartial = configParam.delta / 4f;
    const unsigned int syncDelayPartial = mode.isController? 
                                            25*configParam.delta:
                                            50*configParam.delta;

    DCP_Message_t *message = NULL;

    gpio_set_direction(pin, GPIO_MODE_INPUT);

    while(1){
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

                switch(message.type){
                    case MESSAGE_SYNC:
                        break;
                    case MESSAGE_L3:
                        break;
                    default:
                        for (int i = 0; i < message.size; ++i){
                            //TODO check for endianess
                            for (int j = 8; j != 0; --j){
                                gpio_set_direction(pin, GPIO_MODE_INPUT);
                                delay(2*configParam.delta);

                                //collision
                                if (gpio_get_level(pin) == 0){
                                    state = LISTENING;
                                    continue;
                                }

                                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                                gpio_set_level(pin, 0);
                                delay((message.payload[i] >> j) & 0x1 == 0?
                                        configParam.delta:
                                        2*configParam.delta);
                                gpio_set_direction(pin, GPIO_MODE_INPUT);

                                //collision
                                if (gpio_get_level(pin) == 0){
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
                if (xQueueReceive(messageQueue, &(message), 1) == pdPass){
                    state = LISTENING;
                }
                break;
            case READING:
                //reading the address and/or data

                gpio_set_direction(pin, GPIO_MODE_INPUT);

                //wait for SYNC to end
                while(gpio_get_level(pin) == 0) continue;

                //TODO disable interrupions?

                //reading origin address

                //reading destination address?
                //check L3 package

                break;
        }
    }
}

bool Init(int busPin, DCP_MODE mode){

    gpio_config_t conf = {
        .pin_bit_mask = 1<<busPin,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    
    if(gpio_config(&conf)) return false;

    if(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL6)) return false;
    if(gpio_isr_handler_add(busPin, &BusISR, &busPin)){

        return false;
    }

    busMode = mode;
    configParam.delta = deltaLUT[busMode.speed];

    messageQueue = xQueueCreate(8, sizeof(DCP_Message_t));
    if (!messageQueue){
        //TODO uninstall ISR

        return false;
    } 

    TaskHandle_t t = NULL;
    xTaskCreate(busHandler, "DCP bus handler", 2*1024, NULL, 2, &t);
    if (!t){
        //TODO uninstall ISR
        //delete Queue
        
        return false;
    }

    return true;
}

    taskENTER_CRITICAL(&criticalMutex);
    taskEXIT_CRITICAL(&criticalMutex);
bool SendMessage(const DCP_Message_t message, const uint8_t addr){

    if (xQueueSend(messageQueue, &message, portMAX_DELAY) != pdTRUE)
        return false;

    return true;
}

uint8_t ReadByte(){

    if (instant){

        return 
    }



}

DCP_Message_t* ReadMessage(){
    if (instant){

        return 
    }

    //check for message
    


    return NULL;
}

#ifdef __RPICO__

#else

#endif


