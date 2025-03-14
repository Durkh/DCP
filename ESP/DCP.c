#include "DCP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <freertos/queue.h>
#include "freertos/ringbuf.h"

#include <driver/gpio.h>
#include <esp_private/esp_clk.h>
#include <esp_log.h>
#include <rom/ets_sys.h>
#include "esp_cpu.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static char* TAG = "DCP Driver";

portMUX_TYPE criticalMutex = portMUX_INITIALIZER_UNLOCKED;
volatile DCP_MODE busMode;

QueueHandle_t RXmessageQueue = NULL;
QueueHandle_t TXmessageQueue = NULL;
RingbufHandle_t isrBuf = NULL;

TaskHandle_t busTask = NULL;

static const float deltaLUT[] = {20, 4, 2.5, 1.25};
volatile struct {
    float delta;    //transmission time unit
    float moe;      //transmission margin of error
    esp_cpu_cycle_count_t limits[2];
} configParam;

static double CLOCK_TO_TIME;

/*!
 * @brief generic definition of function that delays for microsseconds
 * @param ticks = delay in us * frequency in MHz
 */
static __attribute__((always_inline)) inline void Delay(const esp_cpu_cycle_count_t ticks){
    esp_cpu_set_cycle_count(0);

    taskENTER_CRITICAL(&criticalMutex);

    //TODO overflow is assumed to not happen, it won't always be the case
    while (esp_cpu_get_cycle_count() < ticks)
        asm volatile ("nop");

    taskEXIT_CRITICAL(&criticalMutex);
}

static inline bool s_ReadBit(const gpio_num_t pin){
    
    while (gpio_get_level(pin) == 0)
        continue;

    //reading high time
    esp_cpu_set_cycle_count(0);
    for (esp_cpu_cycle_count_t lim = configParam.limits[1] << 1; gpio_get_level(pin) == 1 && esp_cpu_get_cycle_count() < lim;)
        continue;

    return esp_cpu_get_cycle_count() <= configParam.limits[1]? 0: 1;
}

static uint8_t s_ReadByte(const gpio_num_t pin){

    uint8_t byte = 0;

    for (int i = 7; i >= 0; --i){
        byte |= s_ReadBit(pin) << i;
    }

    return byte;
}

static void BusISR(void* arg){
    const gpio_num_t pin = (gpio_num_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t data[0xFF];

    vTaskNotifyGiveFromISR(busTask, &xHigherPriorityTaskWoken);

    //reading incoming data
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_level(2, 0);

    //wait for SYNC to end
    while(gpio_get_level(pin) == 0) continue;

    gpio_set_level(2, 1);
    for (esp_cpu_set_cycle_count(0); gpio_get_level(pin) == 1; ){
        if(esp_cpu_get_cycle_count() > 10*configParam.limits[1]){
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            return;
        }
    }

    if(esp_cpu_get_cycle_count() <= 6*configParam.limits[0]){
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        return;
    }

    gpio_set_level(2, 0);

    data[0] = s_ReadByte(pin);
    const uint8_t flag = data[0]? data[0]: sizeof(struct DCP_Message_L3_t)+1;

    for (int i = 1; i < flag; ++i){
        gpio_set_level(2, 1);
        data[i] = s_ReadByte(pin);
        gpio_set_level(2, 0);
    }

    (void)xRingbufferSendFromISR(isrBuf, data, flag, 0);
    gpio_set_level(2, 1);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static inline bool s_SendBytes(gpio_num_t const pin, uint8_t const size, uint8_t const data[size], unsigned const delays[restrict 3]){

     for (int i = 0; i < size; ++i){
        for (int j = 7; j >= 0; --j){
            //bus modulation
            // if bit == 0: 1 delta high, 1 delta low
            // else: 2 delta high, 1 delta low

            gpio_set_direction(pin, GPIO_MODE_INPUT);

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
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
            gpio_set_level(pin, 0);

            Delay(delays[1] + delays[2]);
            gpio_set_direction(pin, GPIO_MODE_INPUT);

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
void _Noreturn busHandler(void* arg){

    const gpio_num_t pin = *((gpio_num_t*)arg);
    enum {STARTING, LISTENING, SENDING, WAITING, READING, END_} state = WAITING;

    //precalculations
    const uint32_t freqMHz = esp_clk_cpu_freq()/1e6;

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

    ESP_LOGV(TAG, "calculated delays:\n\tlistening: %lu cycles\n\tsync: %lu cycles\n\tbit 0: %lu cycles\n\tbit 1: %lu cycles", delays[0], delays[1], delays[2], delays[3]);

    //variables
    size_t rbSize;
    uint8_t* rbItem;
    DCP_Data_t message = {0};
    bool collision = false;

    assert(RXmessageQueue != NULL);
    assert(TXmessageQueue != NULL);

    gpio_set_direction(pin, GPIO_MODE_INPUT);

    while(1){

        gpio_set_level(2, 0);

        switch(state){
            case LISTENING:
                //listening bus for CSMA
                if(gpio_get_level(pin) == 0){
                    taskEXIT_CRITICAL(&criticalMutex);
                    state = WAITING;
                    continue;
                }

                //ESP_LOGV(TAG, "delaying");

                gpio_set_level(2, 0);
                ulTaskNotifyValueClear(busTask, UINT_MAX);
                //protocol piority delay
                //devices with smaller addresses will have the priority
                Delay(delays[0]);

                //while in the delay, did someone take the bus?
                if(ulTaskNotifyTake(pdTRUE, 0)){
                    taskEXIT_CRITICAL(&criticalMutex);
                    ESP_LOGV(TAG, "someone took the bus");
                    state = WAITING;
                    continue;
                }

                gpio_set_level(2, 1);
                __attribute__((fallthrough));
            case STARTING:
                //starting communication
                if(gpio_get_level(pin) == 0){
                    taskEXIT_CRITICAL(&criticalMutex);
                    state = WAITING;
                    continue;
                }

                //sync signal
                gpio_set_level(2, 0);
                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                gpio_set_level(pin, 0);

                Delay(delays[1]);

                //bit sync signal
                //high part
                gpio_set_direction(pin, GPIO_MODE_INPUT);
                Delay((uint32_t)(8 * delays[2]));

                //bit sync signal
                //low part
                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                gpio_set_level(pin, 0);
                Delay((uint32_t)(8 * delays[2]));

                gpio_set_level(2, 1);

                //leaving the bus still low not to interfere in the first bit
                __attribute__((fallthrough));
            case SENDING:
                //sending data
                assert(message.data != NULL);

                gpio_set_level(2, 0);

                collision = s_SendBytes(pin,
                                        message.message->type? message.message->type: sizeof(struct DCP_Message_t),
                                        message.data,
                                        (unsigned[3]){delays[2], delays[3], 150});

                taskEXIT_CRITICAL(&criticalMutex);

                if (collision){
                    ESP_LOGV(TAG, "Collision detected");
                    state = LISTENING;
                    continue;
                }

                gpio_set_level(2, 1);
                free(message.data);

                ESP_LOGV(TAG, "successfully sent message, going to wait mode");

                ulTaskNotifyValueClear(busTask, UINT_MAX);
                gpio_set_direction(pin, GPIO_MODE_INPUT);

                __attribute__((fallthrough));
            case WAITING:
                //waiting for messages to send/receive
                state = WAITING;

                //message to read
                if( (rbItem = xRingbufferReceive(isrBuf, &rbSize, 1)) != NULL ){
                    state = READING;
                    break;
                }
                 
                //message to send
                if (xQueueReceive(TXmessageQueue, &(message.data), 1) == pdPASS){
                    taskENTER_CRITICAL(&criticalMutex);
                    state = LISTENING;
                    break;
                }

                break;
            case READING:
                message.data = malloc(rbSize * sizeof(uint8_t));
                memmove(message.data, rbItem, rbSize);

                vRingbufferReturnItem(isrBuf, rbItem);

                xQueueSend(RXmessageQueue, &(message.data), pdMS_TO_TICKS(15));

                state = WAITING;
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

    gpio_set_direction(2, GPIO_MODE_OUTPUT);

    gpio_num_t pin = busPin;
    CLOCK_TO_TIME = 1.0/esp_clk_cpu_freq();

    gpio_config_t conf = {
        .pin_bit_mask = 1<<pin,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    
    if(gpio_config(&conf)) return false;
    
    RXmessageQueue = xQueueCreate(8, sizeof(uint8_t*));
    if (!RXmessageQueue){
        ESP_LOGE(TAG, "could not create RX message queue");

        return false;
    } 
    ESP_LOGD(TAG, "RX queue created");

    TXmessageQueue = xQueueCreate(8, sizeof(uint8_t*));
    if (!TXmessageQueue){
        ESP_LOGE(TAG, "could not create TX message queue");

        vQueueDelete(RXmessageQueue);

        return false;
    } 
    ESP_LOGD(TAG, "TX queue created");

    busMode = mode;
    configParam.delta = deltaLUT[busMode.speed];
    configParam.moe = .02*configParam.delta;

    configParam.limits[0] = ((configParam.delta - configParam.moe)*1e-6)/CLOCK_TO_TIME;
    configParam.limits[1] = ((configParam.delta + configParam.moe)*1e-6)/CLOCK_TO_TIME;

    ESP_LOGV(TAG, "transmission limits: [%lu ~ %lu]ticks", configParam.limits[0], configParam.limits[1]);
    ESP_LOGV(TAG, "transmission limits: [%.2f ~ %.2f]us", configParam.delta - configParam.moe, configParam.delta + configParam.moe);

    isrBuf = xRingbufferCreate(0xFF0, RINGBUF_TYPE_NOSPLIT);
    if(isrBuf == NULL){
        ESP_LOGE(TAG, "could not create ISR buffer");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        return false;
    }
    ESP_LOGD(TAG, "ISR ringbuffer created");

    if(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3) || gpio_isr_handler_add(pin, BusISR, (void*)pin)){
        ESP_LOGE(TAG, "could not register gpio ISR");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        vRingbufferDelete(isrBuf);

        return false;
    }
    ESP_LOGD(TAG, "Installed ISR handler");


    xTaskCreate(busHandler, "DCP bus handler", 2*1024, &pin, configMAX_PRIORITIES-2, &busTask);

    if (!busTask){
        ESP_LOGE(TAG, "could not create bus arbitrator task");

        vQueueDelete(RXmessageQueue);
        vQueueDelete(TXmessageQueue);

        gpio_uninstall_isr_service();

        vRingbufferDelete(isrBuf);

        return false;
    }
    ESP_LOGI(TAG, "bus arbitrator task created");


    return true;
}

bool SendMessage(const DCP_Data_t message){

#ifdef ESP_LOGD
    if (message.message->type){
        ESP_LOGD(TAG, "sending message: %s", message.message->generic.payload);
    } else {
        ESP_LOGD(TAG, "sending L3 message");
    }
#endif


    if (xQueueSend(TXmessageQueue, (void*)&(message.data), portMAX_DELAY) != pdTRUE){
        ESP_LOGE(TAG, "could not send message to queue");
        return false;
    }

    return true;
}

struct DCP_Message_t* ReadMessage(){

    DCP_Data_t message = {0};

    if ((busMode.flags.flags & 0x1) == FLAG_Instant){
        if (xQueueReceive(RXmessageQueue, &(message.data), 0) == pdTRUE){

#ifdef ESP_LOGD
            if (message.message->type){
                ESP_LOGD(TAG, "generic message received: %s", message.message->generic.payload);
            } else {
                    ESP_LOGD(TAG, "L3 message received: %x\t%x\t%x\t%x\t%x\t%x",
                             message.message->L3.data[0],
                             message.message->L3.data[1],
                             message.message->L3.data[2],
                             message.message->L3.data[3],
                             message.message->L3.data[4],
                             message.message->L3.data[5]
                             );
            }
#endif

            return message.message;
        }

        return NULL;
    }

    if (xQueueReceive(RXmessageQueue, &(message.data), portMAX_DELAY) == pdTRUE){

#ifdef ESP_LOGD
            if (message.message->type){
                ESP_LOGD(TAG, "generic message received: %s", message.message->generic.payload);
            } else {
                    ESP_LOGD(TAG, "L3 message received: %x\t%x\t%x\t%x\t%x\t%x",
                             message.message->L3.data[0],
                             message.message->L3.data[1],
                             message.message->L3.data[2],
                             message.message->L3.data[3],
                             message.message->L3.data[4],
                             message.message->L3.data[5]
                             );
            }
#endif

        return message.message;
    }

    return NULL;
}
