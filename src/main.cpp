#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include <iostream>
#include <hal/uart_types.h>
#include <driver/uart.h>

#define nozzle_0 GPIO_NUM_4
#define nozzle_1 GPIO_NUM_5
#define nozzle_2 GPIO_NUM_6
#define nozzle_3 GPIO_NUM_7
#define isobus_tx GPIO_NUM_10
#define isobus_rx GPIO_NUM_9

// Global queues
std::queue<int> receiver = {};
std::queue<int> command_queue = {};
std::queue<int> actuator_queue = {};
std::mutex receiver_mutex;
std::mutex command_mutex;
std::mutex actuator_mutex;

int nozzle_state_0 = 0;
int nozzle_state_1 = 0;
int nozzle_state_2 = 0;
int nozzle_state_3 = 0;

void gpio_setup(void){
    // Reset GPIO pins, preventing unintended behaviour
    gpio_reset_pin(nozzle_0);
    gpio_reset_pin(nozzle_1);
    gpio_reset_pin(nozzle_2);
    gpio_reset_pin(nozzle_3);

    // Set GPIO pin direction
    gpio_set_direction(nozzle_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(nozzle_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(nozzle_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(nozzle_3, GPIO_MODE_OUTPUT);

    // Set initial nozzle status (off) (unclear if 0 or 1, leaving as 0 for now)
    gpio_set_level(nozzle_0, 0);
    gpio_set_level(nozzle_1, 0);
    gpio_set_level(nozzle_2, 0);
    gpio_set_level(nozzle_3, 0);

}

void propa_callback(const isobus::CANMessage &CANMessage, void *){

    std::cout << CANMessage.get_data_length() << std::endl;
    
    // Both of these methods might work, needs testing
    std::vector message_data = CANMessage.get_data();
    receiver.push(message_data.at(0));

    receiver.push(CANMessage.get_data().at(0));

}

void listener_thread(void){

    while(1){
        if(!receiver.empty()){
            receiver_mutex.lock();
            command_mutex.lock();
            command_queue = receiver;
            receiver = {};
            command_mutex.unlock();
            receiver_mutex.unlock();
        }
        // 10 ms delay, required otherwise the MCU locks up
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void actuator_thread(void){

    while(1){
        if(!command_queue.empty()){
            command_mutex.lock();
            actuator_queue = command_queue;
            command_queue = {};
            command_mutex.unlock();
        }

        // Replace this with a while loop?
        if(!actuator_queue.empty()){
            int command = actuator_queue.front();
            actuator_queue.pop();
            switch(command){
                case 0:
                    gpio_set_level(nozzle_0, !nozzle_state_0);
                    nozzle_state_0 = !nozzle_state_0;
                    break;
                default:
                    // Do nothing if invalid command received
                    break;
            }
        }
        // 10 ms delay, required otherwise the MCU locks up
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}



// Would like to move setup to function but too much local stuff at this time (10 July 2024)
extern "C" void app_main()
{
    gpio_setup();
    
    const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, isobus_tx, isobus_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, \
                                            uart_buffer_size, 10, &uart_queue, 0));

    uint8_t data[128];
    int length = 0;

    while (true)
    {
        uart_get_buffered_data_len(uart_num, (size_t*)&length);
        // CAN stack runs in other threads. Do nothing forever.
        if(uart_read_bytes(uart_num, data, length, 100)){
            printf("%d\n", data[0]);
            switch(data[0]){
                case 0:
                    gpio_set_level(nozzle_0, !nozzle_state_0);
                    nozzle_state_0 = !nozzle_state_0;
                    break;
                case 1:
                    gpio_set_level(nozzle_1, !nozzle_state_1);
                    nozzle_state_1 = !nozzle_state_1;
                    break;
                case 2:
                    gpio_set_level(nozzle_2, !nozzle_state_2);
                    nozzle_state_2 = !nozzle_state_2;
                    break;
                case 3:
                    gpio_set_level(nozzle_3, !nozzle_state_3);
                    nozzle_state_3 = !nozzle_state_3;
                    break;
                default:
                    break;
            }
        }

        //length = uart_read_bytes(uart_num, data, length, 100);
        //printf("%s\n", data);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    isobus::CANHardwareInterface::stop();
}