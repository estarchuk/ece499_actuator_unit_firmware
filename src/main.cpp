#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include <iostream>

#define nozzle_0 GPIO_NUM_4
#define nozzle_1 GPIO_NUM_5
#define nozzle_2 GPIO_NUM_6
#define nozzle_3 GPIO_NUM_7

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
    twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_10, GPIO_NUM_9, TWAI_MODE_NORMAL);
    twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    std::shared_ptr<isobus::CANHardwarePlugin> canDriver = std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming, &twaiFilter);
    std::shared_ptr<isobus::InternalControlFunction> myECU = nullptr; // A pointer to hold our InternalControlFunction

    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);
    isobus::CANHardwareInterface::set_periodic_update_interval(10); // Default is 4ms, but we need to adjust this for default ESP32 tick rate of 100Hz

    if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid())
    {
        ESP_LOGE("AgIsoStack", "Failed to start hardware interface, the CAN driver might be invalid");
    }

    isobus::NAME TestDeviceNAME(0);

    //! Consider customizing some of these fields, like the function code, to be representative of your device
    TestDeviceNAME.set_arbitrary_address_capable(true);
    TestDeviceNAME.set_industry_group(1);
    TestDeviceNAME.set_device_class(0);
    TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    TestDeviceNAME.set_identity_number(2);
    TestDeviceNAME.set_ecu_instance(0);
    TestDeviceNAME.set_function_instance(0);
    TestDeviceNAME.set_device_class_instance(0);
    TestDeviceNAME.set_manufacturer_code(1407);
    auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0);

    isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(0xEF00, propa_callback, nullptr);

    std::array<std::uint8_t, isobus::CAN_DATA_LENGTH> messageData = {1}; // Data is just all zeros

    isobus::CANNetworkManager::CANNetwork.send_can_message(0xEF00, messageData.data(), isobus::CAN_DATA_LENGTH, myECU);

    // Start threads after ISObus setup
    std::thread listener(listener_thread);
    std::thread actuator(actuator_thread);

    while (true)
    {
        // CAN stack runs in other threads. Do nothing forever.
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    isobus::CANHardwareInterface::stop();
}