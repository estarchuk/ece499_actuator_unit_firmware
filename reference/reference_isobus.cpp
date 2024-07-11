#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include <iostream>


void propa_callback(const isobus::CANMessage &CANMessage, void *)
{
  std::cout << CANMessage.get_data_length() << std::endl;
}


extern "C" void app_main()
{
    twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);
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


    while (true)
    {
        // CAN stack runs in other threads. Do nothing forever.
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    isobus::CANHardwareInterface::stop();
}