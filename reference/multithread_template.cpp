#include <iostream>
#include <string>
#include <thread>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   
#include <mutex>
#include <queue>

std::queue<int> receiver = {};
std::queue<int> command_queue = {};
std::queue<int> actuator_queue = {};
std::mutex receiver_mutex;
std::mutex command_mutex;
std::mutex actuator_mutex;

// ISObus callback, simply read out message at this time
void propa_callback(const isobus::CANMessage &CANMessage, void *){
    std::cout << CANMessage.get_data_length() << std::endl;
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

        if(!actuator_queue.empty()){
            int command = actuator_queue.front();
            actuator_queue.pop();
            switch(command){
                case 0:
                    // toggle nozzle 0
                    break;
                default:
                    // Do nothing if invalid command received
                    break;
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void){

    // ISObus setup goes here


    std::thread listener(listener_thread);
    std::thread actuator(actuator_thread);
    
    while(1){   
        // Do nothing forever (subject to change)
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    
    exit(0);

}