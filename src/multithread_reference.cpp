#include <iostream>
#include <string>
#include <thread>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   
#include <mutex>
#include <queue>

std::queue<int> queue_in = {};
std::queue<int> queue_copy = {};
std::queue<int> receiver = {};
std::mutex queue_mutex;
std::mutex copy_mutex;
bool received = true;

void thread_test1(void){
    while(1){
        if(!receiver.empty()){
            queue_mutex.lock();
            std::cout << "I'm writing to the queue so hard right now" << std::endl;
            queue_in = receiver;
            receiver = {};
            queue_mutex.unlock();
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void){

    std::thread tester1(thread_test1);
    int i = 0;
    
    while(1){   
        if(!queue_in.empty()){
            queue_mutex.lock();
            copy_mutex.lock();
            std::cout << "Taking queue and dumping" << std::endl;
            queue_copy = queue_in;
            queue_in = {};
            std::queue copied = queue_copy;
            while(!copied.empty()){
                std::cout << "QUEUE COPY" << std::endl;
                std::cout << copied.front() << std::endl;
                copied.pop();
            }
            copy_mutex.unlock();
            queue_mutex.unlock();
        }
        if(i % 10 == 0){
            receiver.push(i);
            receiver.push(i+1);
            receiver.push(i+2);
            std::queue copied = receiver;
            while(!copied.empty()){
                std::cout << "RECEIVER" << std::endl;
                std::cout << copied.front() << std::endl;
                copied.pop();
            }

        }
        i++;
        std::cout <<  i << std::endl;
        vTaskDelay(10/portTICK_PERIOD_MS);

    }
    

    exit(0);

}