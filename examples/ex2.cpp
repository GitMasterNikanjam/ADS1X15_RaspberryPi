// sudo g++ -o ex2 ex2.cpp ../ADS1X15.cpp -lpigpio
// sudo ./ex2

// ###################################
// Include libraries:

#include <iostream>
#include "../ADS1X15.h"
#include <thread>

using namespace std;

// #######################################
// Define parameters:

#define ADS_ADDRESS         0X48
#define I2C_DEVICE_PATH     (const char*)"/dev/i2c-1"

// #####################################
// Global variables and objects:

ADS1115 ADS(ADS_ADDRESS, I2C_DEVICE_PATH);

// ######################################
// Function declerations:


// ###############################################
// main:

int main()
{

    ADS.config(1, 4, 4);

    if (!ADS.begin())
    {
        cout<< ADS.errorMessage << endl;
        return 1;
    }
        
    while(1)
    {
        ADS.handleConversionNoBlocking();

        cout<< ADS.value[0] << ", ";
        cout<< ADS.value[1] << ", ";
        cout<< ADS.value[2] << ", ";
        cout<< ADS.value[3] << endl;

        std::this_thread::sleep_for(10ms);
    }
    
    return 0;
}
