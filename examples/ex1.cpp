// sudo g++ -o ex1 ex1.cpp ../ADS1X15.cpp -lpigpio
// sudo ./ex1

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
    ADS.config(1, 7);
    
    if (!ADS.begin())
    {
        cout<< ADS.errorMessage << endl;
        return 1;
    }
        
    while(1)
    {
        int16_t val_0 = ADS.readADC(0);  
        int16_t val_1 = ADS.readADC(1);  
        int16_t val_2 = ADS.readADC(2);  
        int16_t val_3 = ADS.readADC(3);  

        float f = ADS.toVoltage(1);  // voltage factor

        cout<< (float)val_0 * f<< ", ";
        cout<< (float)val_1 * f<< ", ";
        cout<< (float)val_2 * f<< ", ";
        cout<< (float)val_3 * f<< endl;

        std::this_thread::sleep_for(100ms);
    }
    
    return 0;
}