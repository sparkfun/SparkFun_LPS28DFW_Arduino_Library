#include <Wire.h>
#include "SparkFun_LPS28DFW_Arduino_Library.h"

// Create a new sensor object
LPS28DFW pressureSensor;

// I2C address selection
uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_DEFAULT; // 0x5C
//uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_SECONDARY; // 0x5D
    
// Every method of the LPS28DFW class returns an error code indicating whether
// it completed successfully. These error codes are 8-bit signed integers,
// and each value has a specific meaning. 0 indicates success, negative values
// mean failure, and positive values are warnings. If you look in
// SparkFun_LPS28DFW_Arduino_Library.h, you'll find the following:
// 
// #define LPS28DFW_OK                 INT32_C(0)
// #define LPS28DFW_E_NOT_CONNECTED    INT32_C(-1)
// #define LPS28DFW_E_COM_FAIL         INT32_C(-2)
// 
// These can be used to handle different errors in different ways. For example,
// if a LPS28DFW_E_COM_FAIL is returned, the sensor didn't communicate properly,
// so you might be able to fix it by trying again
// 
// Because every method returns an error code, we'll create this global variable
// to track them.
int8_t err = LPS28DFW_OK;

// Here we set up a couple parameters for attempting to reconnect to the sensor
#define CONNECT_MAX_RETRIES 10
#define CONNECT_RETRY_INTERVAL 1000
uint8_t connectAttempts = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("LPS28DFW Example 6 - Error Handling");

    // Initialize the I2C library
    Wire.begin();

    // Try to connect and initialize the sensor
    connectToLPS28DFW();
}

void loop()
{
    // Get measurements from the sensor. This must be called before accessing
    // the pressure data, otherwise it will never update
    err = pressureSensor.getSensorData();
    handleError(err);
    
    // Print temperature and pressure
    Serial.print("Temperature (C): ");
    Serial.print(pressureSensor.data.heat.deg_c);
    Serial.print("\t\t");
    Serial.print("Pressure (hPa): ");
    Serial.println(pressureSensor.data.pressure.hpa);

    // Print once per second
    delay(1000);
}

// This is a helper function that will attempt to establish connect with the
// LPS28DFW. If a connection cannot be established, this will retry after a
// short delay. This will retry some number of times until giving up
void connectToLPS28DFW()
{
    // We're starting a new connection attempt, so increment the counter
    connectAttempts++;

    Serial.print("Trying to connect to the LPS28DFW, attempt number ");
    Serial.print(connectAttempts);
    Serial.print("/");
    Serial.print(CONNECT_MAX_RETRIES);
    Serial.println();

    // Try to connect to the sensor and initialize it
    err = pressureSensor.begin(i2cAddress);

    // Check the error to see whether it worked
    if(err == LPS28DFW_OK)
    {
        // We've successfully connected to the sensor!
        Serial.println("LPS28DFW connected!");

        // If the sensor has connection issues in the future, this function will
        // be called again, so we need to reset the connection attempts
        connectAttempts = 0;
        
        // Now we can simply return to where we came from
        return;
    }
    
    // There was a problem connecting, check whether we've maxed out our number
    // of attempts
    if(connectAttempts >= CONNECT_MAX_RETRIES)
    {
        // We've used all our connection attempts, so the problem is unlikely to
        // resovle itself. We'll just give up here
        Serial.println("Max connection attempts made.");
        freeze();
    }

    // We have more connection attempts remaining, see if the error handler can
    // fix the problem
    handleError(err);
}

// This is a helper function that will look at the latest error code and
// determine the best course of action.
void handleError(int8_t errorCode)
{
    switch(errorCode)
    {
        case LPS28DFW_OK:
            // No problem, we can just continue on
            break;
        case LPS28DFW_E_NOT_CONNECTED:
            // Not connected means we got communication with some device, but it
            // wasn't the LPS28DFW. This is not likely to fix itself, so we'll
            // just freeze the program
            Serial.println("Not connected!");
            freeze();
            break;
        case LPS28DFW_E_COM_FAIL:
            // Communication failure, this usually indicates the sensor is not
            // wired up correctly, or the I2C address is wrong. It's possible
            // for this to fix itself (eg. loose wires), so let's just print a
            // message and try again after a short delay
            Serial.println("Communication failure, check wiring and I2C address!");
            delay(CONNECT_RETRY_INTERVAL);
            connectToLPS28DFW();
            break;
        default:
            // An unknown error code was returned. This indicates an issue with
            // the code itself, so let's just print it and freeze
            Serial.print("Unknown error code! ");
            Serial.println(errorCode);
            freeze();
            break;
    }
}

// This is a helper function that freezes the entire program by entering an
// infinite loop. This should only be used in the event of a total failure that
// cannot be recovered from. It's also a good idea to print something before
// calling this to indicate what the problem was
void freeze()
{
    Serial.println("Freezing program. Read the messages above to find and fix the error!");
    while(1);
}