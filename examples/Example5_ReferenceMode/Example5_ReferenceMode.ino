#include <Wire.h>
#include "SparkFun_LPS28DFW_Arduino_Library.h"

// Create a new sensor object
LPS28DFW pressureSensor;

// I2C address selection
uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_DEFAULT; // 0x5C
//uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_SECONDARY; // 0x5D

// Pin used for interrupt detection
int interruptPin = 5;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("LPS28DFW Example5 begin!");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x5C)
    while(pressureSensor.begin(i2cAddress) != LPS28DFW_OK)
    {
        // Not connected, inform user
        Serial.println("Error: LPS28DFW not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("LPS28DFW connected!");

    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    // The LPS28DFW has multiple ODR (output data rate) settings available, from
    // 1Hz up to 200Hz. However it defaults to one-shot mode, where we have to
    // manually trigger measurements. For this example, we want the sensor to
    // make measurements as fast as possible to catch any sudden changes in
    // pressure, so we'll use the max ODR of 200Hz. The other parameters are
    // defaults that can be ignored for this example
    lps28dfw_md_t modeConfig =
    {
        .fs  = LPS28DFW_1260hPa,    // Full scale range
        .odr = LPS28DFW_200Hz,      // Output data rate
        .avg = LPS28DFW_4_AVG,      // Average filter
        .lpf = LPS28DFW_LPF_DISABLE // Low-pass filter
    };
    err = pressureSensor.setModeConfig(&modeConfig);
    if(err != LPS28DFW_OK)
    {
        // Mode config failed, most likely an invalid frequncy (code -2)
        Serial.print("Mode config failed! Error code: ");
        Serial.println(err);
    }

    // Here we set the reference pressure 
    lps28dfw_ref_md_t refConfig =
    {
        .apply_ref = LPS28DFW_OUT_AND_INTERRUPT,
        .get_ref = 1
    };
    err = pressureSensor.setReferenceMode(&refConfig);
    if(err != LPS28DFW_OK)
    {
        // Mode config failed, most likely an invalid frequncy (code -2)
        Serial.print("Mode config failed! Error code: ");
        Serial.println(err);
    }

    // Configure the LPS28DFW to trigger interrupts when the measured pressure
    // exceeds some threshold relative to the reference pressure. The threshold
    // is equal to the pressure in hPa*16 or hPa*8 when the full scale range is
    // 1260hPa or 4000hPa respectively.
    lps28dfw_int_th_md_t thresholdMode =
    {
        .threshold = 16, // Threshold above/below the reference pressure
        .over_th = 1,    // Enable the "over pressure" interrupt condition
        .under_th = 1    // Enable the "under pressure" interrupt condition
    };
    err = pressureSensor.setThresholdMode(&thresholdMode);
    if(err != LPS28DFW_OK)
    {
        // Interrupt enable failed, most likely a communication error (code -2)
        Serial.print("Interrupt enable failed! Error code: ");
        Serial.println(err);
    }

    // Configure the LPS28DFW interrupt pin mode
    lps28dfw_int_mode_t intMode =
    {
        .int_latched  = 0, // Latching mode (not including data ready condition)
        .active_low   = 0, // Signal polarity
        .drdy_latched = 0  // Latching mode (data ready condition only)
    };
    err = pressureSensor.setInterruptMode(&intMode);
    if(err != LPS28DFW_OK)
    {
        // Interrupt mode failed, most likely a communication error (code -2)
        Serial.print("Interrupt mode failed! Error code: ");
        Serial.println(err);
    }

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), lps28dfwInterruptHandler, HIGH);
}

void loop()
{
    // Get measurements from the sensor
    lps28dfw_data_t data;
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if(err == LPS28DFW_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature (C): ");
        Serial.print(data.heat.deg_c);
        Serial.print("\t\t");
        Serial.print("Pressure (hPa): ");
        Serial.println(data.pressure.hpa);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor! Error code: ");
        Serial.println(err);
    }

    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.print("Interrupt occurred!\t");

        // Variable to track errors returned by API calls
        int8_t err = LPS28DFW_OK;

        // Get the interrupt status to know which condition triggered
        lps28dfw_all_sources_t interruptStatus;
        err = pressureSensor.getInterruptStatus(&interruptStatus);
        if(err != LPS28DFW_OK)
        {
            // Status get failed, most likely a communication error (code -2)
            Serial.print("Get interrupt status failed! Error code: ");
            Serial.println(err);
            return;
        }

        // Check if this is the "under pressure" or "over pressure" threshold
        // interrupt condition
        if(interruptStatus.under_pres)
        {
            Serial.println("Under pressure threshold triggered!");
        }
        else if(interruptStatus.over_pres)
        {
            Serial.println("Over pressure threshold triggered!");
        }
        else
        {
            Serial.println("Wrong interrupt condition!");
        }
    }

    // Print at 10Hz
    delay(100);
}

void lps28dfwInterruptHandler()
{
    interruptOccurred = true;
}