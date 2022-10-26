#include <Wire.h>
#include "SparkFun_LPS28DFW_Arduino_Library.h"

// Create a new sensor object
LPS28DFW pressureSensor;

// I2C address selection
uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_DEFAULT; // 0x5C
//uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_SECONDARY; // 0x5D

// Pin used for interrupt detection
int interruptPin = 2;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

// Create a buffer for FIFO data
const uint16_t numSamples = 5;
lps28dfw_fifo_data_t fifoData[numSamples];

// Track FIFO length to give progress updates
uint8_t previousFIFOLength = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("LPS28DFW Example 4 - FIFO Buffer");

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

    // The LPS28DFW has multiple ODR (output data rate) settings available, from
    // 1Hz up to 200Hz. However it defaults to one-shot mode, where we have to
    // manually trigger measurements. For this example, we want the sensor to
    // make measurements at regular intervals, so we'll set the ODR to 1Hz. The
    // other parameters are defaults that can be ignored for this example
    lps28dfw_md_t modeConfig =
    {
        .fs  = LPS28DFW_1260hPa,    // Full scale range
        .odr = LPS28DFW_1Hz,        // Output data rate
        .avg = LPS28DFW_4_AVG,      // Average filter
        .lpf = LPS28DFW_LPF_DISABLE // Low-pass filter
    };
    pressureSensor.setModeConfig(&modeConfig);
    
    // Here we configure the FIFO buffer. including both the operation mode and
    // watermark level. The watermark can be set to a maximum of 127 samples.
    // The operation mode has several possible settings:
    // 
    // LPS28DFW_BYPASS - This disables the FIFO buffer
    // 
    // LPS28DFW_FIFO - Measurements are added to the FIFO buffer until it is
    // full (128 samples, or watermark level if it's non-zero). After filling,
    // new measurements are no longer added to the FIFO buffer
    // 
    // LPS28DFW_STREAM - Similar to LPS28DFW_FIFO, but measurements keep adding
    // to the buffer after it's full
    // 
    // LPS28DFW_BYPASS_TO_FIFO - FIFO buffer starts in LPS28DFW_BYPASS mode
    // until an interrupt occurs, then switches to LPS28DFW_FIFO mode
    // 
    // LPS28DFW_BYPASS_TO_STREAM - FIFO buffer starts in LPS28DFW_BYPASS mode
    // until an interrupt occurs, then switches to LPS28DFW_STREAM mode
    // 
    // LPS28DFW_STREAM_TO_FIFO - FIFO buffer starts in LPS28DFW_STREAM mode
    // until an interrupt occurs, then switches to LPS28DFW_FIFO mode
    lps28dfw_fifo_md_t fifoConfig =
    {
        .operation = LPS28DFW_STREAM,
        .watermark = numSamples
    };
    pressureSensor.setFIFOConfig(&fifoConfig);

    // Configure the LPS28DFW interrupt pin mode
    lps28dfw_int_mode_t intMode =
    {
        .int_latched  = 0, // Latching mode (not including data ready condition)
        .active_low   = 0, // Signal polarity
        .drdy_latched = 0  // Latching mode (data ready condition only)
    };
    pressureSensor.setInterruptMode(&intMode);

    // Configure the LPS28DFW interrupt conditions
    lps28dfw_pin_int_route_t intRoute =
    {
        .drdy_pres = 0, // Trigger interrupts when measurements finish
        .fifo_th   = 1, // Trigger interrupts when FIFO threshold is reached
        .fifo_ovr  = 0, // Trigger interrupts when FIFO overrun occurs
        .fifo_full = 0  // Trigger interrupts when FIFO is full
    };
    pressureSensor.enableInterrupts(&intRoute);

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), lps28dfwInterruptHandler, RISING);
}

void loop()
{
    // Get number of data samples currently stored in FIFO buffer
    uint8_t currentFIFOLength = 0;
    pressureSensor.getFIFOLength(&currentFIFOLength);

    // Check whether number of samples in FIFO buffer has changed
    if(previousFIFOLength != currentFIFOLength)
    {
        // Update FIFO length
        previousFIFOLength = currentFIFOLength;

        // Print current FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(currentFIFOLength);
        Serial.print("/");
        Serial.println(numSamples);
    }
    else if(currentFIFOLength == numSamples)
    {
        // If the buffer length goes beyond the watermark level, then an
        // interrupt was missed. This example will likely run into issues,
        // so we'll just clear the FIFO buffer
        Serial.println("Too many samples in FIFO buffer, flushing...");
        
        pressureSensor.flushFIFO();
    }

    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.println("Interrupt occurred!");

        // Get the interrupt status to know which condition triggered
        lps28dfw_all_sources_t interruptStatus;
        pressureSensor.getInterruptStatus(&interruptStatus);

        // Check if this is the FIFO watermerk interrupt condition
        if(interruptStatus.fifo_th)
        {
            // Get FIFO data from the sensor
            pressureSensor.getFIFOData(fifoData, numSamples);

            // Print out all acquired data
            for(uint16_t i = 0; i < numSamples; i++)
            {
                Serial.print("Sample number: ");
                Serial.print(i);
                Serial.print("\t\t");
                Serial.print("Pressure (Pa): ");
                Serial.println(fifoData[i].hpa);
            }
        }
        else
        {
            Serial.println("Wrong interrupt condition!");
        }
    }
}

void lps28dfwInterruptHandler()
{
    interruptOccurred = true;
}