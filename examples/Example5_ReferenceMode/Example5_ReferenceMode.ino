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

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("LPS28DFW Example 5 - Reference Mode");

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

    // We'll wait here for the user to set the reference pressure
    Serial.println("Enter any key to store the current pressure as the reference pressure");
    while(Serial.available() == 0) {}

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
    pressureSensor.setModeConfig(&modeConfig);

    // Here we make the sensor store the latest measurement as the reference
    // pressure, and all future measurements will have that pressure subtracted.
    // The .apply_ref can have the following values:
    // LPS28DFW_OUT_AND_INTERRUPT - Use reference pressure for measurements and threshold interrupts
    // LPS28DFW_ONLY_INTERRUPT    - Use reference pressure for threshold interrupts only
    // LPS28DFW_RST_REFS          - Reset reference pressure
    // The .get_ref is used to actually store the reference pressure
    lps28dfw_ref_md_t refConfig =
    {
        .apply_ref = LPS28DFW_OUT_AND_INTERRUPT,
        .get_ref = true
    };
    pressureSensor.setReferenceMode(&refConfig);

    // Now we can read the reference pressure that was just stored. The
    // getReferencePressure() will return the raw value store by the sensor,
    // which is either hPa*16 or hPa*8 if the full scale range is 1260hPa or
    // 4000hPa respectively
    int16_t refPressureRaw = 0;
    pressureSensor.getReferencePressure(&refPressureRaw);
    float refPressureHPa = refPressureRaw / 16.0; // Divide by 16 in 1260hPa range

    // Let's print out the reference pressure for the user, and wait a few
    // seconds so they can read it
    Serial.print("Reference pressure (hPa): ");
    Serial.println(refPressureHPa);
    Serial.print("Beginning in 3... ");
    delay(1000);
    Serial.print("2... ");
    delay(1000);
    Serial.print("1... ");
    delay(1000);
    Serial.println("Go!");

    // Configure the LPS28DFW to trigger interrupts when the measured pressure
    // exceeds some threshold relative to the reference pressure. The threshold
    // is equal to the pressure in hPa*16 or hPa*8 when the full scale range is
    // 1260hPa or 4000hPa respectively
    lps28dfw_int_th_md_t thresholdMode =
    {
        .threshold = 16, // Threshold above/below the reference pressure (eg. 16 = 1hPa in 1260hPa range)
        .over_th = true, // Enable the "over pressure" interrupt condition
        .under_th = true // Enable the "under pressure" interrupt condition
    };
    pressureSensor.setThresholdMode(&thresholdMode);

    // Configure the LPS28DFW interrupt pin mode
    lps28dfw_int_mode_t intMode =
    {
        .int_latched  = false, // Latching mode (not including data ready condition)
        .active_low   = false, // Signal polarity
        .drdy_latched = false  // Latching mode (data ready condition only)
    };
    pressureSensor.setInterruptMode(&intMode);

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), lps28dfwInterruptHandler, HIGH);
}

void loop()
{
    // Get measurements from the sensor. This must be called before accessing
    // the pressure data, otherwise it will never update
    pressureSensor.getSensorData();

    // Print temperature and pressure
    Serial.print("Temperature (C): ");
    Serial.print(pressureSensor.data.heat.deg_c);
    Serial.print("\t\t");
    Serial.print("Pressure (hPa): ");
    Serial.println(pressureSensor.data.pressure.hpa);

    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.print("Interrupt occurred!\t");

        // Get the interrupt status to know which condition triggered
        lps28dfw_all_sources_t interruptStatus;
        pressureSensor.getInterruptStatus(&interruptStatus);

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