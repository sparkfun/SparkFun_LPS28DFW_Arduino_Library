#ifndef __SPARKFUN_LPS28DFW_H__
#define __SPARKFUN_LPS28DFW_H__

#include "Arduino.h"
#include <Wire.h>

#include "lps28dfw_api/lps28dfw_reg.h"

// Error codes
#define LPS28DFW_OK                 INT32_C(0)
#define LPS28DFW_E_NOT_CONNECTED    INT32_C(-1)
#define LPS28DFW_E_COM_FAIL         INT32_C(-2)

// Define I2C addresses. LPS28DFW_I2C_ADD_L/H are 8-bit values, need 7-bit
#define LPS28DFW_I2C_ADDRESS_DEFAULT (LPS28DFW_I2C_ADD_L >> 1)   // 0x5C
#define LPS28DFW_I2C_ADDRESS_SECONDARY (LPS28DFW_I2C_ADD_H >> 1) // 0x5D

// The enums within lps28dfw_reg.h require the scope to be explicitely specified
// in C++. These macros allow the user to skip the scope specification, reducing
// clutter in their code.
#define LPS28DFW_SEL_BY_HW          (lps28dfw_bus_mode_t::LPS28DFW_SEL_BY_HW)
#define LPS28DFW_INT_PIN_ON_I3C     (lps28dfw_bus_mode_t::LPS28DFW_INT_PIN_ON_I3C)
#define LPS28DFW_AUTO               (lps28dfw_bus_mode_t::LPS28DFW_AUTO)
#define LPS28DFW_ALWAYS_ON          (lps28dfw_bus_mode_t::LPS28DFW_ALWAYS_ON)
#define LPS28DFW_BUS_AVB_TIME_50us  (lps28dfw_bus_mode_t::LPS28DFW_BUS_AVB_TIME_50us)
#define LPS28DFW_BUS_AVB_TIME_2us   (lps28dfw_bus_mode_t::LPS28DFW_BUS_AVB_TIME_2us)
#define LPS28DFW_BUS_AVB_TIME_1ms   (lps28dfw_bus_mode_t::LPS28DFW_BUS_AVB_TIME_1ms)
#define LPS28DFW_BUS_AVB_TIME_25ms  (lps28dfw_bus_mode_t::LPS28DFW_BUS_AVB_TIME_25ms)
#define LPS28DFW_DRV_RDY            (lps28dfw_init_t::LPS28DFW_DRV_RDY)
#define LPS28DFW_BOOT               (lps28dfw_init_t::LPS28DFW_BOOT)
#define LPS28DFW_RESET              (lps28dfw_init_t::LPS28DFW_RESET)
#define LPS28DFW_1260hPa            (lps28dfw_md_t::LPS28DFW_1260hPa)
#define LPS28DFW_4000hPa            (lps28dfw_md_t::LPS28DFW_4000hPa)
#define LPS28DFW_ONE_SHOT           (lps28dfw_md_t::LPS28DFW_ONE_SHOT)
#define LPS28DFW_1Hz                (lps28dfw_md_t::LPS28DFW_1Hz)
#define LPS28DFW_4Hz                (lps28dfw_md_t::LPS28DFW_4Hz)
#define LPS28DFW_10Hz               (lps28dfw_md_t::LPS28DFW_10Hz)
#define LPS28DFW_25Hz               (lps28dfw_md_t::LPS28DFW_25Hz)
#define LPS28DFW_50Hz               (lps28dfw_md_t::LPS28DFW_50Hz)
#define LPS28DFW_75Hz               (lps28dfw_md_t::LPS28DFW_75Hz)
#define LPS28DFW_100Hz              (lps28dfw_md_t::LPS28DFW_100Hz)
#define LPS28DFW_200Hz              (lps28dfw_md_t::LPS28DFW_200Hz)
#define LPS28DFW_4_AVG              (lps28dfw_md_t::LPS28DFW_4_AVG)
#define LPS28DFW_8_AVG              (lps28dfw_md_t::LPS28DFW_8_AVG)
#define LPS28DFW_16_AVG             (lps28dfw_md_t::LPS28DFW_16_AVG)
#define LPS28DFW_32_AVG             (lps28dfw_md_t::LPS28DFW_32_AVG)
#define LPS28DFW_64_AVG             (lps28dfw_md_t::LPS28DFW_64_AVG)
#define LPS28DFW_128_AVG            (lps28dfw_md_t::LPS28DFW_128_AVG)
#define LPS28DFW_256_AVG            (lps28dfw_md_t::LPS28DFW_256_AVG)
#define LPS28DFW_512_AVG            (lps28dfw_md_t::LPS28DFW_512_AVG)
#define LPS28DFW_LPF_DISABLE        (lps28dfw_md_t::LPS28DFW_LPF_DISABLE)
#define LPS28DFW_LPF_ODR_DIV_4      (lps28dfw_md_t::LPS28DFW_LPF_ODR_DIV_4)
#define LPS28DFW_LPF_ODR_DIV_9      (lps28dfw_md_t::LPS28DFW_LPF_ODR_DIV_9)
#define LPS28DFW_BYPASS             (lps28dfw_fifo_md_t::LPS28DFW_BYPASS)
#define LPS28DFW_FIFO               (lps28dfw_fifo_md_t::LPS28DFW_FIFO)
#define LPS28DFW_STREAM             (lps28dfw_fifo_md_t::LPS28DFW_STREAM)
#define LPS28DFW_STREAM_TO_FIFO     (lps28dfw_fifo_md_t::LPS28DFW_STREAM_TO_FIFO)
#define LPS28DFW_BYPASS_TO_STREAM   (lps28dfw_fifo_md_t::LPS28DFW_BYPASS_TO_STREAM)
#define LPS28DFW_BYPASS_TO_FIFO     (lps28dfw_fifo_md_t::LPS28DFW_BYPASS_TO_FIFO)
#define LPS28DFW_OUT_AND_INTERRUPT  (lps28dfw_ref_md_t::LPS28DFW_OUT_AND_INTERRUPT)
#define LPS28DFW_ONLY_INTERRUPT     (lps28dfw_ref_md_t::LPS28DFW_ONLY_INTERRUPT)
#define LPS28DFW_RST_REFS           (lps28dfw_ref_md_t::LPS28DFW_RST_REFS)

// Struct to hold data about the communication interface being used (I2C or SPI)
struct LPS28DFW_InterfaceData
{
    // I2C settings
    uint8_t i2cAddress;
    TwoWire* i2cPort;
};

class LPS28DFW
{
    public:
        // Constructor
        LPS28DFW();

        // Sensor initialization, must specify communication interface
        int32_t begin(uint8_t address = LPS28DFW_I2C_ADDRESS_DEFAULT, TwoWire& wirePort = Wire);

        // Configuration control
        int32_t init();
        int32_t boot();
        int32_t reset();
        int32_t setModeConfig(lps28dfw_md_t* mode);
        int32_t getModeConfig(lps28dfw_md_t* mode);
        int32_t getStatus(lps28dfw_stat_t* status);

        // Data acquisistion
        int32_t getSensorData();

        // Interrupt control
        int32_t setInterruptMode(lps28dfw_int_mode_t* intMode);
        int32_t enableInterrupts(lps28dfw_pin_int_route_t* intRoute);
        int32_t getInterruptStatus(lps28dfw_all_sources_t* status);

        // FIFO control
        int32_t setFIFOConfig(lps28dfw_fifo_md_t* fifoConfig);
        int32_t getFIFOConfig(lps28dfw_fifo_md_t* fifoConfig);
        int32_t getFIFOLength(uint8_t* numData);
        int32_t getFIFOData(lps28dfw_fifo_data_t* data, uint8_t numData);
        int32_t flushFIFO();

        // Reference mode control
        int32_t setReferenceMode(lps28dfw_ref_md_t* mode);
        int32_t setThresholdMode(lps28dfw_int_th_md_t* mode);
        int32_t getReferencePressure(int16_t* pressRaw);
        
        // Latest measurement from the sensor
        lps28dfw_data_t data;

    private:
        // Read/write helper functions
        static int32_t readRegisters(void* interfacePtr, uint8_t regAddress, uint8_t* dataBuffer, uint16_t numBytes);
        static int32_t writeRegisters(void* interfacePtr, uint8_t regAddress, const uint8_t* dataBuffer, uint16_t numBytes);

        // Deley helper function
        static void msDelay(uint32_t period);

        // Reference to the sensor
        stmdev_ctx_t sensor;

        // Information about the selected communication interface (I2C)
        LPS28DFW_InterfaceData interfaceData;

        // Place to store mode config values
        lps28dfw_md_t modeConfig;
};

#endif