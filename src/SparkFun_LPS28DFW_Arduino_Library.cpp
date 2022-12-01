#include "SparkFun_LPS28DFW_Arduino_Library.h"

/// @brief Default constructor
LPS28DFW::LPS28DFW()
{
    // Set sensor helper fucntions
    sensor.read_reg = readRegisters;
    sensor.write_reg = writeRegisters;
    sensor.mdelay = msDelay;

    // Set default interface parameters
    interfaceData.i2cAddress = LPS28DFW_I2C_ADDRESS_DEFAULT;
    interfaceData.i2cPort = &Wire;
    sensor.handle = &interfaceData;
}

/// @brief Begin communication with the sensor over I2C
/// @param address I2C address of sensor
/// @param wirePort I2C port to use for communication, defaults to Wire
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::begin(uint8_t address, TwoWire& wirePort)
{
    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    // Set interface parameters as requested
    interfaceData.i2cAddress = address;
    interfaceData.i2cPort = &wirePort;

    // Check whether the sensor is actually connected
    lps28dfw_id_t chipID;
    err = lps28dfw_id_get(&sensor, &chipID);
    if(err != LPS28DFW_OK)
    {
        return err;
    }
    if(chipID.whoami != LPS28DFW_ID)
    {
        return LPS28DFW_E_NOT_CONNECTED;
    }

    // Sensor is connected, send soft reset to clear any provious configuration
    err = reset();
    if(err != LPS28DFW_OK)
    {
        return err;
    }

    // Send init command
    return init();
}

/// @brief Enables the BDU and IF_ADD_INC bits in the control registers
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::init()
{
    return lps28dfw_init_set(&sensor, LPS28DFW_DRV_RDY);
}

/// @brief Enables the BOOT bit in the control registers
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::boot()
{
    return lps28dfw_init_set(&sensor, LPS28DFW_BOOT);
}

/// @brief Tells the sensor to reset itself
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::reset()
{
    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    // Send reset command
    err = lps28dfw_init_set(&sensor, LPS28DFW_RESET);

    // Wait for reset to finish
    lps28dfw_stat_t status;
    do
    {
        err = getStatus(&status);
        if(err != LPS28DFW_OK)
        {
            return err;
        }
    } while(status.sw_reset);

    return LPS28DFW_OK;
}

/// @brief Sets operational parameters of the sensor, such as range and ODR
/// @param config Struct of configuration parameters, see lps28dfw_md_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::setModeConfig(lps28dfw_md_t* config)
{
    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    // Attempt to set the provided config
    err = lps28dfw_mode_set(&sensor, config);
    if(err != LPS28DFW_OK)
    {
        return err;
    }

    // Config set correctly, save config for use later
    memcpy(&modeConfig, config, sizeof(lps28dfw_md_t));

    return LPS28DFW_OK;
}

/// @brief Gets operational parameters of the sensor, such as range and ODR
/// @param config Struct of configuration parameters, see lps28dfw_md_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getModeConfig(lps28dfw_md_t* config)
{
    memcpy(config, &modeConfig, sizeof(lps28dfw_md_t));
    return LPS28DFW_OK;
}

/// @brief Gets sensor status bits, such as data ready, overrun, etc.
/// @param status Struct of status bits, see lps28dfw_stat_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getStatus(lps28dfw_stat_t* status)
{
    return lps28dfw_status_get(&sensor, status);
}

/// @brief Gets pressure data from the sensor. This must be called to update
/// the data struct
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getSensorData()
{
    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    // Check whether we're in one-shot mode
    if(modeConfig.odr == LPS28DFW_ONE_SHOT)
    {
        // We're in one-shot mode, trigger a measurement
        err = lps28dfw_trigger_sw(&sensor, &modeConfig);
        if(err != LPS28DFW_OK)
        {
            return err;
        }
    }

    // Wait for measurement to finish
    lps28dfw_stat_t status;
    do
    {
        err = getStatus(&status);
        if(err != LPS28DFW_OK)
        {
            return err;
        }
    } while(!status.end_meas);

    // Grab latest measurement
    return lps28dfw_data_get(&sensor, &modeConfig, &data);
}

/// @brief Sets interrupt pin to be active high/low and latched/pulsed
/// @param intMode Struct of config values, see lps28dfw_int_mode_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::setInterruptMode(lps28dfw_int_mode_t* intMode)
{
    return lps28dfw_interrupt_mode_set(&sensor, intMode);
}

/// @brief Enables the data ready and FIFO interrupt conditions
/// @param intRoute Struct of which conditions to enable, see lps28dfw_pin_int_route_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::enableInterrupts(lps28dfw_pin_int_route_t* intRoute)
{
    return lps28dfw_pin_int_route_set(&sensor, intRoute);
}

/// @brief Gets interrupt status flags
/// @param status Interrupt status flags, see lps28dfw_all_sources_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getInterruptStatus(lps28dfw_all_sources_t* status)
{
    return lps28dfw_all_sources_get(&sensor, status);
}

/// @brief Sets the FIFO config
/// @param fifoConfig FIFO config, see lps28dfw_fifo_md_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::setFIFOConfig(lps28dfw_fifo_md_t* fifoConfig)
{
    return lps28dfw_fifo_mode_set(&sensor, fifoConfig);
}

/// @brief Gets the FIFO config
/// @param fifoConfig FIFO config, see lps28dfw_fifo_md_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getFIFOConfig(lps28dfw_fifo_md_t* fifoConfig)
{
    return lps28dfw_fifo_mode_get(&sensor, fifoConfig);
}

/// @brief Gets the number of data samples stored in the FIFO buffer, up to 128
/// @param numData Number of data samples
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getFIFOLength(uint8_t* numData)
{
    return lps28dfw_fifo_level_get(&sensor, numData);
}

/// @brief Gets pressure data out of FIFO buffer
/// @param data Array of data structs, see lps28dfw_fifo_data_t
/// @param numData Number of data samples to read
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getFIFOData(lps28dfw_fifo_data_t* data, uint8_t numData)
{
    return lps28dfw_fifo_data_get(&sensor, numData, &modeConfig, data);
}

/// @brief Clears all data in FIFO buffer
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::flushFIFO()
{
    // Variable to track errors returned by API calls
    int8_t err = LPS28DFW_OK;

    // Get current FIFO config
    lps28dfw_fifo_md_t oldConfig;
    lps28dfw_fifo_md_t newConfig;
    err = lps28dfw_fifo_mode_get(&sensor, &oldConfig);
    if(err != LPS28DFW_OK)
    {
        return err;
    }

    // FIFO can be flushed by setting to bypass mode
    memcpy(&newConfig, &oldConfig, sizeof(lps28dfw_fifo_md_t));
    newConfig.operation = LPS28DFW_BYPASS;
    err = lps28dfw_fifo_mode_set(&sensor, &newConfig);
    if(err != LPS28DFW_OK)
    {
        return err;
    }

    // Restore original configuration
    return lps28dfw_fifo_mode_set(&sensor, &oldConfig);
}

/// @brief Sets reference mode, where measurements are relative to a reference
// pressure. If mode.get_ref is 1, this updates the reference pressure
/// @param mode Reference mode struct, see lps28dfw_ref_md_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::setReferenceMode(lps28dfw_ref_md_t* mode)
{
    return lps28dfw_reference_mode_set(&sensor, mode);
}

/// @brief Sets threshold interrupt, setReferenceMode() must be called first
/// @param mode Threshold interrupt mode, see lps28dfw_int_th_md_t
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::setThresholdMode(lps28dfw_int_th_md_t* mode)
{
    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    err = lps28dfw_int_on_threshold_mode_set(&sensor, mode);
    if(err != LPS28DFW_OK)
    {
        return err;
    }

    // lps28dfw_int_on_threshold_mode_set() doesn't set the INT_EN bit in
    // CTRL_REG4, so we need to do that manually. First grab the contents of the
    // CTRL_REG4 register
    lps28dfw_ctrl_reg4_t reg4;
    uint8_t reg[1];
    err = lps28dfw_read_reg(&sensor, LPS28DFW_CTRL_REG4, reg, 1);
    if(err != LPS28DFW_OK)
    {
        return err;
    }
    memcpy(&reg4, reg, 1);

    // Now set the INT_EN bit accordingly
    reg4.int_en = mode->under_th | mode->over_th;
    memcpy(reg, &reg4, 1);
    return lps28dfw_write_reg(&sensor, LPS28DFW_CTRL_REG4, reg, 1);

    // return LPS28DFW_OK;
}

/// @brief Gets the current reference pressure after calling setReferenceMode()
/// @param pressRaw Raw pressure value. Can convert to hPa by dividing by either
// 16 or 8 if the full scale range is 1260hPa or 4000hPa respectively
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::getReferencePressure(int16_t* pressRaw)
{
    return lps28dfw_refp_get(&sensor, pressRaw);
}

/// @brief Helper function to read sensor registers over I2C
/// @param interfacePtr Pointer to interface data, see LPS28DFW_InterfaceData
/// @param regAddress Start address to read
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to read
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::readRegisters(void* interfacePtr, uint8_t regAddress, uint8_t* dataBuffer, uint16_t numBytes)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return LPS28DFW_E_COM_FAIL;
    }

    // Get interface data
    LPS28DFW_InterfaceData* interfaceData = (LPS28DFW_InterfaceData*) interfacePtr;

    // Jump to desired register address
    interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);
    interfaceData->i2cPort->write(regAddress);
    if(interfaceData->i2cPort->endTransmission())
    {
        return LPS28DFW_E_COM_FAIL;
    }

    // Read bytes from these registers
    interfaceData->i2cPort->requestFrom(interfaceData->i2cAddress, (uint8_t) numBytes);

    // Store all requested bytes
    for(uint32_t i = 0; i < numBytes && interfaceData->i2cPort->available(); i++)
    {
        dataBuffer[i] = interfaceData->i2cPort->read();
    }

    return LPS28DFW_OK;
}

/// @brief Helper function to write sensor registers over I2C
/// @param interfacePtr Pointer to interface data, see LPS28DFW_InterfaceData
/// @param regAddress Start address to read
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to read
/// @return Error code. 0 means success, negative means failure
int32_t LPS28DFW::writeRegisters(void* interfacePtr, uint8_t regAddress, const uint8_t* dataBuffer, uint16_t numBytes)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return LPS28DFW_E_COM_FAIL;
    }
    // Get interface data
    LPS28DFW_InterfaceData* interfaceData = (LPS28DFW_InterfaceData*) interfacePtr;

    // Begin transmission
    interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);

    // Write the address
    interfaceData->i2cPort->write(regAddress);
    
    // Write all the data
    for(uint32_t i = 0; i < numBytes; i++)
    {
        interfaceData->i2cPort->write(dataBuffer[i]);
    }

    // End transmission
    if(interfaceData->i2cPort->endTransmission())
    {
        return LPS28DFW_E_COM_FAIL;
    }

    return LPS28DFW_OK;
}

/// @brief Helper function to delay for some amount of time
/// @param period Number of milliseconds to delay
void LPS28DFW::msDelay(uint32_t period)
{
    delay(period);
}