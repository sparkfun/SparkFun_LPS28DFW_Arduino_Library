#include "SparkFun_LPS28DFW_Arduino_Library.h"

// TODO - Delete once API is updated
// This is a temporary fix due to a bug in lps28dfw_reference_mode_set(), where
// the second read function needs to be a write. An issue has been submitted,
// and this should be removed once the API is changed.
static int32_t lps28dfw_reference_mode_set_temp(stmdev_ctx_t *ctx, lps28dfw_ref_md_t *val)
{
  lps28dfw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps28dfw_read_reg(ctx, LPS28DFW_INTERRUPT_CFG,
                          (uint8_t *)&interrupt_cfg, 1);
  if (ret == 0)
  {

    interrupt_cfg.autozero = val->get_ref;
    interrupt_cfg.autorefp = (uint8_t)val->apply_ref & 0x01U;

    interrupt_cfg.reset_az  = ((uint8_t)val->apply_ref & 0x02U) >> 1;
    interrupt_cfg.reset_arp = ((uint8_t)val->apply_ref & 0x02U) >> 1;

    ret = lps28dfw_write_reg(ctx, LPS28DFW_INTERRUPT_CFG,
                            (uint8_t *)&interrupt_cfg, 1);
  }
  return ret;
}

// TODO - Delete once API is updated
// This is a temporary fix due to a bug in lps28dfw_int_on_threshold_mode_set(),
// where the second read function needs to be a write. An issue has been
// submitted, and this should be removed once the API is changed.
static int32_t lps28dfw_int_on_threshold_mode_set_temp(stmdev_ctx_t *ctx,
                                           lps28dfw_int_th_md_t *val)
{
  lps28dfw_interrupt_cfg_t interrupt_cfg;
  lps28dfw_ths_p_l_t ths_p_l;
  lps28dfw_ths_p_h_t ths_p_h;
  uint8_t reg[3];
  int32_t ret;

  ret = lps28dfw_read_reg(ctx, LPS28DFW_INTERRUPT_CFG, reg, 3);
  if (ret == 0)
  {
    memcpy((uint8_t *)&interrupt_cfg, &reg[0], 1);
    memcpy((uint8_t *)&ths_p_l, &reg[1], 1);
    memcpy((uint8_t *)&ths_p_h, &reg[2], 1);

    interrupt_cfg.phe = val->over_th;
    interrupt_cfg.ple = val->under_th;
    ths_p_h.ths = (uint8_t)(val->threshold / 256U);
    ths_p_l.ths = (uint8_t)(val->threshold - (ths_p_h.ths * 256U));

    memcpy(&reg[0], (uint8_t *)&interrupt_cfg, 1);
    memcpy(&reg[1], (uint8_t *)&ths_p_l, 1);
    memcpy(&reg[2], (uint8_t *)&ths_p_h, 1);

    ret = lps28dfw_write_reg(ctx, LPS28DFW_INTERRUPT_CFG, reg, 3);
  }
  return ret;
}

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

int32_t LPS28DFW::init()
{
    return lps28dfw_init_set(&sensor, LPS28DFW_DRV_RDY);
}

int32_t LPS28DFW::boot()
{
    return lps28dfw_init_set(&sensor, LPS28DFW_BOOT);
}

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

int32_t LPS28DFW::getModeConfig(lps28dfw_md_t* config)
{
    memcpy(config, &modeConfig, sizeof(lps28dfw_md_t));
    return LPS28DFW_OK;
}

int32_t LPS28DFW::getStatus(lps28dfw_stat_t* status)
{
    return lps28dfw_status_get(&sensor, status);
}

int32_t LPS28DFW::getSensorData(lps28dfw_data_t* data)
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
    return lps28dfw_data_get(&sensor, &modeConfig, data);
}

int32_t LPS28DFW::setInterruptMode(lps28dfw_int_mode_t* intMode)
{
    return lps28dfw_interrupt_mode_set(&sensor, intMode);
}

int32_t LPS28DFW::enableInterrupts(lps28dfw_pin_int_route_t* intRoute)
{
    return lps28dfw_pin_int_route_set(&sensor, intRoute);
}

int32_t LPS28DFW::getInterruptStatus(lps28dfw_all_sources_t* status)
{
    return lps28dfw_all_sources_get(&sensor, status);
}

int32_t LPS28DFW::setFIFOConfig(lps28dfw_fifo_md_t* fifoConfig)
{
    return lps28dfw_fifo_mode_set(&sensor, fifoConfig);
}

int32_t LPS28DFW::getFIFOConfig(lps28dfw_fifo_md_t* fifoConfig)
{
    return lps28dfw_fifo_mode_get(&sensor, fifoConfig);
}

int32_t LPS28DFW::getFIFOLength(uint8_t* numData)
{
    return lps28dfw_fifo_level_get(&sensor, numData);
}

int32_t LPS28DFW::getFIFOData(lps28dfw_fifo_data_t* data, uint8_t numData)
{
    return lps28dfw_fifo_data_get(&sensor, numData, &modeConfig, data);
}

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

int32_t LPS28DFW::setReferenceMode(lps28dfw_ref_md_t* mode)
{
    // TODO - Change to lps28dfw_reference_mode_set() once API is updated
    // This is a temporary fix due to a bug in lps28dfw_reference_mode_set(),
    // where the second read function needs to be a write. An issue has been
    // submitted, and this should be changed once the API is changed.
    return lps28dfw_reference_mode_set_temp(&sensor, mode);
}

int32_t LPS28DFW::setThresholdMode(lps28dfw_int_th_md_t* mode)
{
    // Variable to track errors returned by API calls
    int32_t err = LPS28DFW_OK;

    // TODO - Change to lps28dfw_int_on_threshold_mode_set() once API is updated
    // This is a temporary fix due to a bug in lps28dfw_int_on_threshold_mode_set(),
    // where the second read function needs to be a write. An issue has been
    // submitted, and this should be changed once the API is changed.
    err = lps28dfw_int_on_threshold_mode_set_temp(&sensor, mode);
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

void LPS28DFW::msDelay(uint32_t period)
{
    delay(period);
}