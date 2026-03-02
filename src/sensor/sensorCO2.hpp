#ifndef _SENSOR_CO2_H
#define _SENSOR_CO2_H

#define CO2
#ifdef CO2

#include "sensorClass.hpp"
#include <SensirionI2CScd4x.h>
#include <Wire.h>

#define SENSOR_SCD4X_I2C_ADDR 0x62

class sensorCO2 : public sensorClass
{
public:
    sensorCO2() : sensorClass("CO2"){};
    ~sensorCO2(){};

    uint16_t init(uint16_t reg, bool i2c_available);
    bool connected();
    bool sample();

    enum
    {
        TEMPERATURE,
        HUMIDITY,
        CO2,
        SERIAL_HIGH,
        SERIAL_LOW,
        MAX
    };

private:
    SensirionI2cScd4x _scd4x; // IIC
    uint64_t _serial;
};

uint16_t sensorCO2::init(uint16_t reg, bool i2c_available)
{
    uint16_t t_reg = reg;
    for (uint16_t i = 0; i < sensorCO2::MAX; i++)
    {
        sensorClass::reg_t value;
        value.addr = t_reg;
        value.type = sensorClass::regType_t::REG_TYPE_S32_ABCD;
        value.value.s32 = 0;
        m_valueVector.emplace_back(value);
        t_reg += sensorClass::valueLength(value.type);
    }

    if (!i2c_available)
    {
        _connected = false;
        return t_reg - reg;
    }
    GROVE_SWITCH_IIC;
    Wire.begin();
    Wire.beginTransmission(SENSOR_SCD4X_I2C_ADDR);
    if (Wire.endTransmission() != 0)
    {
        _connected = false;
        return t_reg - reg;
    }

    uint64_t serial;
    uint32_t timeout = 5000;

    _scd4x.begin(Wire,SENSOR_SCD4X_I2C_ADDR);
    if (_scd4x.stopPeriodicMeasurement())
    {
        _connected = false;
        return t_reg - reg;
    }
    if (_scd4x.getSerialNumber(_serial))
    {
        _connected = false;
        return t_reg - reg;
    }
    if (_scd4x.setAutomaticSelfCalibrationEnabled(0))
    {
        _connected = false;
        return t_reg - reg;
    }
    if (_scd4x.startPeriodicMeasurement())
    {
        _connected = false;
        return t_reg - reg;
    }

    _connected = true;

    return t_reg - reg;
}

bool sensorCO2::sample()
{
    GROVE_SWITCH_IIC;
    uint16_t error = 0;
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;

    error = _scd4x.readMeasurement(co2, temperature, humidity);

    if (error == 0)
    {
        m_valueVector[CO2].value.s32 = co2 * SCALE;
        m_valueVector[HUMIDITY].value.s32 = humidity * SCALE;
        m_valueVector[TEMPERATURE].value.s32 = temperature * SCALE;

        m_valueVector[SERIAL_HIGH].value.s32 = (int32_t)(_serialNumber >> 32);
        m_valueVector[SERIAL_LOW].value.s32 = (int32_t)(_serialNumber & 0xFFFFFFFF);
    }

    return true;
}

bool sensorCO2::connected()
{
    return _connected;
}

#endif /* #ifdef CO2 */

#endif /* #ifndef _SENSOR_CO2_H */