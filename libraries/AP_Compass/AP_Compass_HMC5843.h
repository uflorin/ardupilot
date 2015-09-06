/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HMC5843_H
#define AP_Compass_HMC5843_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AP_HMC5843_SerialBus
{
public:
    struct PACKED raw_value {
        int16_t val[3];
        uint8_t st2;
    };

    virtual uint8_t register_read(uint8_t address, uint8_t *value, uint8_t count) = 0;
    uint8_t register_read(uint8_t address) {
        uint8_t reg;
        register_read(address, &reg, 1);
        return reg;
    }
    virtual uint8_t register_write(uint8_t address, uint8_t value) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual bool configure() = 0;
    virtual bool start_measurements() = 0;
    virtual uint8_t read_raw(struct raw_value *rv) = 0;
    virtual uint32_t get_dev_id() = 0;
};

class AP_Compass_HMC5843 : public AP_Compass_Backend
{
private:
    float               calibration[3];
    bool                _initialised;
    bool                read_raw(void);
    uint8_t             _base_config;
    bool                re_initialise(void);
    bool                read_register(uint8_t address, uint8_t *value);
    bool                write_register(uint8_t address, uint8_t value);
    uint32_t            _retry_time; // when unhealthy the millis() value to retry at
    AP_HMC5843_SerialBus*_bus;
    AP_HAL::Semaphore*  _bus_sem;

    int16_t			    _mag_x;
    int16_t			    _mag_y;
    int16_t			    _mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t			    _accum_count;
    uint32_t            _last_accum_time;

    uint8_t             _compass_instance;
    uint8_t             _product_id;

public:
    AP_Compass_HMC5843(Compass &compass, AP_HMC5843_SerialBus *bus);
    bool        init(void);
    void        read(void);
    void        accumulate(void);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);
    static AP_Compass_Backend *detect_mpu9250(Compass &compass);

};

class AP_HMC5843_SerialBus_MPU9250: public AP_HMC5843_SerialBus
{
public:
    AP_HMC5843_SerialBus_MPU9250(AP_HAL::I2CDriver *i2c, uint8_t addr);
    uint8_t register_read(uint8_t address, uint8_t *value, uint8_t count);
    uint8_t register_write(uint8_t address, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool configure();
    bool start_measurements();
    uint8_t read_raw(struct raw_value *rv);
    uint32_t get_dev_id();
private:
    void _read(uint8_t address, uint8_t *value, uint32_t count);
    void _write(uint8_t address, const uint8_t *value,  uint32_t count);
    void _write(uint8_t address, const uint8_t value) {
        _write(address, &value, 1);
    }
    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;
    uint8_t _addr;
};

class AP_HMC5843_SerialBus_I2C: public AP_HMC5843_SerialBus
{
public:
    AP_HMC5843_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr);
    uint8_t register_read(uint8_t address, uint8_t *value, uint8_t count);
    uint8_t register_write(uint8_t address, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool configure(){ return true; }
    bool start_measurements() { return true; }
    uint8_t read_raw(struct raw_value *rv);
    uint32_t get_dev_id();
private:
    void _read(uint8_t address, uint8_t *value, uint32_t count);
    void _write(uint8_t address, const uint8_t *value,  uint32_t count);
    void _write(uint8_t address, const uint8_t value) {
        _write(address, &value, 1);
    }
    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;
    uint8_t _addr;
};
#endif
