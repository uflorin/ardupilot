/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_HMC5843.cpp - Arduino Library for HMC5843 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_HMC5843.h"
#include "../AP_InertialSensor/AP_InertialSensor_MPU9250.h"
#include "../AP_InertialSensor/AP_InertialSensor_MPU6000.h"

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

#define HMC5843_HXL                                      0x03

/* bit definitions for MPUREG_USER_CTRL */
#define MPUREG_USER_CTRL                                0x6A
/* Enable MPU to act as the I2C Master to external slave sensors */
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20

#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_I2c_BYPASS_EN                                0x02


#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode

#if !defined(HAL_INS_MPU60XX_I2C_ADDR)
#define HAL_INS_MPU60XX_I2C_ADDR 0x68
#endif

// constructor
AP_Compass_HMC5843::AP_Compass_HMC5843(Compass &compass, AP_HMC5843_SerialBus *bus):
    AP_Compass_Backend(compass),
    _retry_time(0),
    _bus_sem(NULL),
    _mag_x(0),
    _mag_y(0),
    _mag_z(0),
    _mag_x_accum(0),
    _mag_y_accum(0),
    _mag_z_accum(0),
    _accum_count(0),
    _last_accum_time(0),
    _compass_instance(0),
    _product_id(0),
    _bus(bus)
{}

// detect the sensor

AP_Compass_Backend *AP_Compass_HMC5843::detect(Compass &compass)
{
    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(compass,
                                                  new AP_HMC5843_SerialBus_I2C(
                                                  hal.i2c, COMPASS_ADDRESS));

    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_Backend *AP_Compass_HMC5843::detect_mpu9250(Compass &compass)
{
    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(compass,
                                                  new AP_HMC5843_SerialBus_MPU9250(
                                                  hal.i2c, COMPASS_ADDRESS));
    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// read_register - read a register value
bool AP_Compass_HMC5843::read_register(uint8_t address, uint8_t *value)
{
    if (_bus->register_read(address, value, 1) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_HMC5843::write_register(uint8_t address, uint8_t value)
{
    if (_bus->register_write(address, value) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_HMC5843::read_raw()
{
    //uint8_t buff[6];
    uint8_t *buff;
    
    struct AP_HMC5843_SerialBus::raw_value rv;

    if (_bus->read_raw(&rv) != 0) {
        hal.i2c->setHighSpeed(false);
        _retry_time = hal.scheduler->millis() + 1000;
        _bus_sem->give();
        return false;
    }
    
    buff = (uint8_t *)rv.val;

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[0]) << 8) | buff[1];
    //rx = rv.val[0];
    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        rz = (((int16_t)buff[2]) << 8) | buff[3];
        ry = (((int16_t)buff[4]) << 8) | buff[5];
        //rz = rv.val[1];
        //ry = rv.val[2];
    } else {
        ry = (((int16_t)buff[2]) << 8) | buff[3];
        rz = (((int16_t)buff[4]) << 8) | buff[5];
        //ry = rv.val[1];
        //rz = rv.val[2];
    }
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_HMC5843::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   if (_accum_count != 0 && (tnow - _last_accum_time) < 13333) {
	  // the compass gets new data at 75Hz
	  return;
   }

   if (!_bus_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _bus_sem->give();

   if (result) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
	  _mag_x_accum += _mag_x;
	  _mag_y_accum += _mag_y;
	  _mag_z_accum += _mag_z;
	  _accum_count++;
	  if (_accum_count == 14) {
		 _mag_x_accum /= 2;
		 _mag_y_accum /= 2;
		 _mag_z_accum /= 2;
		 _accum_count = 7;
	  }
	  _last_accum_time = tnow;
   }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_HMC5843::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_HMC5843::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->delay(10);

    _bus_sem = _bus->get_semaphore();
    if (!_bus_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get HMC5843 semaphore"));
    }
    
    if (!_bus->configure()) {
        hal.scheduler->panic(PSTR("HMC5843: Could not configure bus for HMC5843\n"));
    }

    // determine if we are using 5843 or 5883L
    _base_config = 0;
    if (!write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        _bus_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }
    if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        // a 5883L supports the sample averaging config
        _product_id = AP_COMPASS_TYPE_HMC5883L;
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0f / 1090;  // adjustment for runtime vs calibration gain
    } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
        _product_id = AP_COMPASS_TYPE_HMC5843;
    } else {
        // not behaving like either supported compass type
        _bus_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 25 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!write_register(ConfigRegA, PositiveBiasConfig))
            continue;      // compass not responding on the bus
        hal.scheduler->delay(50);

        // set gains
        if (!write_register(ConfigRegB, calibration_gain) ||
            !write_register(ModeRegister, SingleConversion))
            continue;

        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        hal.scheduler->delay(10);

        float cal[3];

         hal.console->printf_P(PSTR("mag %d %d %d\n"), _mag_x, _mag_y, _mag_z);
        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

         hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f\n"), cal[0], cal[1], cal[2]);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts > 2 &&
            cal[0] > 0.7f && cal[0] < 1.35f &&
            cal[1] > 0.7f && cal[1] < 1.35f &&
            cal[2] > 0.7f && cal[2] < 1.35f) {
             hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f good\n"), cal[0], cal[1], cal[2]);
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }

#if 1
        /* useful for debugging */
        hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
#endif
    }

    if (good_count >= 5) {
        /*
          The use of gain_multiple below is incorrect, as the gain
          difference between 2.5Ga mode and 1Ga mode is already taken
          into account by the expected_x and expected_yz values.  We
          are not going to fix it however as it would mean all
          APM1/APM2 users redoing their compass calibration. The
          impact is that the values we report on APM1/APM2 are lower
          than they should be (by a multiple of about 0.6). This
          doesn't have any impact other than the learned compass
          offsets
         */
        calibration[0] = calibration[0] * gain_multiple / good_count;
        calibration[1] = calibration[1] * gain_multiple / good_count;
        calibration[2] = calibration[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        calibration[0] = 1.0;
        calibration[1] = 1.0;
        calibration[2] = 1.0;
    }

    // leave test mode
    if (!re_initialise()) {
        _bus_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }

    _bus_sem->give();
    hal.scheduler->resume_timer_procs();
    _initialised = true;

	// perform an initial read
	read();

#if 0
    hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), 
                          calibration[0], calibration[1], calibration[2]);
#endif

    if (success) {
        // register the compass instance in the frontend
        _compass_instance = register_compass();
        set_dev_id(_compass_instance, _product_id);
    }

    return success;
}

// Read Sensor data
void AP_Compass_HMC5843::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
    if (_retry_time != 0) {
        if (hal.scheduler->millis() < _retry_time) {
            return;
        }
        if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
			hal.i2c->setHighSpeed(false);
            return;
        }
    }

	if (_accum_count == 0) {
	   accumulate();
       if (_retry_time != 0) {
		  hal.i2c->setHighSpeed(false);
		  return;
	   }
	}

    Vector3f field(_mag_x_accum * calibration[0],
                   _mag_y_accum * calibration[1],
                   _mag_z_accum * calibration[2]);
    field /= _accum_count;

	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    // rotate to the desired orientation
    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        field.rotate(ROTATION_YAW_90);
    }

    publish_field(field, _compass_instance);
    _retry_time = 0;
}

/* MPU9250 implementation of the HMC5843 */
AP_HMC5843_SerialBus_MPU9250::AP_HMC5843_SerialBus_MPU9250(AP_HAL::I2CDriver *i2c, uint8_t addr) :
    _i2c(i2c),
    _addr(addr)
{
}

uint8_t AP_HMC5843_SerialBus_MPU9250::register_write(uint8_t address, uint8_t value)
{
    return _i2c->writeRegister(_addr, address, value);
}

uint8_t AP_HMC5843_SerialBus_MPU9250::register_read(uint8_t address, uint8_t *value, uint8_t count)
{
    return _i2c->readRegisters(_addr, address, count, value);
}

bool AP_HMC5843_SerialBus_MPU9250::configure()
{
    uint8_t user_ctrl;
    if (_i2c->readRegisters(HAL_INS_MPU60XX_I2C_ADDR, MPUREG_USER_CTRL, 1, &user_ctrl))
        return false;
    if (_i2c->writeRegister(HAL_INS_MPU60XX_I2C_ADDR, MPUREG_USER_CTRL, user_ctrl & ~BIT_USER_CTRL_I2C_MST_EN))
        return false;
    
    uint8_t ra_int_pin_cfg;
    if (_i2c->readRegisters(HAL_INS_MPU60XX_I2C_ADDR, MPUREG_INT_PIN_CFG, 1, &ra_int_pin_cfg))
        return false;
    if (_i2c->writeRegister(HAL_INS_MPU60XX_I2C_ADDR, MPUREG_INT_PIN_CFG, ra_int_pin_cfg | BIT_I2c_BYPASS_EN))
        return false;
    
    uint8_t ra_pwr_mgmt_1;
    if (_i2c->readRegisters(HAL_INS_MPU60XX_I2C_ADDR, MPUREG_PWR_MGMT_1, 1, &ra_pwr_mgmt_1))
        return false;
    if (_i2c->writeRegister(HAL_INS_MPU60XX_I2C_ADDR, MPUREG_PWR_MGMT_1, ra_pwr_mgmt_1 & ~BIT_PWR_MGMT_1_SLEEP))
        return false;

    return true;
}

uint8_t AP_HMC5843_SerialBus_MPU9250::read_raw(struct raw_value *rv)
{
    return _i2c->readRegisters(_addr, HMC5843_HXL, sizeof(*rv), (uint8_t *) rv);
}

AP_HAL::Semaphore * AP_HMC5843_SerialBus_MPU9250::get_semaphore()
{
    return _i2c->get_semaphore();
}

bool AP_HMC5843_SerialBus_MPU9250::start_measurements()
{
//    const uint8_t count = sizeof(struct raw_value);

//    /* Don't sample HMC5843 at MPU9250's sample rate. See MPU9250's datasheet
//     * about registers below and registers 73-96, External Sensor Data */
//    _write(MPUREG_I2C_SLV4_CTRL, 31);
//    _write(MPUREG_I2C_MST_DELAY_CTRL, I2C_SLV0_DLY_EN);
//
//    /* Configure the registers from HMC5843 that will be read by MPU9250's
//     * master: we will get the result directly from MPU9250's registers starting
//     * from MPUREG_EXT_SENS_DATA_00 when read_raw() is called */
//    _write(MPUREG_I2C_SLV0_ADDR, HMC5843_I2C_ADDR | READ_FLAG);
//    _write(MPUREG_I2C_SLV0_REG, HMC5843_HXL);
//    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count);

    return true;
}

uint32_t AP_HMC5843_SerialBus_MPU9250::get_dev_id()
{
    //return AP_COMPASS_TYPE_HMC5843_MPU9250;
    return 0;
}

/* I2C implementation of the HMC5843 */
AP_HMC5843_SerialBus_I2C::AP_HMC5843_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr) :
    _i2c(i2c),
    _addr(addr)
{
}

uint8_t AP_HMC5843_SerialBus_I2C::register_write(uint8_t address, uint8_t value)
{
    return _i2c->writeRegister(_addr, address, value);
}

uint8_t AP_HMC5843_SerialBus_I2C::register_read(uint8_t address, uint8_t *value, uint8_t count)
{
    return _i2c->readRegisters(_addr, address, count, value);
}

uint8_t AP_HMC5843_SerialBus_I2C::read_raw(struct raw_value *rv)
{
    return _i2c->readRegisters(_addr, HMC5843_HXL, sizeof(*rv), (uint8_t *) rv);
}

AP_HAL::Semaphore * AP_HMC5843_SerialBus_I2C::get_semaphore()
{
    return _i2c->get_semaphore();
}

uint32_t AP_HMC5843_SerialBus_I2C::get_dev_id()
{
    //return AP_COMPASS_TYPE_HMC5843_I2C;
    return 0;
}