#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>

#include "RCInput.h"
#include "sbus.h"
#include "dsm.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

LinuxRCInput::LinuxRCInput() :
    new_rc_input(false)
{
    ppm_state._channel_counter = -1;
}

void LinuxRCInput::init(void* machtnichts)
{
}

bool LinuxRCInput::new_input() 
{
    return new_rc_input;
}

uint8_t LinuxRCInput::num_channels() 
{
    return _num_channels;
}

uint16_t LinuxRCInput::read(uint8_t ch) 
{
    new_rc_input = false;
    if (_override[ch]) {
        return _override[ch];
    }
    if (ch >= _num_channels) {
        return 0;
    }
    return _pwm_values[ch];
}

uint8_t LinuxRCInput::read(uint16_t* periods, uint8_t len) 
{
    uint8_t i;
    for (i=0; i<len; i++) {
        if((periods[i] = read(i))){
            continue;
        }
        else{
            break;
        }
    }
    return (i+1);
}

bool LinuxRCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
    bool res = false;
    if(len > LINUX_RC_INPUT_NUM_CHANNELS){
        len = LINUX_RC_INPUT_NUM_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool LinuxRCInput::set_override(uint8_t channel, int16_t override) 
{
    if (override < 0) return false; /* -1: no change. */
    if (channel < LINUX_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            new_rc_input = true;
            return true;
        }
    }
    return false;
}

void LinuxRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
       _override[i] = 0;
    }
}


/*
  process a PPM-sum pulse of the given width
 */
void LinuxRCInput::_process_ppmsum_pulse(uint16_t width_usec)
{
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (ppm_state._channel_counter >= 5) {
            for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
                _pwm_values[i] = ppm_state._pulse_capt[i];
            }
            _num_channels = ppm_state._channel_counter;
            new_rc_input = true;
        }
        ppm_state._channel_counter = 0;
        return;
    }
    if (ppm_state._channel_counter == -1) {
        // we are not synchronised
        return;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

        // move to next channel
        ppm_state._channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (ppm_state._channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
        for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
            _pwm_values[i] = ppm_state._pulse_capt[i];
        }
        _num_channels = ppm_state._channel_counter;
        new_rc_input = true;
        ppm_state._channel_counter = -1;
    }
}

/*
  process a SBUS input pulse of the given width
 */
void LinuxRCInput::_process_sbus_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    uint16_t bits_s0 = (width_s0+1) / 10;
    uint16_t bits_s1 = (width_s1+1) / 10;
    uint16_t nlow;

    uint8_t byte_ofs = sbus_state.bit_ofs/12;
    uint8_t bit_ofs = sbus_state.bit_ofs%12;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }
	
    if (bits_s0+bit_ofs > 10) {
        // invalid data as last two bits must be stop bits
        goto reset;
    }

    // pull in the high bits
    sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
    sbus_state.bit_ofs += bits_s0;
    bit_ofs += bits_s0;

    // pull in the low bits
    nlow = bits_s1;
    if (nlow + bit_ofs > 12) {
        nlow = 12 - bit_ofs;
    }
    bits_s1 -= nlow;
    sbus_state.bit_ofs += nlow;

    if (sbus_state.bit_ofs == 32*12 && bits_s1 > 12) {
        // we have a full frame
        uint8_t bytes[25];
        uint8_t i;
        for (i=0; i<25; i++) {
            // get inverted data
            uint16_t v = ~sbus_state.bytes[i];
            // check start bit
            if ((v & 1) != 0) {
                goto reset;
            }
            // check stop bits
            if ((v & 0xC00) != 0xC00) {
                goto reset;
            }
            // check parity
            uint8_t parity = 0, j;
            for (j=1; j<=8; j++) {
                parity ^= (v & (1U<<j))?1:0;
            }
            if (parity != (v&0x200)>>9) {
                goto reset;
            }
            bytes[i] = ((v>>1) & 0xFF);
        }
        uint16_t values[LINUX_RC_INPUT_NUM_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe=false, sbus_frame_drop=false;
        if (sbus_decode(bytes, values, &num_values, 
                        &sbus_failsafe, &sbus_frame_drop, 
                        LINUX_RC_INPUT_NUM_CHANNELS) && 
            num_values >= 5) {
            for (i=0; i<num_values; i++) {
                _pwm_values[i] = values[i];
            }
            _num_channels = num_values;
            new_rc_input = true;
        }
        goto reset;
    } else if (bits_s1 > 12) {
        // break
        goto reset;
    }
    return;
reset:
    memset(&sbus_state, 0, sizeof(sbus_state));        
}

/*
  process a iBUS input pulse of the given width
 */
void LinuxRCInput::_process_ibus_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    uint16_t bits_s0 = (width_s0+2) / 8.0f;
    uint16_t bits_s1 = (width_s1+2) / 8.0f;
    uint16_t nlow;
    uint8_t byte_ofs;
    uint8_t bit_ofs;
    
    //hal.console->printf_P(PSTR("bits_s0 %d, bits_s1 %d\n"), bits_s0, bits_s1);
    
    if (ibus_state.frame_size == 0)
    {
        ibus_state.frame_size = 1;
    }

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        hal.console->printf_P(PSTR("invalid data\n"));
        goto reset;
    }
    
//    if (bits_s0 > 50) {
//        memset(&ibus_state, 0, sizeof(ibus_state));
//    }
    
    byte_ofs = ibus_state.bit_ofs/10;
    bit_ofs = ibus_state.bit_ofs%10;
	
    if (bits_s0+bit_ofs >= 10) {
        //hal.console->printf_P(PSTR("*******************************************************************************************\n"));
        
        //if (bit_ofs > 1 && bit_ofs < 10)
        if (bit_ofs > 1)
        {
            bits_s0 = 10 - bit_ofs;
        }
        else
        {
            //hal.console->printf_P(PSTR("ibus_state.bit_ofs 10 != 9, %d\n"), ibus_state.bit_ofs);
            bits_s0 = 0;
            byte_ofs = 0;
            bit_ofs = 0;
            memset(&ibus_state, 0, sizeof(ibus_state));
        }
    }
	
//    if (bits_s0+bit_ofs > 10) {
//        // invalid data as last two bits must be stop bits
//        hal.console->printf_P(PSTR("invalid data as last two bits must be stop bits\n"));
//        goto reset;
//    }

    // pull in the high bits
    ibus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
    ibus_state.bit_ofs += bits_s0;
    bit_ofs += bits_s0;

    // pull in the low bits
    nlow = bits_s1;
    if (nlow + bit_ofs > 10) {
        nlow = 10 - bit_ofs;
    }
    bits_s1 -= nlow;
    ibus_state.bit_ofs += nlow;
    
    //hal.console->printf_P(PSTR("ibus_state.bit_ofs: %d; bits_s1: %d\n"), ibus_state.bit_ofs, bits_s1);

//    if (ibus_state.bit_ofs == 32*10) {
      if (ibus_state.bit_ofs == ibus_state.frame_size*10) {
          //hal.console->printf_P(PSTR("                   ibus_state.bit_ofs == frame_size*10\n"));
          
        // we have a full frame
        uint8_t bytes[ibus_state.frame_size];
        memset(bytes, 0, ibus_state.frame_size);
        
        uint8_t i;
        for (i=0; i<ibus_state.frame_size; i++) {
            // get inverted data
            uint16_t v = ibus_state.bytes[i];
            // check start bit
            if ((v & 1) != 0) {
                //hal.console->printf_P(PSTR("check start bit %d\n"), i);
                goto reset;
            }
            // check stop bits
            if ((v & 0x200) != 0x200) {
//                hal.console->printf_P(PSTR("check stop bits %d\n"), i);
//                
//                for (int j=0; j<=i; j++) {
//                    hal.console->printf_P(PSTR("%04x "), ibus_state.bytes[j]);
//                }
//                
//                hal.console->printf_P(PSTR("\n"));
                
                goto reset;
            }
            bytes[i] = ((v>>1) & 0xFF);
        }
//        uint16_t values[LINUX_RC_INPUT_NUM_CHANNELS];
//        uint16_t num_values=0;
//        bool ibus_failsafe=false, ibus_frame_drop=false;
        
        if (ibus_state.frame_size == 1 && bytes[0] == 0x20)
        {
            //hal.console->printf_P(PSTR("0x20\n"));
            
            ibus_state.bit_ofs += bits_s1;
            ibus_state.frame_size = 2;
        } else if (ibus_state.frame_size == 2 && bytes[0] == 0x20 && bytes[1] == 0x40)
        {
            //hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
            //hal.console->printf_P(PSTR("0x20 0x40\n"));
            ibus_state.bit_ofs += bits_s1;
            ibus_state.frame_size = 32;
            //ibus_state.frame_size = 1;
        }
        else
        {
            //goto reset;
            if (ibus_state.frame_size > 2 && bytes[0] == 0x20 && bytes[1] == 0x40
                    //&& bytes[15] == 0xf3
                    )
            {
//                hal.console->printf_P(PSTR("ALL GOOD\n"));
//                
//                for (i=0; i<ibus_state.frame_size; i++) {
//                    hal.console->printf_P(PSTR("%02x "), bytes[i]);
//                }
//                
//                hal.console->printf_P(PSTR("\nEND GOOD\n"));
                
                uint8_t chksum = 159;
                for(i=2;i<30;i++) chksum -= bytes[i];
                
                if (chksum == bytes[30])
                {
                    //hal.console->printf_P(PSTR("\nCHKSUM\n"));
                    
                    _num_channels = 8;
                    
                    _pwm_values[0] = (bytes[3]<<8) + bytes[2];
                    _pwm_values[1] = (bytes[5]<<8) + bytes[4];
                    _pwm_values[2] = (bytes[7]<<8) + bytes[6];
                    _pwm_values[3] = (bytes[9]<<8) + bytes[8];
                    _pwm_values[4] = (bytes[11]<<8) + bytes[10];
                    _pwm_values[5] = (bytes[13]<<8) + bytes[12];
                    _pwm_values[6] = (bytes[15]<<8) + bytes[14];
                    _pwm_values[7] = (bytes[17]<<8) + bytes[16];
                    
                    bool isInvalid = false;
                    
                    for(i=0;i<_num_channels;i++)
                    {
                        if (_pwm_values[i] > 2000)
                        {
                            isInvalid = true;
                            break;
                        }
                    }
                    
                    if (isInvalid)
                    {
                        for(i=0;i<_num_channels;i++)
                        {
                            _pwm_values[i] = 0;
                        }
                        
                        new_rc_input = false;
                    }
                    else
                    {
                        new_rc_input = true;
                        
                        //hal.console->printf_P(PSTR("_pwm_values[0] %d\n"), _pwm_values[0]);
                    }
                }
                
                //hal.console->printf_P(PSTR("chksum %02x "), chksum);
            }
            
            //hal.console->printf_P(PSTR("bytes[0]: %02x\n"), bytes[0]);
            
            memset(&ibus_state, 0, sizeof(ibus_state));
            ibus_state.bit_ofs = bits_s1;
        }
//        num_values = 5;
//        if (num_values > 3
////                ibus_decode(bytes, values, &num_values, 
////                        &ibus_failsafe, &ibus_frame_drop, 
////                        LINUX_RC_INPUT_NUM_CHANNELS) && 
////            num_values >= 5
//                ) {
//            for (i=0; i<num_values; i++) {
//                _pwm_values[i] = values[i];
//            }
//            _num_channels = num_values;
//            new_rc_input = true;
//        }
        //goto reset;
    } else
        if (bits_s1 > 10) {
        // break
        hal.console->printf_P(PSTR("bits_s1 > 10\n"));
        goto reset;
    }
        else
        {
            ibus_state.bit_ofs += bits_s1;
        }
    
    
    return;
reset:
    memset(&ibus_state, 0, sizeof(ibus_state));
}

void LinuxRCInput::_process_dsm_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 115200 bps
    uint16_t bits_s0 = ((width_s0+4)*(uint32_t)115200) / 1000000;
    uint16_t bits_s1 = ((width_s1+4)*(uint32_t)115200) / 1000000;
    uint8_t bit_ofs, byte_ofs;
    uint16_t nbits;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;
    
    if(byte_ofs > 15) {
        // invalid data
        goto reset;
    }

    // pull in the high bits
    nbits = bits_s0;
    if (nbits+bit_ofs > 10) {
        nbits = 10 - bit_ofs;
    }
    dsm_state.bytes[byte_ofs] |= ((1U<<nbits)-1) << bit_ofs;
    dsm_state.bit_ofs += nbits;
    bit_ofs += nbits;

    if (bits_s0 - nbits > 10) {
        if (dsm_state.bit_ofs == 16*10) {
            // we have a full frame
            uint8_t bytes[16];
            uint8_t i;
            for (i=0; i<16; i++) {
                // get raw data
                uint16_t v = dsm_state.bytes[i];
                
                // check start bit
                if ((v & 1) != 0) {
                    goto reset;
                }
                // check stop bits
                if ((v & 0x200) != 0x200) {
                    goto reset;
                }
                bytes[i] = ((v>>1) & 0xFF);
            }
            uint16_t values[8];
            uint16_t num_values=0;
            if (dsm_decode(hal.scheduler->micros64(), bytes, values, &num_values, 8) && 
                num_values >= 5) {
                for (i=0; i<num_values; i++) {
                    _pwm_values[i] = values[i];
                }
                _num_channels = num_values;                
                new_rc_input = true;
            }
        }
        memset(&dsm_state, 0, sizeof(dsm_state));
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;

    if (bits_s1+bit_ofs > 10) {
        // invalid data
        goto reset;
    }

    // pull in the low bits
    dsm_state.bit_ofs += bits_s1;
    return;
reset:
    memset(&dsm_state, 0, sizeof(dsm_state));        
}

/*
  process a RC input pulse of the given width
 */
void LinuxRCInput::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
#if 0
    // useful for debugging
    static FILE *rclog;
    if (rclog == NULL) {
        rclog = fopen("/tmp/rcin.log", "w");
    }
    if (rclog) {
        fprintf(rclog, "%u %u\n", (unsigned)width_s0, (unsigned)width_s1);
    }
#endif
    
    //hal.console->printf_P(PSTR("width_s0: %02x; width_s1: %02x\n"), width_s0, width_s1);
    
    // treat as PPM-sum
    //_process_ppmsum_pulse(width_s0 + width_s1);

    // treat as SBUS
    //_process_sbus_pulse(width_s0, width_s1);

    // treat as iBUS
    _process_ibus_pulse(width_s0, width_s1);

    // treat as DSM
    //_process_dsm_pulse(width_s0, width_s1);
}

#endif // CONFIG_HAL_BOARD
