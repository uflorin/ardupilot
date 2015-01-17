
#ifndef __AP_HAL_LINUX_GPIO_ODROID_C1_H__
#define __AP_HAL_LINUX_GPIO_ODROID_C1_H__

#include <stdint.h>
#include <AP_HAL_Linux.h>

#define LOW                 0
#define HIGH                1

// ODROID-C1 GPIO memory

#define ODROIDC_GPIO_MASK (0xFFFFFF80)
#define ODROIDC_PERI_BASE 0xC1100000
#define GPIO_REG_OFFSET   0x8000
#define ODROID_GPIO_BASE  (ODROIDC_PERI_BASE + GPIO_REG_OFFSET)
#define BLOCK_SIZE        (4*1024)

#define GPIOX_PIN_START         97
#define GPIOX_PIN_END           118
#define GPIOX_FSEL_REG_OFFSET   0x0C
#define GPIOX_OUTP_REG_OFFSET   0x0D
#define GPIOX_INP_REG_OFFSET    0x0E
#define GPIOX_PUPD_REG_OFFSET   0x3E
#define GPIOX_PUEN_REG_OFFSET   0x4C

#define GPIOY_PIN_START         80
#define GPIOY_PIN_END           96
#define GPIOY_FSEL_REG_OFFSET   0x0F
#define GPIOY_OUTP_REG_OFFSET   0x10
#define GPIOY_INP_REG_OFFSET    0x11
#define GPIOY_PUPD_REG_OFFSET   0x3D
#define GPIOY_PUEN_REG_OFFSET   0x4B

// ODROID-C1 GPIO mapping
#define ODROID_C1_GPIO_83       83    // Pin 7             NAVIO_PPM_INPUT
#define ODROID_C1_GPIO_118      118   // Pin 26    CE1     NAVIO_MPU9250_CS
#define ODROID_C1_GPIO_117      117   // Pin 24    CE0     NAVIO_UBLOX_CS
#define ODROID_C1_GPIO_106      106   // Pin 21    MISO
#define ODROID_C1_GPIO_104      104   // Pin 19    MOSI
#define ODROID_C1_GPIO_105      105   // Pin 23    SCLK
#define ODROID_C1_GPIO_88       88    // Pin 11            NAVIO_UART_PORT_5
#define ODROID_C1_GPIO_87       87    // Pin 12            NAVIO_UART_PORT_4
#define ODROID_C1_GPIO_115      115   // Pin 15            NAVIO_UBLOX_PPS
#define ODROID_C1_GPIO_104      104   // Pin 16            NAVIO_MPU9250_DRDY
#define ODROID_C1_GPIO_102      102   // Pin 18            NAVIO_SPI_PORT_6
#define ODROID_C1_GPIO_103      103   // Pin 22            NAVIO_SPI_PORT_5
#define ODROID_C1_GPIO_116      116   // Pin 13            NAVIO_PCA9685_OE

class Linux::LinuxGPIO_ODROID_C1 : public AP_HAL::GPIO {
private:
    int  mem_fd;
    volatile uint32_t *gpio;

public:
    LinuxGPIO_ODROID_C1();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};

#endif // __AP_HAL_LINUX_GPIO_ODROID_C1_H__
