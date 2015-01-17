#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO_ODROID_C1

#include "GPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
LinuxGPIO_ODROID_C1::LinuxGPIO_ODROID_C1()
{}

void LinuxGPIO_ODROID_C1::init()
{
    if ((mem_fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
        hal.scheduler->panic("Can't open /dev/mem");

    gpio = (volatile uint32_t *) mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, ODROID_GPIO_BASE);

    close(mem_fd);

    if ((int32_t)gpio == -1)
        hal.scheduler->panic("GPIO mmap failed");
}

void LinuxGPIO_ODROID_C1::pinMode(uint8_t pin, uint8_t output)
{
    if (pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END) 
        if (output == HAL_GPIO_INPUT)
            *(gpio + GPIOX_FSEL_REG_OFFSET) = (*(gpio + GPIOX_FSEL_REG_OFFSET) |  (1 << (pin - GPIOX_PIN_START)));
        else
            *(gpio + GPIOX_FSEL_REG_OFFSET) = (*(gpio + GPIOX_FSEL_REG_OFFSET) & ~(1 << (pin - GPIOX_PIN_START)));
    
    if (pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
        if (output == HAL_GPIO_INPUT)
            *(gpio + GPIOY_FSEL_REG_OFFSET) = (*(gpio + GPIOY_FSEL_REG_OFFSET) |  (1 << (pin - GPIOY_PIN_START)));
        else
            *(gpio + GPIOY_FSEL_REG_OFFSET) = (*(gpio + GPIOY_FSEL_REG_OFFSET) & ~(1 << (pin - GPIOY_PIN_START)));
}

int8_t LinuxGPIO_ODROID_C1::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}

uint8_t LinuxGPIO_ODROID_C1::read(uint8_t pin)
{
    if (pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
        if ((*(gpio + GPIOX_INP_REG_OFFSET) & (1 << (pin - GPIOX_PIN_START))) != 0)
            return HIGH;
        else
            return LOW;

    if (pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
        if ((*(gpio + GPIOY_INP_REG_OFFSET) & (1 << (pin - GPIOY_PIN_START))) != 0)
            return HIGH;
        else
            return LOW;

    return LOW;    
}

void LinuxGPIO_ODROID_C1::write(uint8_t pin, uint8_t value)
{
    if (pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
        if (value == LOW)
            *(gpio + GPIOX_OUTP_REG_OFFSET) &= ~(1 << (pin - GPIOX_PIN_START));
        else
            *(gpio + GPIOX_OUTP_REG_OFFSET) |=  (1 << (pin - GPIOX_PIN_START));

    if (pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
        if (value == LOW)
            *(gpio + GPIOY_OUTP_REG_OFFSET) &= ~(1 << (pin - GPIOY_PIN_START));
        else
            *(gpio + GPIOY_OUTP_REG_OFFSET) |=  (1 << (pin - GPIOY_PIN_START));
}

void LinuxGPIO_ODROID_C1::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO_ODROID_C1::channel(uint16_t n) {
    return new LinuxDigitalSource(n);
}

/* Interrupt interface: */
bool LinuxGPIO_ODROID_C1::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

bool LinuxGPIO_ODROID_C1::usb_connected(void)
{
    return false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
