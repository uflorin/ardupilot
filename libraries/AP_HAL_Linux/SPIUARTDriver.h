#ifndef __AP_HAL_LINUX_SPIUARTDRIVER_H__
#define __AP_HAL_LINUX_SPIUARTDRIVER_H__

#include <AP_HAL_Linux.h>

#include "UARTDriver.h"

#define RTKLIB_PROXY 1
#ifdef RTKLIB_PROXY
#include <sys/socket.h>
#include <arpa/inet.h>

#endif

class Linux::LinuxSPIUARTDriver : public Linux::LinuxUARTDriver {
public:
    LinuxSPIUARTDriver();
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void _timer_tick(void);

protected:
    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);

private:
    bool sem_take_nonblocking();
    void sem_give();

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    uint32_t _last_update_timestamp;

    uint8_t *_buffer;
    bool _external;

#ifdef RTKLIB_PROXY
    int _sockfd;
    struct sockaddr_in servaddr;
#endif
};

#endif //__AP_HAL_LINUX_SPIUARTDRIVER_H__
