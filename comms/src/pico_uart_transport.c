#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "comms/dual_cdc.h"
#include <uxr/client/profile/transport/custom/custom_transport.h>

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

// Open transport: Simple open, dual_cdc_init() already called in main
bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    // printf("microROS transport open requested. CDC1 connected: %s\r\n", 
    //        dual_cdc_connected() ? "YES" : "NO");
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    // printf("microROS transport close requested\r\n");
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode)
{
    // Write data to CDC1 using our thread-safe function
    dual_cdc_write_chars(1, (const char*)buf, len);
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();
    
    // Wait for data to be available
    while (dual_cdc_available(1) < len) {
        // Check for timeout
        int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
        if (elapsed_time_us < 0) {
            *errcode = 1;
            // printf("MicroROS read timeout\r\n");
            return 0;
        }
        
        // Short sleep to prevent tight loop
        sleep_us(1000);
    }
    
    // Read using our thread-safe function
    int rc = dual_cdc_read_chars(1, (char*)buf, len);
    return (rc > 0) ? (size_t)rc : 0;
}
