#include <stdio.h>
#include "pico/stdlib.h"
#include "tusb.h"

int main() {
    // Set system clock
    set_sys_clock_khz(125000, true);
    
    // Initialize USB
    stdio_init_all();  // This handles TinyUSB initialization
    
    // Simple message
    printf("USB CDC test program starting\r\n");
    
    // Main loop
    uint32_t counter = 0;
    while (1) {
        // Handle USB tasks
        tud_task();
        
        // Print a message every second to both stdio (CDC0) and directly to CDC1
        if (counter % 100 == 0) {
            // Print to stdio (CDC0)
            printf("CDC0 Counter: %lu\r\n", counter);
            
            // Print directly to CDC1 if connected
            if (tud_cdc_n_connected(1)) {
                char buffer[32];
                sprintf(buffer, "CDC1 Counter: %lu\r\n", counter);
                tud_cdc_n_write(1, buffer, strlen(buffer));
                tud_cdc_n_write_flush(1);
            }
        }
        
        // Increment counter
        counter++;
        sleep_ms(10);
    }
    
    return 0;
} 