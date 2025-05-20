#include <stdio.h>
#include "pico/stdlib.h"
#include "comms/dual_cdc.h"
#include <string.h>

int main() {
    // Initialize stdio (will set up the default system clock)
    stdio_init_all();
    
    // Initialize dual CDC
    dual_cdc_init();
    
    // Wait for initialization
    sleep_ms(1000);
    
    printf("Dual CDC test program starting\r\n");
    
    uint32_t counter = 0;
    while (1) {
        // Process USB tasks
        dual_cdc_task();
        
        // Send message to both CDC interfaces every second
        if (counter % 100 == 0) {
            char buffer[64];
            sprintf(buffer, "CDC0 Counter: %lu\r\n", counter);
            dual_cdc_write_chars(0, buffer, strlen(buffer));
            
            sprintf(buffer, "CDC1 Counter: %lu\r\n", counter);
            dual_cdc_write_chars(1, buffer, strlen(buffer));
        }
        
        counter++;
        sleep_ms(10);
    }
    
    return 0;
} 