#include <stdio.h>
#include "pico/stdlib.h"
#include "comms/dual_cdc.h"
#include <string.h>

int main() {
    // Initialize stdio (will set up the default system clock)
    stdio_init_all();
    
    // Initialize dual CDC
    dual_cdc_init();

    // wait the cdc to connect
    mbot_wait_ms(2000);

    printf("Dual CDC test program starting\r\n");
    
    uint32_t counter = 0;
    while (1) {
        dual_cdc_task();

        if (counter % 100 == 0) {
            char buffer[64];
            // print to ttyACM0
            printf("CDC0 Counter: %lu\r\n", counter);
            
            // print to ttyACM1
            sprintf(buffer, "CDC1 Counter: %lu\r\n", counter);
            dual_cdc_write_chars(1, buffer, strlen(buffer));
        }
        
        counter++;
    }
    
    return 0;
} 