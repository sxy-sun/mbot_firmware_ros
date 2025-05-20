#include "comms/dual_cdc.h"
#include "tusb.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include <string.h>

// Mutex to protect CDC access
static mutex_t dual_cdc_mutex;

// Timeout for CDC operations
#define CDC_TIMEOUT_US 500000 // 500ms

bool dual_cdc_init(void) {
    // Initialize the mutex
    mutex_init(&dual_cdc_mutex);
    
    // TinyUSB may already be initialized by stdio_init_all()
    tusb_init();
    
    return true;
}

bool dual_cdc_connected(void) {
    return tud_cdc_n_connected(1);
}

void dual_cdc_write_chars(int itf, const char *buf, int length) {
    // Validate interface number
    if (itf < 0 || itf > 1) return;
    
    mutex_enter_blocking(&dual_cdc_mutex);
    
    if (tud_cdc_n_connected(itf)) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = (int)tud_cdc_n_write_available(itf);
            if (n > avail) n = avail;
            
            if (n) {
                int n2 = (int)tud_cdc_n_write(itf, buf + i, (uint32_t)n);
                tud_task();
                tud_cdc_n_write_flush(itf);
                i += n2;
            } else {
                tud_task();
                tud_cdc_n_write_flush(itf);
            }
        }
    }
    
    mutex_exit(&dual_cdc_mutex);
}

int dual_cdc_read_chars(int itf, char *buf, int length) {
    // Validate interface number
    if (itf < 0 || itf > 1) return -1;
    
    mutex_enter_blocking(&dual_cdc_mutex);
    
    int rc = -1;
    if (tud_cdc_n_connected(itf) && tud_cdc_n_available(itf)) {
        int count = (int)tud_cdc_n_read(itf, buf, (uint32_t)length);
        rc = count ? count : -1;
    }
    
    mutex_exit(&dual_cdc_mutex);
    return rc;
}

int dual_cdc_available(int itf) {
    // Validate interface number
    if (itf < 0 || itf > 1) return 0;
    
    return tud_cdc_n_available(itf);
}

void dual_cdc_task(void) {
    // Process TinyUSB tasks with mutex to prevent concurrent access
    if (mutex_try_enter(&dual_cdc_mutex, NULL)) {
        tud_task();
        mutex_exit(&dual_cdc_mutex);
    }
} 