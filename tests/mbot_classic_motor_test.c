#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <mbot/motor/motor.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"
#include "hardware/clocks.h"

void drive_motor_up_down(int motor);
void blink();

int mbot_init_pico(void){
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
     if(!set_sys_clock_khz(SYS_CLOCK, true)){
         printf("ERROR mbot_init_pico: cannot set system clock\n");
         return MBOT_ERROR;
     };

    stdio_init_all(); // enable USB serial terminal
    sleep_ms(500);
    printf("\nMBot Booting Up!\n");
    return MBOT_OK;
}

int main() {
    sleep_ms(500);
    mbot_init_pico();
    sleep_ms(2000);
    printf("\033[2J\r");
    printf("***MBot Classic Motor Test***\n");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    int freq = 5000;

    mbot_motor_init_freq(MOT_R, freq);
    mbot_motor_init_freq(MOT_L, freq);

    blink();
    printf("Testing right motor...\n");
    drive_motor_up_down(MOT_R);

    blink();
    printf("Testing left motor...\n");
    drive_motor_up_down(MOT_L);

    blink();
    printf("Done!\n");

    mbot_motor_cleanup(MOT_R);
    mbot_motor_cleanup(MOT_L);

    blink();
    return 0;
}

void drive_motor_up_down(int motor) {
    float d = 0;
    printf("Driving Forward...\n");
    for (; d < 1.0; d += 0.01) {
        mbot_motor_set_duty(motor, d);
        sleep_ms(25);
    }
    for (; d > 0.0; d -= 0.01) {
        mbot_motor_set_duty(motor, d);
        sleep_ms(25);
    }
    printf("Driving Backward...\n");
    for (; d > -1.0; d -= 0.01) {
        mbot_motor_set_duty(motor, d);
        sleep_ms(25);
    }
    for (; d < 0.0; d += 0.01) {
        mbot_motor_set_duty(motor, d);
        sleep_ms(25);
    }
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
