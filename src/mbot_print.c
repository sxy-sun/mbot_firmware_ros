#include <stdio.h>
#include <string.h>
#include "mbot_print.h"

void generateTableInt(char* buf, int rows, int cols, const char* title, const char* headings[], int data[rows][cols]) {
    char line[256] = {0};
    int title_len = strlen(title);
    int line_len = cols * 12;
    memset(line, '-', line_len-1);
    line[line_len] = '\0';
    sprintf(buf, "|%s|\n", line);
    sprintf(buf + strlen(buf), "|\033[33m %s%*s\033[0m|\n", title, line_len - title_len - 2, "");
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|\033[34m  %s%*s\033[0m", headings[i], 9 - (int)strlen(headings[i]), "");
    }
    sprintf(buf + strlen(buf), "|\n");
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|%s", "-----------");
    }
    sprintf(buf + strlen(buf), "|\n");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sprintf(buf + strlen(buf), "| %9d ", data[i][j]);
        }
        sprintf(buf + strlen(buf), "|\n");
    }
}

void generateTableFloat(char* buf, int rows, int cols, const char* title, const char* headings[], float data[rows][cols]) {
    char line[256] = {0};
    int title_len = strlen(title);
    int line_len = cols * 12;
    memset(line, '-', line_len-1);
    line[line_len] = '\0';
    sprintf(buf, "|%s|\n", line);
    sprintf(buf + strlen(buf), "|\033[33m %s%*s\033[0m|\n", title, line_len - title_len - 2, "");
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|\033[34m  %s%*s\033[0m", headings[i], 9 - (int)strlen(headings[i]), "");
    }
    sprintf(buf + strlen(buf), "|\n");
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|%s", "-----------");
    }
    sprintf(buf + strlen(buf), "|\n");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sprintf(buf + strlen(buf), "|  %8.4f ", data[i][j]);
        }
        sprintf(buf + strlen(buf), "|\n");
    }
}

void generateBottomLine(char* buf, int cols) {
    char line[256] = {0};
    int line_len = cols * 12;
    memset(line, '-', line_len-1);
    line[line_len] = '\0';
    sprintf(buf, "|%s|\n", line);
}

void mbot_print_state(const mbot_state_t* state) {
    printf("\033[2J\r");
    printf("| \033[32m MBot State \033[0m TIME: %lld |\n", state->timestamp_us);

    const char* analog_headings[] = {"AIN 0","AIN 1","AIN 2","BATT (V)"};
    const char* enc_headings[] = {"ENC L", "ENC R"};
    const char* imu_headings[] = {"ROLL", "PITCH", "YAW"};
    const char* motor_vel_headings[] = {"MOT L", "MOT R"};
    const char* odom_headings[] = {"X", "Y", "THETA"};
    char buf[1024] = {0};

    // Analog
    float adc_array[1][4] = {{state->analog_in[0], state->analog_in[1], state->analog_in[2], state->analog_in[3]}};
    generateTableFloat(buf, 1, 4, "ANALOG", analog_headings, adc_array);
    printf("\r%s", buf);
    buf[0] = '\0';

    // Encoders
    int encs[1][2] = {{state->encoder_ticks[0], state->encoder_ticks[1]}};
    generateTableInt(buf, 1, 2, "ENCODERS", enc_headings, encs);
    printf("\r%s", buf);
    buf[0] = '\0';

    // IMU (RPY)
    float imu_array[1][3] = {{state->imu_rpy[0], state->imu_rpy[1], state->imu_rpy[2]}};
    generateTableFloat(buf, 1, 3, "IMU", imu_headings, imu_array);
    printf("\r%s", buf);
    buf[0] = '\0';

    // Motor velocities
    float motor_array[1][2] = {{state->wheel_vel[0], state->wheel_vel[1]}};
    generateTableFloat(buf, 1, 2, "MOTOR", motor_vel_headings, motor_array);
    printf("\r%s", buf);
    buf[0] = '\0';

    // Odometry
    float odom_array[1][3] = {{state->odom_x, state->odom_y, state->odom_theta}};
    generateTableFloat(buf, 1, 3, "ODOMETRY", odom_headings, odom_array);
    printf("\r%s", buf);
    buf[0] = '\0';

    generateBottomLine(buf, 3);
    printf("\r%s\n", buf);
} 