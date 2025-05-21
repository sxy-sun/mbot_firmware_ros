#ifndef MBOT_PRINT_H
#define MBOT_PRINT_H

#include "mbot_classic_ros.h" // for mbot_state_t

#ifdef __cplusplus
extern "C" {
#endif

void generateTableInt(char* buf, int rows, int cols, const char* title, const char* headings[], int data[rows][cols]);
void generateTableFloat(char* buf, int rows, int cols, const char* title, const char* headings[], float data[rows][cols]);
void generateBottomLine(char* buf, int cols);

void mbot_print_state(const mbot_state_t* state);

#ifdef __cplusplus
}
#endif

#endif // MBOT_PRINT_H