#ifndef _DUAL_CDC_H_
#define _DUAL_CDC_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize both CDC interfaces
 * 
 * @return true if initialization was successful, false otherwise
 */
bool dual_cdc_init(void);

/**
 * @brief Check if both CDC interfaces are connected
 * 
 * @return true if both CDC interfaces are connected, false otherwise
 */
bool dual_cdc_connected(void);

/**
 * @brief Write data to a specific CDC interface
 * 
 * @param itf The interface to write to (0 or 1)
 * @param buf The buffer containing the data to write
 * @param length The number of bytes to write
 */
void dual_cdc_write_chars(int itf, const char *buf, int length);

/**
 * @brief Read data from a specific CDC interface
 * 
 * @param itf The interface to read from (0 or 1)
 * @param buf The buffer to store the data
 * @param length The maximum number of bytes to read
 * @return The number of bytes read, or a negative error code
 */
int dual_cdc_read_chars(int itf, char *buf, int length);

/**
 * @brief Check how many bytes are available to read from a specific CDC interface
 * 
 * @param itf The interface to check (0 or 1)
 * @return The number of bytes available to read
 */
int dual_cdc_available(int itf);

/**
 * @brief Process USB tasks to handle CDC interfaces
 * 
 * This function should be called periodically in the main loop
 */
void dual_cdc_task(void);

#endif // _DUAL_CDC_H_ 