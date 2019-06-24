/******************************************************************************
 * @serial_io_uart.h
 * @brief Interface for reading and parsing data from the uart connection.
 * @version 0.0.1
 *  **************************************************************************/

#ifndef SRC_SERIAL_IO_UART_H_
#define SRC_SERIAL_IO_UART_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/******************************************************************************
 * 								DEFS
*****************************************************************************/
#define SERIAL_TIMEOUT -1
#define RX_BUFFER_SIZE_GPS 83             // Software receive buffer size

extern volatile uint32_t msTicks;
extern bool DEBUG;


/******************************************************************************
 * @brief Initiates the serial connection.
 * @param port - input port
 * @param baud - baud rate
 * @return true if successful.
 *****************************************************************************/
bool SerialInitGPS(char* port, unsigned int baud);


/******************************************************************************
 * @brief Receive data from serial connection.
 * @param buf - buffer to be filled.
 * @param maxlen - maximum length of a line of data.
 * @param timeout_ms - length of the timeout in ms.
 *****************************************************************************/
unsigned int SerialRecvGPS(unsigned char* buf, unsigned int maxlen, unsigned int timeout_ms);


/******************************************************************************
 * @brief Receive data from serial connection.
 * @param buf - buffer to be filled.
 * @param maxlen - maximum length of a line of data.
 * @param timeout_ms - length of the timeout in ms.
 *****************************************************************************/
void SerialFlushInputBuffGPS(void);


/******************************************************************************
 * @brief Disable the serial connection.
 *****************************************************************************/
void SerialDisableGPS();


/*******************************************************************************
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void DelayGPS(uint32_t ms);


#endif /* SRC_SERIAL_IO_UART_H_ */
