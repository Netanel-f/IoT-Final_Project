/******************************************************************************
 * @serial_io_usart.h
 * @brief Interface for reading and parsing data from the usart connection.
 * @version 0.0.1
 *  **************************************************************************/

#ifndef SRC_SERIAL_IO_USART_H_
#define SRC_SERIAL_IO_USART_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


/******************************************************************************
 * 								DEFS
*****************************************************************************/
#define SERIAL_TIMEOUT -1
//#define RX_BUFFER_SIZE_CELL 1000             // Software receive buffer size
#define RX_BUFFER_SIZE_CELL 1600             // Software receive buffer size
static char rxBufferCellular[RX_BUFFER_SIZE_CELL]; // Software receive buffer
extern volatile uint32_t msTicks;
extern bool DEBUG;

/**************************************************************************//**
 * @brief Initiates the serial connection.
 * @param port - input port.
 * @param baud - baud rate
 * @return true if successful.
 *****************************************************************************/
bool SerialInitCellular(char* port, unsigned int baud);


/**************************************************************************//**
 * @brief Receive data from serial connection.
 * @param buf - buffer to be filled.
 * @param maxlen - maximum length of a line of data.
 * @param timeout_ms - length of the timeout in ms.
 *****************************************************************************/
unsigned int SerialRecvCellular(unsigned char* buf, unsigned int maxlen, unsigned int timeout_ms);

/**
 * writing buf string to serial port
 * @param buf
 * @param size
 * @return true on success
 */
bool SerialSendCellular(unsigned char *buf, unsigned int size);


/**************************************************************************//**
 * @brief Empties the input buffer.
 *****************************************************************************/
void SerialFlushInputBuffCellular(void);


/**************************************************************************//**
 * @brief Disable the serial connection.
 *****************************************************************************/
void SerialDisableCellular();


/***************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void DelayCellular(uint32_t ms);


#endif /* SRC_SERIAL_IO_USART_H_ */
