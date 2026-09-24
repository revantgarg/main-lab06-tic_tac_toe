#ifndef UART_DRIVER_H
#define UART_DRIVER_H

/**
 ** UART functions
 **/
#define UART_PARTITY_NONE               0
#define UART_PARTITY_ODD                1
#define UART_PARTITY_EVEN               2
#define UART_PARTITY_MARK               3
#define UART_PARTITY_SPACE              4
/*
 * initialize UART based on the port number, baudrate and parity type
 *
 * returns 0 if no error and opened properly else error no indicating error type
 * */
int UART_Init(
int iPort,          /* UART port number as per chip */
int iBaudRate,      /* baudrate not all may be supported */
int iParity_NOEMS); /* parity enable 0- none, 1-odd, 2-even, 3-mark, 4-space */

/*
 * UART write - send chars through UART using buffer and count passed as parameters. If the FIFO is
 * full return back with number of bytes that was successfully added to FIFO (Non blocking)
 *
 * returns number of bytes actually transferred
 * */

int UART_Write(
int iPort,                      /* port identifier */
int iBytes,                     /* max number of bytes that should be written */
unsigned char *pcBytes);        /* buffer to be used to write into FIFO */

/*
 * UART Read a character from UART FIFO (non blocking). IF fifo does not have sufficient bytes then function
 * reads available bytes and returns the count of bytes that was read
 *
 * return number of bytes actually read
 * */

int UART_Read(
int iPort,                      /* port identifier */
int iBytes,                     /* max number of bytes that should be read */
unsigned char *pcBytes);        /* buffer to be used to read into FIFO */

/*
 * UART BytesInRx - returns number of bytes available in RX fifo that can be read
 *
 * return number of bytes in the RX fifo
 * */

int UART_BytesInRx(
int iPort);                 /* port identifier */

#endif /* UART_DRIVER_H */
