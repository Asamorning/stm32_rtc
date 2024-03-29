

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"	// basic

void initUart(UART_HandleTypeDef *inHuart);
uint8_t getChar();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//int _write(int file, char*p, int len);

#endif /* INC_UART_H_ */
