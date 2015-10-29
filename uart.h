#pragma once

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>

extern FILE uart_io;

void setup_uart(void);
uint8_t uart_kbhit(void);
