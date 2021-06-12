#ifndef OPEN_DEV_H
#define OPEN_DEV_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#define TRUE 1
#define FALSE 0
#define UART_DEVICE "/dev/ttyUSB0"
#define SPEED 115200
#define MAX_BUFF_SIZE 16

int OpenDev(void);
void set_speed(int fd, int speed);
int set_Parity(int fd, int databits, int stopbits, int parity);
void print_hex(unsigned char *chr, int num);

#endif