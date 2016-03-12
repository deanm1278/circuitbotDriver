/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#include <unistd.h>

#include <stdint.h>
#include <stdlib.h>

#include "libservodrv.h"

#define SERVODRV_GET_DATAWIDTH                  _IOR('q', 1, unsigned char *)
#define SERVODRV_SET_DATAWIDTH                  _IOW('q', 2, unsigned char *)
#define SERVODRV_GET_ELEMENTS_PER_TXN           _IOR('q', 3, unsigned char *)
#define SERVODRV_SET_ELEMENTS_PER_TXN           _IOW('q', 4, unsigned char *)
#define SERVODRV_WRITE_READY                    _IOR('q', 5, unsigned char *)


int servodrv_open(void){
    return open("/dev/servodrv_misc", O_RDWR);
}

int servodrv_close(int fd){
    return close(fd);
}

int servodrv_get_data_width(int fd){
    unsigned char ret;
    if (ioctl(fd, SERVODRV_GET_DATAWIDTH, &ret) == -1)
    {
        perror("ioctl get error: servodrv_get_data_width");
        return -1;
    }
    else
    {
        return ret;
    }
}

int servodrv_set_data_width(int fd, unsigned char data_width){
    if (ioctl(fd, SERVODRV_SET_DATAWIDTH, &data_width) == -1)
    {
        perror("ioctl set error: servodrv_set_data_width");
        return -1;
    }
    return 0;
}

int servodrv_get_elements_per_txn(int fd){
    unsigned char ret;
    if (ioctl(fd, SERVODRV_GET_ELEMENTS_PER_TXN, &ret) == -1)
    {
        perror("ioctl get error: servodrv_get_elements_per_txn");
        return -1;
    }
    else
    {
        return ret;
    }
}

int servodrv_set_elements_per_txn(int fd, unsigned char elements_per_txn){
    if (ioctl(fd, SERVODRV_SET_DATAWIDTH, &elements_per_txn) == -1)
    {
        perror("ioctl set error: servodrv_set_elements_per_tnx");
        return -1;
    }
    return 0;
}

int servodrv_write_ready(int fd){
    unsigned char ret;
    if (ioctl(fd, SERVODRV_WRITE_READY, &ret) == -1)
    {
        perror("ioctl get error: servodrv_write_ready");
        return -1;
    }
    else
    {
        return ret;
    }
}

size_t servodrv_write(int fd, const void *buf, size_t len){
    return write(fd, buf, len);
}