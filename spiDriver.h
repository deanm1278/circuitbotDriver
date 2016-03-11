/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   spiDriver.h
 * Author: Dean Miller
 *
 * Created on March 10, 2016, 8:25 PM
 */

#include <linux/ioctl.h>

#ifndef SPIDRIVER_H
#define SPIDRIVER_H

#define SERVODRV_GET_DATAWIDTH                  _IOR('q', 1, unsigned char *)
#define SERVODRV_SET_DATAWIDTH                  _IOW('q', 2, unsigned char *)
#define SERVODRV_GET_ELEMENTS_PER_TXN           _IOR('q', 3, unsigned char *)
#define SERVODRV_SET_ELEMENTS_PER_TXN           _IOW('q', 4, unsigned char *)
#define SERVODRV_WRITE_READY                    _IOR('q', 5, unsigned char *)


#define DEFAULT_DATA_WIDTH          2
#define DEFAULT_ELEMENTS_PER_TXN    3

#endif /* SPIDRIVER_H */

