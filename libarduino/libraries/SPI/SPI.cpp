/*
 * Copyright (c) 2013 by Anuj Deshpande <anujdeshpande92@gmail.com>
 * Copyright (c) 2013 by Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 * SPI Library for Userspace Arduino
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "SPI.h"
#include "linux-virtual.h"
#include <assert.h>

SPIClass SPI;

SPIClass::SPIClass()
{
  ret = 0;
  fd = -1;
  device = "/dev/spidev1.0";
  bitOrder = 0;
  tr.len = 1;
  tr.speed_hz = 4000000; // default set to 4 Mhz
  tr.bits_per_word = 8;
}

void SPIClass::begin()
{
    fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Can't open device");
        abort();
    }
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &tr.bits_per_word);
    if (ret == -1)
        perror("SPI_IOC_WR_BITS_PER_WORD not set");
}

byte SPIClass::transfer(byte txData)
{
    byte rxData = 0xFF;
    tr.tx_buf = (__u64) &txData;
    tr.rx_buf = (__u64) &rxData;
    tr.len = sizeof(byte);
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0)
        perror("SPI_IOC_MESSAGE not sent");

    return rxData;
}

void SPIClass::setBitOrder(uint8_t bOrder)
{
    if (bOrder == LSBFIRST) {
        bitOrder = SPI_LSB_FIRST; // TODO: this fails as the HW doesn't support it
                                  // code could be modified to do a bit reversal instead
    }
    else if (bOrder == MSBFIRST) {
        bitOrder = 0;
    }
    else {
        assert(0); // invalid bOrder provided
    }

    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &bitOrder) < 0) {
        perror("Failed to set SPI bit justification\n");
    }
}

void SPIClass::setDataMode(uint8_t mode)
{
    if ((mode == SPI_MODE0) | (mode == SPI_MODE1) |
        (mode == SPI_MODE2) | (mode == SPI_MODE3)
        ) {
        ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
        if (ret == -1)
            perror("SPI_IOC_WR_MODE not set");
    } else {
        perror("INVALID DATA_MODE set");
    }
}

void SPIClass::setClockDivider(uint32_t rate) {
   if( 	(rate == SPI_CLOCK_DIV2) | 
	(rate == SPI_CLOCK_DIV4) | 
	(rate == SPI_CLOCK_DIV16) | 
	(rate == SPI_CLOCK_DIV32) | 
	(rate == SPI_CLOCK_DIV32) | 
	(rate == SPI_CLOCK_DIV64) | 
	(rate == SPI_CLOCK_DIV128)
	)										 //changes
  {
  	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &rate);
        if (ret == -1)
		perror("SPI_IOC_WR_MAX_SPEED_HZ not set");
  }
  else
  	perror("INVALID SPI_CLOCK_DIV set");
}

void SPIClass::end() {
  close(fd);
}
