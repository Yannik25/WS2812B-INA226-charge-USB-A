/*
 * Copyright (c) 2019 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef USCI_B_I2C_H_
#define USCI_B_I2C_H_

#include <msp430.h>

 /****************************************************************
  *
  * Configuration
  *
  ****************************************************************/
//------------------------------------------------------------------------------
// void I2C_master_init(void)
//
// This function is used to initialize I2C registers in master mode, configure
// I/O pins for I2C communication, and reset I2C tracking variables
//
//------------------------------------------------------------------------------
void I2C_master_init(void);

//------------------------------------------------------------------------------
// void I2C_set_slave_address(unsigned char address)
//
// This function is used save in memory which address to communicate for with
// for the following I2C transactions. 10-bit addressing requires different configuration
//
// IN:   unsigned char address => 7-bit address of slave device
//------------------------------------------------------------------------------
void I2C_set_slave_address(unsigned char address);


/****************************************************************
 *
 * Low Level Read/Write
 *
 ****************************************************************/
//------------------------------------------------------------------------------
// void I2C_master_write(unsigned char * data_ptr, unsigned int num_bytes)
//
// This function write the contents of data_ptr to the slave device defined in the
// UCB0I2CSA register. The number of bytes send is defined by num_bytes, and will
// be followed by a STOP condition if send_stop_condition = 1
// I2C control registers are reconfigured before write to clear STOP/START conditions
//
// IN:   unsigned char * data_ptr  =>  pointer to data field to be transmitted
//       unsigned char num_bytes   =>  number of bytes to be transmitted
//------------------------------------------------------------------------------
void I2C_master_write(unsigned char * data, unsigned int num_bytes);

//------------------------------------------------------------------------------
// void I2C_master_read(unsigned char * data_ptr, unsigned int num_bytes)
//
// This function read from a slave device and save the resulting values in data_ptr.
// The number of bytes to be read is defined by num_bytes. If less than 2 bytes are
// to be read, LowPowerMode is not entered. This function can be called to tranmitt
// a re-start condition if I2C_master_write() was
// previously called with send_stop_condition = 0
//
// IN:   unsigned char * data_ptr  =>  pointer to data field to receive read data
//       unsigned char num_bytes   =>  number of bytes to be received
// OUT:  unsigned char *           => returns a pointer to data field for ease of access
//------------------------------------------------------------------------------
unsigned char * I2C_master_read (unsigned char * data, unsigned int num_bytes);

 /****************************************************************
  *
  * Register Level Read/Write
  *
  ****************************************************************/
//------------------------------------------------------------------------------
// void I2C_register_write(unsigned char reg, unsigned char val)
//
// This function transmits a data-packet containing first the reg value
// followed by the val value. It is meant to write value to a specific register
// on a device that expects such a data-packet (i.e. TCA9539).
//
// IN:   unsigned char reg  =>  register of slave device to be written to
//       unsigned char val  =>  desired value to be written to register
//------------------------------------------------------------------------------
void I2C_register_write(unsigned char reg, unsigned char val1, unsigned char val2);

//------------------------------------------------------------------------------
// unsigned char I2C_register_read(unsigned char reg)
//
// This function is designed to read the value of a specific register from a slave device.
// It will first write a single byte to the slave containing the register. It will then
// perform a re-start conditions followed by a read instruction. The resulting read value
// will be returned by this function.
//
// IN:   unsigned char reg  =>  register of slave device to be read
// OUT:  unsigned char      =>  value read from specified register of slave
//------------------------------------------------------------------------------
unsigned char I2C_register_read_char(unsigned char reg);

//------------------------------------------------------------------------------
// unsigned int  I2C_register_read(unsigned char reg)
//
// This function is designed to read the value of a specific register from a slave device.
// It will first write a single byte to the slave containing the register. It will then
// perform a re-start conditions followed by a read instruction. The resulting read value
// will be returned by this function.
//
// IN:   unsigned char reg  =>  register of slave device to be read
// OUT:  unsigned char      =>  value read from specified register of slave
//------------------------------------------------------------------------------
unsigned int  I2C_register_read_int (unsigned char reg);

//------------------------------------------------------------------------------
// unsigned char I2C_master_write_skip_stop(unsigned char * data_ptr, unsigned int num_bytes))
//
// This function is a helper function to automatically disable and re-enable send_stop_condition
// so that a subsequent read can be performed using a re-start condition.
//
// IN:   unsigned char * data_ptr  =>  pointer to data field to be transmitted
//       unsigned char num_bytes   =>  number of bytes to be transmitted
//------------------------------------------------------------------------------
void I2C_master_write_skip_stop(unsigned char * data, unsigned int num_bytes);

/****************************************************************
 *
 * NACK management
 *
 ****************************************************************/

void I2C_resetNackCount();
unsigned long I2C_getNackCount();

#endif /* USCI_B_I2C_H_ */
