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
 *

******************************************************************************
    PINOUT I2C:
                                /|\  /|\
                                4.7k  4.7k     MSP430FR5720
                   slave         |    |        master
             -----------------   |    |   -----------------
            |              SDA|<-|----+->|P1.6/UCB0SDA  XIN|-
            |                 |  |       |                 | 32kHz
            |                 |  |       |             XOUT|-
            |              SCL|<-+------>|P1.7/UCB0SCL     |
            |                 |          |                 |


******************************************************************************/
#include <msp430.h>
#include <USCI_B_I2C.h>


signed char byteCount;              // counter to track bytes left to send/receive
unsigned char * Data_Buffer;        // buffer location for read/write data
unsigned char I2C_nack_flag;        // flag to indicate if slave nacked
unsigned long I2C_nack_count;       // Track NACKs from slave since init
unsigned char send_stop_condition;  // boolean to track when repeated starts are needed

//------------------------------------------------------------------------------
// void I2C_config_master(void)
//
// This function is used to reset I2C registers in a master configuration
//
//------------------------------------------------------------------------------
void I2C_config_master()
{
    // I2C register init
        UCB0CTLW0 = UCSWRST;            // enable software reset (allows change to following registers)
        UCB0CTLW0 |= UCMODE_3 |         // I2C mode
                    UCMST |             // Master mode
                    UCSSEL__SMCLK |     // select SMCLK as clock source
                    UCSYNC;             // synchronous mode
        UCB0BRW = 0x0028;               // Baudrate = SMCLK / 10 (1 MHz/10 = 100 kHz)
        UCB0CTLW0 &= ~UCSWRST;          // Clear software reset, continue operation (locks prev registers)
}

//------------------------------------------------------------------------------
// void I2C_master_init(void)
//
// This function is used to initialize I2C registers in master mode, configure
// I/O pins for I2C communication, and reset I2C tracking variables
//
//------------------------------------------------------------------------------
void I2C_master_init()
{
    I2C_config_master();
    UCB0IE |= UCNACKIE;             // Nack interrupt enabled
    I2C_resetNackCount();           // reset nack counter
    send_stop_condition = 1;        // send stop condition by default
}

//------------------------------------------------------------------------------
// void I2C_set_slave_address(unsigned char address)
//
// This function is used save in memory which address to communicate for with
// for the following I2C transactions. 10-bit addressing requires different configuration
//
// IN:   unsigned char address => 7-bit address of slave device
//------------------------------------------------------------------------------
void I2C_set_slave_address(unsigned char address)
{
    UCB0I2CSA = address;
}

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
void I2C_master_write(unsigned char * data_ptr, unsigned int num_bytes)
{
    //I2C_config_master();          // soft reset to clear config
    Data_Buffer = data_ptr;         // point buffer to data
    byteCount = num_bytes;          // set byteCounter
    I2C_nack_flag = 0;

    UCB0IE &= ~UCRXIE0;                 // Disable RX interrupt
    UCB0IE |= UCTXIE0;                  // Enable TX interrupt
    UCB0IFG &= ~(UCTXIFG0 + UCRXIFG0);  // Clear any pending interrupts

    UCB0CTLW0 |= UCTR + UCTXSTT;    // I2C TX, start condition

    //Enter low power mode with interrupts
    if(!I2C_nack_flag)
        __bis_SR_register(LPM0_bits | GIE);
}

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
unsigned char* I2C_master_read(unsigned char * data_ptr, unsigned int num_bytes)
{
    Data_Buffer = data_ptr;         // Point buffer to data
    byteCount = num_bytes - 2;      // Set byteCounter
    I2C_nack_flag = 0;

    UCB0IE = UCRXIE;                // Enable RX interrupt
    UCB0CTLW0 &= ~UCTR;             // Receiver mode

    // if only 1 byte needed, skip LPM
    if(num_bytes < 2)
    {
        UCB0IE &= ~UCRXIE;          // disable interrupt
        UCB0CTL1 |= UCTXSTT;        // I2C start condition
        while(UCB0CTLW0 & UCTXSTT); // wait for start condition
        UCB0CTLW0 |= UCTXSTP;       // I2C stop condition
        UCB0IE |= UCRXIE;           // enable interrupt
    }
    else
        UCB0CTLW0 |= UCTXSTT;       // I2C start condition

    // Enter low power mode with interrupts
    if(!I2C_nack_flag)
        __bis_SR_register(LPM0_bits | GIE);

    return data_ptr;
}

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
void I2C_register_write(unsigned char reg, unsigned char val1, unsigned char val2)
{
    unsigned char data[3] = {reg, val1, val2};
    I2C_master_write(data, 3);
}

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
unsigned char I2C_register_read_char(unsigned char reg)
{
    unsigned char data[3] = {reg, 0x00, 0x00};
    I2C_master_write_skip_stop(data, 1);
    I2C_master_read(data, 2);
    return data[0];
}

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
unsigned int  I2C_register_read_int (unsigned char reg)
{
    unsigned char data[3] = {reg, 0x00, 0x00};
    I2C_master_write_skip_stop(data, 1);
    I2C_master_read(data, 3);
    return (data[1] << 8) + data[0];
}
//------------------------------------------------------------------------------
// unsigned char I2C_master_write_skip_stop(unsigned char * data_ptr, unsigned int num_bytes))
//
// This function is a helper function to automatically disable and re-enable send_stop_condition
// so that a subsequent read can be performed using a re-start condition.
//
// IN:   unsigned char * data_ptr  =>  pointer to data field to be transmitted
//       unsigned char num_bytes   =>  number of bytes to be transmitted
//------------------------------------------------------------------------------
void I2C_master_write_skip_stop(unsigned char * data_ptr, unsigned int num_bytes)
{
    send_stop_condition = 0;        // tells interrupt vector to skip stop condition
    I2C_master_write(data_ptr, num_bytes);
    send_stop_condition = 1;        // reset for next transaction
}

void I2C_resetNackCount()
{
    I2C_nack_count = 0;
}

unsigned long I2C_getNackCount()
{
    return I2C_nack_count;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
        I2C_nack_count++;                   // count NACK
        I2C_nack_flag = 1;
        UCB0CTL1 |= UCTXSTP;                // I2C stop condition
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        if(byteCount <= 0) {                // if all bytes received
            UCB0CTL1 |= UCTXSTP;            // I2C stop condition
            *Data_Buffer = UCB0RXBUF;       // Get RX data
            Data_Buffer++;                  // increment buffer pointer
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        }
        *Data_Buffer = UCB0RXBUF;       // Get RX data
        Data_Buffer++;                  // increment buffer pointer
        byteCount--;
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        if(byteCount <= 0) {                // if all bytes transmitted
            if(send_stop_condition)         // skip stop if repeated-start needed
                UCB0CTL1 |= UCTXSTP;        // I2C stop condition
            __bic_SR_register_on_exit(LPM0_bits);   // Exit LPM0
            break;
        }
        UCB0TXBUF = *Data_Buffer;           // Load buffer data
        Data_Buffer++;                      // increment buffer pointer
        byteCount--;
        break;
    case USCI_I2C_UCBCNTIFG: break;         // Vector 26: BCNTIFG
    case USCI_I2C_UCCLTOIFG:                // Vector 28: clock low timeout
        UCB0CTL1 |= UCTXSTP;                // stop condition
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case USCI_I2C_UCBIT9IFG: break;         // Vector 30: 9th bit
    default: break;
    }
}

