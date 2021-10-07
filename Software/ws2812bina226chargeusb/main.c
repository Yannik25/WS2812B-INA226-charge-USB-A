#include <msp430.h>
#include "ws2812.h"
#include "USCI_B_I2C.h"

//I2C Slave Address
#define SLAVE_ADDR  0x40

#define NUMBEROFLEDS    37
#define ENCODING        3       // possible values 3 and 4

#define Cal_Reg        0x05
#define Vol_Reg        0x02
#define Cur_Reg        0x04
#define Pow_Reg        0x03

void configClock(void);
void configSPI(void);
void sendBuffer(uint8_t* buffer, ledcount_t ledCount);
void sendBufferDma(uint8_t* buffer, ledcount_t ledCount);
void shiftLed(ledcolor_t* leds, ledcount_t ledCount);


int main(void) {

    //I2C on UCB0
    P1SEL0 &= ~(BIT6 + BIT7);
    P1SEL1 |= BIT6 + BIT7;


    unsigned int regvalue[1];
    double inacurrent[1];
    double inavoltage[1];

    // buffer to store encoded transport data
    uint8_t frameBuffer[(ENCODING * sizeof(ledcolor_t) * NUMBEROFLEDS)] = { 0, };

    // array with 15 rbg colors
    ledcolor_t leds[NUMBEROFLEDS] = {

            // rainbow colors
            { 0xFF, 0x00, 0x00 },
            { 0xFF, 0x3F, 0x00 },
            { 0xFF, 0x7F, 0x00 },
            { 0xFF, 0xFF, 0x00 },
            { 0x00, 0xFF, 0x00 },
            { 0x00, 0xFF, 0x3F },
            { 0x00, 0xFF, 0x7F },
            { 0x00, 0xFF, 0xFF },
            { 0x00, 0x00, 0xFF },
            { 0x3F, 0x00, 0xFF },
            { 0x7F, 0x00, 0xFF },
            { 0xFF, 0x00, 0xFF },
            { 0xFF, 0x00, 0x7F },
            { 0xFF, 0x00, 0x3F },
            { 0xFF, 0x00, 0x0F },
    };

    uint8_t update;
    ledcolor_t blankLed = {0x00, 0x00, 0x00};
    uint8_t colorIdx;
    ledcolor_t led;

    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    configClock();
    configSPI();

    // Enable Interrupts
       __bis_SR_register(GIE);

    //I2C Configuration as Master
    I2C_master_init();
    I2C_set_slave_address(SLAVE_ADDR);

    I2C_register_write(Cal_Reg, 0x03, 0x47);      //data[1] = 0x03; //02 //03
                                                  //data[2] = 0x47; //00 //47

    __delay_cycles(0x10000);

    while(1) {

        regvalue[0] = I2C_register_read_int(Vol_Reg);
        inavoltage[0] = (regvalue[0] << 8) * 0.00125;

        __delay_cycles(0x10000);

        regvalue[0] = I2C_register_read_int(Cur_Reg);
        inacurrent[0] = (regvalue[0] << 8) * 0.000061;

        // blank all LEDs
        fillFrameBufferSingleColor(&blankLed, NUMBEROFLEDS, frameBuffer, ENCODING);
        sendBufferDma(frameBuffer, NUMBEROFLEDS);
        __delay_cycles(0x100000);

        // Animation - Part1
        // set one LED after an other (one more with each round) with the colors from the LEDs array
        fillFrameBuffer(leds, NUMBEROFLEDS, frameBuffer, ENCODING);
        for(update=1; update <= NUMBEROFLEDS; update++) {
            sendBufferDma(frameBuffer, update);
            __delay_cycles(0xFFFFF);
        }
        __delay_cycles(0xFFFFFF);

       /* // Animation - Part2
        // shift previous LED pattern
        for(update=0; update < 15*8; update++) {
            shiftLed(leds, NUMBEROFLEDS);
            fillFrameBuffer(leds, NUMBEROFLEDS, frameBuffer, ENCODING);
            sendBufferDma(frameBuffer, NUMBEROFLEDS);
            __delay_cycles(0x7FFFF);
        }*/

        /*// Animation - Part3
        led = blankLed;
        // set all LEDs with the same color and simulate a sunrise
        for(colorIdx=0; colorIdx < 0xFF; colorIdx++) {
            led.red = colorIdx + 1;
            fillFrameBufferSingleColor(&led, NUMBEROFLEDS, frameBuffer, ENCODING);
            sendBufferDma(frameBuffer, NUMBEROFLEDS);
            __delay_cycles(0x1FFFF);
        }
        for(colorIdx=0; colorIdx < 0xD0; colorIdx++) {
            led.green = colorIdx;
            fillFrameBufferSingleColor(&led, NUMBEROFLEDS, frameBuffer, ENCODING);
            sendBufferDma(frameBuffer, NUMBEROFLEDS);
            __delay_cycles(0x2FFFF);
        }
        for(colorIdx=0; colorIdx < 0x50; colorIdx++) {
            led.blue = colorIdx;
            fillFrameBufferSingleColor(&led, NUMBEROFLEDS, frameBuffer, ENCODING);
            sendBufferDma(frameBuffer, NUMBEROFLEDS);
            __delay_cycles(0x3FFFF);
        }
        __delay_cycles(0xFFFFF);*/
    }
    //return 0;
}

void shiftLed(ledcolor_t* leds, ledcount_t ledCount) {
    ledcolor_t tmpLed;
    ledcount_t ledIdx;

    tmpLed = leds[ledCount-1];
    for(ledIdx=(ledCount-1); ledIdx > 0; ledIdx--) {
        leds[ledIdx] = leds[ledIdx-1];
    }
    leds[0] = tmpLed;
}

// copy bytes from the buffer to SPI transmit register
// should be reworked to use DMA
void sendBuffer(uint8_t* buffer, ledcount_t ledCount) {
    uint16_t bufferIdx;
    for (bufferIdx=0; bufferIdx < (ENCODING * sizeof(ledcolor_t) * ledCount); bufferIdx++) {
        while (!(UCA0IFG & UCTXIFG));       // wait for TX buffer to be ready
        UCA0TXBUF_L = buffer[bufferIdx];
    }
    __delay_cycles(300);
}

void sendBufferDma(uint8_t* buffer, ledcount_t ledCount) {
    DMA0SA = (__SFR_FARPTR) buffer;     // source address
    DMA0DA = (__SFR_FARPTR) &UCA0TXBUF_L;   // destination address
    // single transfer, source increment, source and destination byte access, interrupt enable
    DMACTL0 = DMA0TSEL__UCA0TXIFG;      // DMA0 trigger input
    DMA0SZ = (ENCODING * sizeof(ledcolor_t) * ledCount);
    DMA0CTL = DMADT_0 | DMASRCINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAEN; //| DMAIE;

    //start DMA
    UCA0IFG ^= UCTXIFG;
    UCA0IFG ^= UCTXIFG;
}

// configure MCLK and SMCLK to be sourced by DCO with a frequency of
//   8Mhz (3-bit encoding)
// 6.7MHz (4-bit encoding)
void configClock(void) {
    CSCTL0_H = 0xA5;
#if ENCODING == 3
    CSCTL1 = DCOFSEL_3;            // DCO frequency setting = 8MHz
#else
    CSCTL1 = DCOFSEL_2;            // DCO frequency setting = 6.7MHz
#endif
    CSCTL2 = SELS__DCOCLK + SELM__DCOCLK;
    CSCTL3 = DIVS__1 + DIVM__1;

}

void configSPI(void) {
    UCA0CTLW0 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTLW0 |= UCMST + UCSYNC + UCCKPL + UCMSB;   // 3-pin, 8-bit SPI master
                                                    // Clock polarity high, MSB
    UCA0CTLW0 |= UCSSEL__SMCLK;                     // SMCLK

#if ENCODING == 3
    UCA0BR0 = 3;        // SPICLK 8MHz/3 = 2.66MHz
    UCA0BR1 = 0;
#else
    UCA0BR0 = 2;        // SPICLK 6.7MHz/2 = 3.35MHz
    UCA0BR1 = 0;
#endif
    UCA0MCTLW = 0;                            // No modulation
    UCA0CTLW0 &= ~UCSWRST;

    // UCB0SIMO = LED data signal on P2.0
    P2SEL0 &= ~BIT0;
    P2SEL1 |= BIT0;
}
