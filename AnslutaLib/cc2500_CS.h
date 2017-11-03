/***************************************************************
 *
 *  Command Strobe registers
 *
 ***************************************************************/

#define CC2500_SRES     0x30	// Reset chip
#define CC2500_SFSTXON  0x31	// Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
#define CC2500_SXOFF	0x32	// Turn off crystal oscillator.
#define CC2500_SCAL		0x33	// Calibrate frequency synthesizer and turn it off.
#define CC2500_SRX      0x34    // Enable RX. Perform calibration if enabled
#define CC2500_STX      0x35    // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_SIDLE    0x36    // Exit RX / TX
#define CC2500_SWOR		0x38	// Start automatic RX polling sequence (Wake-on-Radio)
#define CC2500_SPWD		0x39	// Enter power down mode when CSn goes high.
#define CC2500_SFRX     0x3A    // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_SFTX     0x3B    // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_SWORRST	0x3C	// Reset real time clock to Event1 value.
#define CC2500_SNOP		0x3D	// No operation. May be used to get access to the chip status byte.
#define CC2500_FIFO     0x3F    // TX and RX FIFO


