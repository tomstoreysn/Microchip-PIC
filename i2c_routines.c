/*******************************************************************************
 * File:    i2c_routines.c
 * Author:  Tom Storey <github.com/tomstorey>
 * 
 * Description
 *      Routines for reading from/writing to I2C devices as master.
 * 
 *      Specific implementations include routines for performing read, write and
 *      ACK polling operations on EEPROMs.
 * 
 *      Generic read and write routines are provided for other implementations.
 * 
 *      Code was developed/tested against two different PIC families/devices:
 * 
 *      * PIC18F25K22
 *      * PIC24EP128MC202
 * 
 *      _PIC18 ifdef sections were developed on the PIC18 family device with the
 *      XC8 toolchain, while __PIC24E__ were developed on the PIC24 family
 *      device with XC16. These may or may not work out of the box on other
 *      devices or families that use the same toolchains without modification.
 * 
 *      Peripheral wise, code assumes the use of MSSP1 on PIC18 and I2C1 on
 *      PIC24.
 * 
 * Revision History
 * Date         Author      Detail
 * 13/08/2017   Tom         Created, with support for PIC18
 * 16/09/2017   Tom         Add support for PIC24
 ******************************************************************************/

#include "i2c_routines.h"
#include "types_consts.h"

void
i2c_setup(void)
{
#ifdef _PIC18
    PMD1bits.MSSP1MD = PERIPH_ON; /* Enable MSSP1 peripheral */
    
    SSP1CON1 = 0;
    SSP1CON1bits.SSPEN = TRUE;  /* I2C enabled */
    SSP1CON1 |= 0x08;           /* I2C master mode */
    
    SSP1CON3 = 0;
    SSP1CON3bits.SCIE = TRUE;   /* Enable interrupts for start and stop */
    SSP1CON3bits.PCIE = TRUE;   /* condition completion */
    PIE1bits.SSP1IE = FALSE;    /* Branch to ISR on interrupt? */
    
    SSP1STATbits.SMP = TRUE;    /* Slew rate control disabled */
    SSP1STATbits.CKE = TRUE;    /* SMbus compliancy enabled */
    
    /*
     * Approximate baud rate can be calculated with this formula:
     * 
     *        Fosc
     *    ------------
     *    baud / 4 - 1
     * 
     * Example:
     * 
     *  Fosc = 16000000 (16MHz)
     *  baud = 400000 (400KHz)
     * 
     * (16000000 / 400000) / 4 - 1 = 9
     * 
     * This produces a baud rate of ~380KHz. A value of 8 gives ~420KHz.
     */
    
    SSP1ADD = 14;
#elif __PIC24E__
    PMD1bits.I2C1MD = PERIPH_ON; /* Enable I2C1 peripheral */
    
    I2C1CON = 0;
    I2C1CONbits.DISSLW = TRUE;  /* Slew rate control disabled */
    I2C1CONbits.SMEN = TRUE;    /* SMbus compliancy enabled */
    
    IEC1bits.MI2C1IE = FALSE;   /* Branch to ISR on interrupt? */
    
    /*
     * Approximate baud rate can be calculated with this formula:
     * 
     *     1
     * (( ---- - Delay ) * Fcy ) - 2
     *    FSCL
     * 
     * Example:
     * 
     *  Fcy = 86000000 (86MHz or 43 MIPS)
     *  FSCL = 400000 (400KHz)
     *  Delay = 0.000000120 (120ns)
     * 
     * ((1 / 400000 - 0.000000120) * 86000000) - 2 = 202
     * 
     * This produces a baud rate of ~375KHz.
     */
    I2C1BRG = 190;
    
    I2C1CONbits.I2CEN = TRUE;   /* I2C enabled */
#else
    #error "I2C routines have not been tested on the target PIC device/family"
#endif
}

i2c_status_t
i2c_master_wait(void)
{
    /*
     * When all of the following flags are FALSE, the transceiver is idle.
     * 
     * Read/write bit LOW = no TX in progress; and
     * xBF bit(s) LOW = buffers empty; and
     * TRSTAT (PIC24) bit LOW = no TX in progress; and
     * SEN, RSEN, PEN, RCEN, ACKEN bits LOW
     */
    
    uint8_t timeout = 255;
    
#ifdef _PIC18
    while ((SSP1STATbits.R_nW || SSP1STATbits.BF || SSP1CON2 & 0x1F) && 
            timeout)
#elif __PIC24E__
    while ((I2C1STATbits.R_W || I2C1STATbits.RBF || I2C1STATbits.TBF ||
            I2C1STATbits.TRSTAT || I2C1CON & 0x1F) && timeout)
#endif
        timeout--;
        
    if (timeout == 0) 
        return I2C_ERR_NOT_RDY;
    
    return I2C_OK;
}

i2c_status_t
i2c_interrupt_wait(void)
{
    /*
     * Wait for an I2C sourced interrupt to occur.
     */
    
    uint8_t timeout = 255;
    
#ifdef _PIC18
    while (PIR1bits.SSP1IF == FALSE && PIR2bits.BCL1IF == FALSE && timeout)
        timeout--;
    
    PIR1bits.SSP1IF = FALSE;
    
    if (timeout == 0)
        return I2C_ERR_INT_TIMEOUT;
    
    if (PIR2bits.BCL1IF) {
        PIR2bits.BCL1IF = FALSE;
        return I2C_ERR_BCL;
    }
    
    PIR1bits.SSP1IF = FALSE;
#elif __PIC24E__
    while (IFS1bits.MI2C1IF == FALSE && I2C1STATbits.BCL == FALSE && timeout)
        timeout--;
    
    IFS1bits.MI2C1IF = FALSE;
    
    if (timeout == 0)
        return I2C_ERR_INT_TIMEOUT;
    
    if (I2C1STATbits.BCL) {
        I2C1STATbits.BCL = FALSE;
        return I2C_ERR_BCL;
    }
#endif
    
    return I2C_OK;
}

i2c_status_t
i2c_start(uint8_t sen_rsen)
{
    /*
     * Generate start or restart condition.
     * 
     * Args:
     *     sen_rsen: 0 for START, 1 for RESTART
     */
    i2c_status_t status = i2c_master_wait();
    
    if (status)
        return status;
    
    sen_rsen &= 0x1;
#ifdef _PIC18
    SSP1CON2 |= sen_rsen + 1;
#elif __PIC24E__
    I2C1CON |= sen_rsen + 1;
#endif
    
    return i2c_interrupt_wait();
}

i2c_status_t
i2c_stop(void)
{
    /*
     * Generate stop condition.
     */
    i2c_status_t status = i2c_master_wait();
    
    if (status)
        return status;
    
#ifdef _PIC18
    SSP1CON2bits.PEN = TRUE;
#elif __PIC24E__
    I2C1CONbits.PEN = TRUE;
#endif
    
    return i2c_interrupt_wait();
}

i2c_status_t
i2c_write(char_t byte)
{
    /*
     * Send a byte to the addressed slave.
     * 
     * Args:
     *     data: the byte to be sent
     */
    i2c_status_t status = i2c_master_wait();
    
    if (status)
        return status;
    
    uint8_t timeout = 255;
    
#ifdef _PIC18
    SSP1BUF = byte;
    
    if (SSP1CON1bits.WCOL)
        return I2C_ERR_WCOL;
    
    while (SSP1STATbits.BF && timeout)
        timeout--;
        
    if (timeout == 0)
        return I2C_ERR_TX_TIMEOUT;
    
    status = i2c_interrupt_wait();
    
    if (SSP1CON2bits.ACKSTAT)
        return I2C_ERR_NAK;
#elif __PIC24E__
    I2C1TRN = byte;
    
    if (I2C1STATbits.IWCOL)
        return I2C_ERR_WCOL;
    
    while (I2C1STATbits.TBF && I2C1STATbits.TRSTAT && timeout)
        timeout--;
    
    if (timeout == 0)
        return I2C_ERR_TX_TIMEOUT;
    
    status = i2c_interrupt_wait();
    
    if (I2C1STATbits.ACKSTAT)
        return I2C_ERR_NAK;
#endif
    
    return status;
}

i2c_status_t
i2c_read(char_t *dest)
{
    /*
     * Receive a byte into dest buffer.
     * 
     * Args:
     *     *dest: pointer to where received byte should be stored
     */
    i2c_status_t status = i2c_master_wait();
    
    if (status)
        return status;
    
    uint8_t timeout = 255;
    
#ifdef _PIC18
    SSP1CON2bits.RCEN = TRUE;
    
    while (SSP1STATbits.BF == FALSE && SSP1CON2bits.RCEN && timeout)
        timeout--;
    
    if (timeout == 0)
        return I2C_ERR_RX_TIMEOUT;
    
    status = i2c_interrupt_wait();
    *dest = SSP1BUF;
    
    if (SSP1CON1bits.SSPOV)
        return I2C_ERR_OV;
#elif __PIC24E__
    I2C1CONbits.RCEN = TRUE;
    
    while (I2C1STATbits.RBF == FALSE && I2C1CONbits.RCEN && timeout)
        timeout--;
    
    if (timeout == 0)
        return I2C_ERR_RX_TIMEOUT;
    
    status = i2c_interrupt_wait();
    *dest = I2C1RCV;
    
    if (I2C1STATbits.I2COV)
        return I2C_ERR_OV;
#endif
    
    return status;
}

i2c_status_t
i2c_eeprom_ctrl_word(uint8_t ctrl_code, uint8_t cs, uint8_t r_nw)
{
    /*
     * Build a control word and write it.
     * 
     * Args:
     *     ctrl_code: 4 bit control code
     *     cs: 3 bits matching slave chip select bits
     *     r_nw: 0 for WRITE, 1 for READ
     */
    ctrl_code <<= 4;
    cs &= 0x7;
    cs <<= 1;
    r_nw &= 0x1;
    
    return i2c_write(ctrl_code | cs | r_nw);
}

i2c_status_t
i2c_ack(uint8_t ack_nak)
{
    /*
     * Generate acknowledgement.
     * 
     * Args:
     *     ack_nak: 0 for ACK, 1 for NAK
     */
    ack_nak &= 0x1;
    
#ifdef _PIC18
    SSP1CON2bits.ACKDT = ack_nak;
    SSP1CON2bits.ACKEN = TRUE;
#elif __PIC24E__
    I2C1CONbits.ACKDT = ack_nak;
    I2C1CONbits.ACKEN = TRUE;
#endif
    
    return i2c_interrupt_wait();
}

i2c_status_t
i2c_eeprom_read(uint8_t cs, uint16_t addr, uint8_t adsz, uint16_t len,
        char_t *dest)
{
    /*
     * Read a number of bytes from EEPROM.
     * 
     * Args:
     *     cs: 3 bits matching slave chip select bits
     *     addr: 16 bits of memory address to begin reading from
     *     adsz: address size - 0 for 16 bit, 1 for 8 bit
     *     len: number of bytes to read
     *     *dest: pointer to where first received byte is stored
     */
    
    uint8_t sm_run = TRUE;
    uint16_t ctr = 0;
    i2c_status_t status = I2C_OK;
    i2c_sm_state_t i2c_sm_state = I2C_SM_START;
    
    while (sm_run && status == I2C_OK) {
        switch (i2c_sm_state) {
            case I2C_SM_START:
                status = i2c_start(I2C_SEN);
                i2c_sm_state = I2C_SM_CTRL_WD;
                break;
                
            case I2C_SM_CTRL_WD:
                status = i2c_eeprom_ctrl_word(0xA, cs, I2C_WR);
                    
                if (adsz == I2C_AD8) {
                    i2c_sm_state = I2C_SM_ADDR_LO;
                } else {
                    i2c_sm_state = I2C_SM_ADDR_HI;
                }
                
                break;
                
            case I2C_SM_ADDR_HI:
                status = i2c_write((char_t)(addr >> 8));
                i2c_sm_state = I2C_SM_ADDR_LO;
                break;
                
            case I2C_SM_ADDR_LO:
                status = i2c_write((char_t)addr);
                i2c_sm_state = I2C_SM_RD_RSTART;
                break;
                
            case I2C_SM_RD_RSTART:
                status = i2c_start(I2C_RSEN);
                i2c_sm_state = I2C_SM_RD_CTRL_WD;
                break;
                
            case I2C_SM_RD_CTRL_WD:
                status = i2c_eeprom_ctrl_word(0xA, cs, I2C_RD);
                i2c_sm_state = I2C_SM_TRXLOOP;
                break;
                
            case I2C_SM_TRXLOOP:
                while (ctr < len) {
                    status = i2c_read(dest + ctr);

                    if (status)
                        break;

                    ctr++;
                    
                    if (ctr < len)
                        i2c_ack(I2C_ACK);
                }
                
                if (status == I2C_OK)
                    i2c_ack(I2C_NAK);
                
                i2c_sm_state = I2C_SM_STOP;
                break;
                
            case I2C_SM_STOP:
            default:
                i2c_stop();
                sm_run = FALSE;
        }
    }
    
    return status;
}

i2c_status_t
i2c_eeprom_write(uint8_t cs, uint16_t addr, uint8_t adsz, uint16_t len,
        char_t *src)
{
    /*
     * Write a number of bytes to EEPROM.
     * 
     * Args:
     *     cs: 3 bits matching slave chip select bits
     *     addr: 16 bits of memory address to begin writing at
     *     adsz: address size - 0 for 16 bit, 1 for 8 bit
     *     len: number of bytes to write
     *     *src: pointer to where first byte is stored
     */
    
    uint8_t sm_run = TRUE;
    uint16_t ctr = 0;
    i2c_status_t status = I2C_OK;
    i2c_sm_state_t i2c_sm_state = I2C_SM_START;
    
    while (sm_run && status == I2C_OK) {
        switch (i2c_sm_state) {
            case I2C_SM_START:
                status = i2c_start(I2C_SEN);
                i2c_sm_state = I2C_SM_CTRL_WD;
                break;
                
            case I2C_SM_CTRL_WD:
                status = i2c_eeprom_ctrl_word(0xA, cs, I2C_WR);
                    
                if (adsz) {
                    i2c_sm_state = I2C_SM_ADDR_LO;
                } else {
                    i2c_sm_state = I2C_SM_ADDR_HI;
                }
                
                break;
                
            case I2C_SM_ADDR_HI:
                status = i2c_write((char_t)(addr >> 8));
                i2c_sm_state = I2C_SM_ADDR_LO;
                break;
                
            case I2C_SM_ADDR_LO:
                status = i2c_write((char_t)addr);
                i2c_sm_state = I2C_SM_TRXLOOP;
                break;
                
            case I2C_SM_TRXLOOP:
                while (ctr < len) {
                    status = i2c_write(*(src + ctr));

                    ctr++;

                    if (status)
                        break;
                }
                
                i2c_sm_state = I2C_SM_STOP;
                break;
                
            case I2C_SM_STOP:
            default:
                i2c_stop();
                sm_run = FALSE;
        }
    }
    
    return status;
}

i2c_status_t
i2c_eeprom_ack_poll(uint8_t cs)
{
    /*
     * Poll an EEPROM by issuing a control word with R/W bit low. If EEPROM is
     * not ready for more data, NAK error is returned.
     * 
     * Args:
     *     cs: 3 bits matching slave chip select bits
     */
    
    uint8_t sm_run = TRUE;
    i2c_status_t status = I2C_OK;
    i2c_sm_state_t i2c_sm_state = I2C_SM_START;
    
    while (sm_run && status == I2C_OK) {
        switch (i2c_sm_state) {
            case I2C_SM_START:
                status = i2c_start(I2C_SEN);
                i2c_sm_state = I2C_SM_CTRL_WD;
                break;
                
            case I2C_SM_CTRL_WD:
                status = i2c_eeprom_ctrl_word(0xA, cs, I2C_WR);
                i2c_sm_state = I2C_SM_STOP;
                break;
                
            case I2C_SM_STOP:
            default:
                i2c_stop();
                sm_run = FALSE;
        }
    }
    
    return status;
}
