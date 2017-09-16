/*******************************************************************************
 * File:    i2c_routines.h
 * Author:  Tom Storey <github.com/tomstorey>
 * 
 * Description
 *      Prototypes and variables for i2c_routines.c
 * 
 * Revision History
 * Date         Author      Detail
 * 13/08/2017   Tom         Created
 ******************************************************************************/

#ifndef XC_HEADER_I2C_ROUTINES_H
#define	XC_HEADER_I2C_ROUTINES_H

#include <xc.h>
#include "types_consts.h"

#define I2C_WR 0                /* I2C write operation */
#define I2C_RD 1                /* I2C read operation */
#define I2C_ACK 0               /* I2C acknowledge */
#define I2C_NAK 1               /* I2C not acknowledge */
#define I2C_SEN 0               /* I2C start condition */
#define I2C_RSEN 1              /* I2C restart condition */

#define I2C_AD16 0              /* Two byte address */
#define I2C_AD8 1               /* Single byte address */

/* 
 * I2C operational error codes
 *
 * These are not standardised - they are defined for this implementation.
 */
typedef enum _i2c_status {
    I2C_OK,                     /* No errors */
    I2C_ERR_NOT_RDY,            /* Not ready for operation */
    I2C_ERR_INT_TIMEOUT,        /* Timeout while waiting for interrupt */
    I2C_ERR_WCOL,               /* Write collision, tx buffer wasnt empty */
    I2C_ERR_TX_TIMEOUT,         /* Timeout waiting for tx to complete */
    I2C_ERR_NAK,                /* Last tx byte not ACKed */
    I2C_ERR_RX_TIMEOUT,         /* Timeout waiting for byte to be received */
    I2C_ERR_OV,                 /* Overflow, rx buffer wasnt empty */
    I2C_ERR_BCL                 /* Bus collision */
} i2c_status_t;

/* I2C state machine states */
typedef enum _i2c_sm_state {
    I2C_SM_START,               /* Generate start */
    I2C_SM_STOP,                /* Generate stop */
    I2C_SM_CTRL_WD,             /* Send control word */
    I2C_SM_ADDR_HI,             /* Send high address byte */
    I2C_SM_ADDR_LO,             /* Send low address byte */
    I2C_SM_TRXLOOP,             /* Transmit/receive loop */
    I2C_SM_RD_RSTART,           /* Generate restart in read operation */
    I2C_SM_RD_CTRL_WD,          /* Re-send control word for read operation */
} i2c_sm_state_t;

void i2c_setup(void);
i2c_status_t i2c_master_wait(void);
i2c_status_t i2c_interrupt_wait(void);
i2c_status_t i2c_start(uint8_t sen_rsen);
i2c_status_t i2c_stop(void);
i2c_status_t i2c_write(char_t byte);
i2c_status_t i2c_read(char_t *dest);
i2c_status_t i2c_eeprom_ctrl_word(uint8_t ctrl_code, uint8_t cs, uint8_t r_nw);
i2c_status_t i2c_ack(uint8_t ack_nak);
i2c_status_t i2c_eeprom_read(uint8_t cs, uint16_t addr, uint8_t adsz, 
        uint16_t len, char_t *dest);
i2c_status_t i2c_eeprom_write(uint8_t cs, uint16_t addr, uint8_t adsz, 
        uint16_t len, char_t *src);
i2c_status_t i2c_eeprom_ack_poll(uint8_t cs);

#endif
