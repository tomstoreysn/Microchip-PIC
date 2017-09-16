# Microchip PIC Code Library
This is my repository of PIC microcontroller related code.

I am working with MPLAB X and the XC toolchains. Where possible I will include code for multiple PIC families, but multi-family support is largely driven by my own requirements or happenstance, or if I get bored :-).

## i2c_routines
These are C routines for interacting with I2C devices. The primary objective was reading/writing I2C EEPROMs, but it is built upon hopefully generic enough read/write/etc routines that it should be adaptable to almost any I2C device.

PIC families (test device) supported:

* PIC18 (PIC18F25K22)
* PIC24 (PIC24EP128MC202)

EEPROM read/write example:

```c
#include "types_consts.h"
#include "i2c_routines.h"

int16_t
main(void)
{
    /* 
     * Set up the I2C peripheral.
     *
     * Additional configuration may be required through configuration
     * bits or elsewhere. e.g. on my test PIC24, I needed to assign
     * the I2C1 peripheral to the ASDA/ASCL pins. See datasheet of
     * your target device for details.
     */
    i2c_setup();

    char_t wr_buf[4] = {0x81, 0x42, 0x24, 0x18};
    char_t rd_buf[4] = {0, 0, 0, 0};
    i2c_status_t status;

    /*
     * Write some bytes to EEPROM.
     * 
     * status will be I2C_OK if write succeeded, otherwise it will be
     * an error as defined in i2c_routines.h.
     */
    status = i2c_eeprom_write(0x0,      /* CS bits */
                              0x0,      /* Write start address */
                              I2C_AD16, /* Address size */
                              4,        /* Number of bytes to write */
                              wr_buf);  /* Write buffer */

    /*
     * Poll EEPROM to determine if it is busy.
     * 
     * status will be I2C_OK if EEPROM is ready for more data, or
     * I2C_ERR_NAK if it is still busy with a previous write, otherwise
     * it will be an error as defined in i2c_routines.h.
     */
    status = i2c_eeprom_ack_poll(0x0);  /* CS bits only */
    
    /*
     * Read some bytes from EEPROM.
     * 
     * status will be I2C_OK if read succeeded, otherwise it will be
     * an error as defined in i2c_routines.h.
     * 
     * rd_buf will contain contents as read from the EEPROM, in this
     * example {0x81, 0x42, 0x24, 0x18}.
     */
    status = i2c_eeprom_read(0x0,       /* CS bits */
                             0x0,       /* Read start address */
                             I2C_AD16,  /* Address size */
                             4,         /* Number of bytes to write */
                             rd_buf);   /* Read buffer */
}
```

## types_consts.h
This header file contains typedefs etc for variable types that I use through my code, and some other common/generic constants.

By default XC8 considers the `char` type to be unsigned, while XC16 considers it signed. So you either explicitly define the signdness in code, or modify your compiler behaviour to suit - either of which is open to user error. So I like to use these types as at a glance they are a bit more descriptive, and less error prone.
