11/13/2012:

This is a slightly modified version of the I2C-rev5 library.  Additional functions were added to provide wider support (ala LCD).  Mostly this involved removing the Command operand from functions, as not all I2C devices use that during I2C writes:

    uint8_t write(uint8_t, char*);
    uint8_t write(uint8_t, uint8_t*, uint8_t);
    uint8_t write(uint8_t, uint8_t, uint8_t, uint8_t);
