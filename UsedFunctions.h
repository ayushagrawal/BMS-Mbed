#ifndef USEDFUNCT_H
#define USEDFUNCT_H


 



void spi_enable(uint8_t spi_clock_divider); // Configures SCK frequency. Use constant defined in header file.


void delayMicroseconds(int howmany);
void wakeup_sleep();

void set_adc(uint8_t MD, //ADC Mode
             uint8_t DCP, //Discharge Permit
             uint8_t CH, //Cell Channels to be measured
             uint8_t CHG //GPIO Channels to be measured
             );

void LTC6804_initialize();

void wakeup_idle();

void spi_write(int8_t  data);  // Byte to be written to SPI port

// Read and write a data byte using the SPI hardware
// Returns the data byte read

uint16_t pec15_calc(uint8_t len, uint8_t *data);

int8_t spi_read(int8_t  data); //!The data byte to be written
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                     );

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port 
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                    );

void LTC6804_wrcfg(uint8_t total_ic,uint8_t config[][6]);

void LTC6804_adcv();


void LTC6804_rdcv_reg(uint8_t reg,
                      uint8_t total_ic, 
                      uint8_t *data
                      );
int8_t LTC6804_rdcv(uint8_t reg,
                     uint8_t total_ic,
                     uint16_t cell_codes[][12]
                     );
/*
    LTC6804_rdcv Sequence
    
    1. Switch Statement:
        a. Reg = 0
            i. Read cell voltage registers A-D for every IC in the stack
            ii. Parse raw cell voltage data in cell_codes array
            iii. Check the PEC of the data read back vs the calculated PEC for each read register command
        b. Reg != 0 
            i.Read single cell voltage register for all ICs in stack
            ii. Parse raw cell voltage data in cell_codes array
            iii. Check the PEC of the data read back vs the calculated PEC for each read register command
    2. Return pec_error flag
*/



int8_t LTC6804_rdcfg(uint8_t total_ic, uint8_t r_config[][8]);

#endif