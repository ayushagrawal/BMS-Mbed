//These are first added directly
//Then corresponding changes in the definitions are made
#include "mbed.h"
#include "Defined.h"
#include "LTC68042.h"

SPI spi(p5,p6,p7); // mosi, miso, sclk
DigitalOut cs(p8);
const int test_ic = 1 ; // Address of IC to be tested
void spi_enable(uint8_t spi_clock_divider) // Configures SCK frequency. Use constant defined in header file.
{
   spi.format(8,3);
   spi.frequency(20000);
}


void wakeup_sleep()
{
  cs=0;
  wait(1); // Guarantees the LTC6804 will be in standby
  cs=1;
}

uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.

void set_adc(uint8_t MD, //ADC Mode
             uint8_t DCP, //Discharge Permit
             uint8_t CH, //Cell Channels to be measured
             uint8_t CHG //GPIO Channels to be measured
             )
{
  uint8_t md_bits;
  
  md_bits = (MD & 0x02) >> 1;
  ADCV[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  ADCV[1] =  md_bits + 0x60 + (DCP<<4) + CH;
 
  md_bits = (MD & 0x02) >> 1;
  ADAX[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  ADAX[1] = md_bits + 0x60 + CHG ;
  
}

void LTC6804_initialize()
{
  spi_enable(SPI_CLOCK_DIV16);
  set_adc(MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);
}

void wakeup_idle()
{
  cs=0;
  wait(1); //Guarantees the isoSPI will be in ready mode
  cs=1;
}

void spi_write(int8_t  data)  // Byte to be written to SPI port
{
  //SPDR = data;                  //! 1) Start the SPI transfer
  //while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complet
  spi.write(data);

}

// Read and write a data byte using the SPI hardware
// Returns the data byte read

uint16_t pec15_calc(uint8_t len, uint8_t *data)
{
    uint16_t remainder,addr;
    
    remainder = 16;//initialize the PEC
    for(uint8_t i = 0; i<len;i++) // loops for each byte in data array
    {
        addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address 
        remainder = (remainder<<8)^crc15Table[addr];
    }
    return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

int8_t spi_read(int8_t  data) //!The data byte to be written
{
  int8_t output;
  output=(int8_t) spi.write(data);
  return output;
/*  char* temp = (char*) &data;
  char reqdata[8];
  for(int i=0; i<8; i++) 
  {
    reqdata[i] = spi.write(temp[i]);
  }
  int8_t DATA = (int8_t) *reqdata;
  return DATA;                  //! 3) Return the data read*/
}

void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                     )
{
  for(uint8_t i = 0; i < len; i++)
  {
     spi.write((char)data[i]);
  }
}

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port 
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                    )
{
  for(uint8_t i = 0; i < tx_len; i++)
  {
   spi_write(tx_Data[i]);
  }

  for(uint8_t i = 0; i < rx_len; i++)
  {
    rx_data[i] = (uint8_t)spi_read(0xFF);
  }

}


void LTC6804_wrcfg(uint8_t total_ic,uint8_t config[][6])
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+8; // 8 = 8*total_ic
  uint8_t *cmd;
  uint16_t temp_pec;
  uint8_t cmd_index; //command counter
  
  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
  //1
  cmd[0] = 0x00;
  cmd[1] = 0x01;
  cmd[2] = 0x3d;
  cmd[3] = 0x6e;
  
  //2
  cmd_index = 4;
  for (uint8_t current_ic = 1; current_ic > 0; current_ic--)             // 1= total_ic executes for each LTC6804 in stack,
  {                             
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each byte in the CFGR register
    {                           // i is the byte counter
    
      cmd[cmd_index] = config[current_ic-1][current_byte];      //adding the config data to the array to be sent 
      cmd_index = cmd_index + 1;                
    }
    //3
    temp_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);// calculating the PEC for each board
    cmd[cmd_index] = (uint8_t)(temp_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)temp_pec;
    cmd_index = cmd_index + 2;
  }
  
  //4
  wakeup_idle ();                                                            //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  //5
   for(int current_ic = 0; current_ic<1; current_ic++) //1 = total_ic
  {
    cmd[0] = 0x80 + (test_ic<<3); //Setting address
    //cmd[0]=0x80 + 0x00; //Use this command to manually set address
    temp_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(temp_pec >> 8);
    cmd[3] = (uint8_t)(temp_pec); 
    LT_SPI_START;
    spi_write_array(4,cmd);
    spi_write_array(8,&cmd[4+(8*current_ic)]);
    LT_SPI_END;
  }
  free(cmd);
}

void LTC6804_adcv()
{

  uint8_t cmd[4];
  uint16_t temp_pec;
  
  //1
  cmd[0] = ADCV[0];
  cmd[1] = ADCV[1];
  
  //2
  temp_pec = pec15_calc(2, ADCV);
  cmd[2] = (uint8_t)(temp_pec >> 8);
  cmd[3] = (uint8_t)(temp_pec);
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  
  //4
  LT_SPI_START;
  spi_write_array(4,cmd);
  LT_SPI_END;

}



void LTC6804_rdcv_reg(uint8_t reg,
                      uint8_t total_ic, 
                      uint8_t *data
                      )
{
  uint8_t cmd[4];
  uint16_t temp_pec;
  
  //1
  if (reg == 1)
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if(reg == 2)
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  } 
  else if(reg == 3)
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  } 
  else if(reg == 4)
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  } 

  //2
 
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  
  //4
  for(int current_ic = 0; current_ic<1; current_ic++) // 1= total_ic
  {
    cmd[0] = 0x80 + (test_ic<<3); //Setting address

    temp_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(temp_pec>>8);
    cmd[3] = (uint8_t)(temp_pec); 
    LT_SPI_START;
    spi_write_read(cmd,4,&data[current_ic*8],8);
    LT_SPI_END;
  }
}

int8_t LTC6804_rdcv(uint8_t reg,
                     uint8_t total_ic,
                     uint16_t cell_codes[][12]
                     )
{
  
  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;
  
  uint8_t *cell_data;
  int8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*1)*sizeof(uint8_t)); //  1= total_ic
  //1.a
  if (reg == 0)
  {
    //a.i
    for(uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
    {
      data_counter = 0;
      LTC6804_rdcv_reg(cell_reg, total_ic,cell_data);
      for (uint8_t current_ic = 0 ; current_ic <1; current_ic++) //  1= total_ic executes for every LTC6804 in the stack
      {                                                                   // current_ic is used as an IC counter
        //a.ii
        for(uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)                                   // This loop parses the read back data. Loops 
        {                                                                 // once for each cell voltages in the register 
          parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);
          cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
          data_counter = data_counter + 2;
        }
        //a.iii
        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1];
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT ]);
        if(received_pec != data_pec)
        {
          pec_error--;//pec_error = -1;
          break;
        }
        data_counter=data_counter+2;
      }
    }
  }
 //1.b
  else
  {
    //b.i
    
    LTC6804_rdcv_reg(reg, total_ic,cell_data);
    for (uint8_t current_ic = 0 ; current_ic < 1; current_ic++) //  1= total_ic executes for every LTC6804 in the stack
    {                                                               // current_ic is used as an IC counter
        //b.ii
        for(uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)                                       // This loop parses the read back data. Loops 
        {                                                           // once for each cell voltage in the register 
            parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8);
            cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
            data_counter= data_counter + 2;
        }
        //b.iii
        received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1];
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT * (reg-1)]);
        if(received_pec != data_pec)
        {
            pec_error--;//pec_error = -1;
        }
    }
  }
 free(cell_data);
 //2
return(pec_error);
}
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


int8_t LTC6804_rdcfg(uint8_t total_ic, uint8_t r_config[][8])
{
  const uint8_t BYTES_IN_REG = 8;
  
  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t received_pec;
  rx_data = (uint8_t *) malloc((8*1)*sizeof(uint8_t));  //  1= total_ic
  //1
  cmd[0] = 0x00;
  cmd[1] = 0x02;
  cmd[2] = 0x2b;
  cmd[3] = 0x0A;

  //2
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  //3
   for(int current_ic = 0; current_ic<1; current_ic++) //  1= total_ic
  {
    cmd[0] = 0x80 + (test_ic<<3); //Setting address
    data_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(data_pec >> 8);
    cmd[3] = (uint8_t)(data_pec);
    cs=0;
    spi_write_read(cmd,4,&rx_data[current_ic*8],8);
    cs=1;
  }

  for (uint8_t current_ic = 0; current_ic < 1; current_ic++) //  1= total_ic, executes for each LTC6804 in the stack
  {
    //4.a
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
    }
    //4.b
    received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
    data_pec = pec15_calc(6, &r_config[current_ic][0]);
    if(received_pec != data_pec)
    {
      pec_error = -1;
    }
  }
  free(rx_data);
  //5
  return(pec_error);
}
