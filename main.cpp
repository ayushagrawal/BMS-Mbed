#include "mbed.h"
#include "UsedFunctions.h"

 
const int TOTAL_IC = 1;
uint8_t tx_cfg[TOTAL_IC][6];
uint8_t rx_cfg[TOTAL_IC][8];
uint16_t cell_codes[TOTAL_IC][12];
int8_t err= 0;
int8_t err1=0;
Serial PC(USBTX, USBRX); // USB tx and rx

void init_config()
{
  for(int i=0; i<TOTAL_IC; i++)
  {
    tx_cfg[i][0] = 0x04;
    tx_cfg[i][1] = 0x00;
    tx_cfg[i][2] = 0x00;
    tx_cfg[i][3] = 0x00;
    tx_cfg[i][4] = 0x00;
    tx_cfg[i][5] = 0x10;    //Currently DCTO is disabled(no balancing)
  }
}

void print_cells()
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    PC.printf(" IC ");
    PC.printf("%d",current_ic+1);
    for(int i=0; i<12; i++)
    {
      PC.printf(" C");
      PC.printf("%x",(int)i+1);
      PC.printf(":");
      PC.printf("%f",cell_codes[current_ic][i]*.0001);
      PC.printf(",");
    }
     PC.printf("\n");
  }
}


                               // setting P8 as Slave Select pin     
int main()
{
  PC.baud(9600);
  init_config();          //Initializes the configuartion array
  LTC6804_initialize();   //Initializes the IC hardware
  PC.printf("Writing the configuration array \n");
  wakeup_sleep();
  LTC6804_wrcfg(TOTAL_IC,tx_cfg);
  wait_ms(100);
  err1=LTC6804_rdcfg(TOTAL_IC,rx_cfg);
  PC.printf(" %d",(int)err1);
  if(err1==0)
  {
    PC.printf("Written the configuration array \n");
  }
  else
  {
    PC.printf("Write Failed \n");
  }
  while(1)
  {
    wakeup_idle();
    PC.printf("Starting Voltage conversion \n");
    LTC6804_adcv();             //Starting the voltage conversion
    wakeup_idle();
    PC.printf("Reading cell voltages \n");
    err= LTC6804_rdcv(0, TOTAL_IC, cell_codes);    //gets the voltages on the array and returns if there is a PEC error
    if((int)err < 0)
    {
      PC.printf("PEC error detected \n");
      while ((int)err<0)
      {
        err= LTC6804_rdcv(0, TOTAL_IC,cell_codes);
        PC.printf("Error Detected \n");
        print_cells();

      }
    }
    else if((int)err == 0)
    {
     print_cells();
      PC.printf("\n"); 
    }
    wait(3);
  }
}
