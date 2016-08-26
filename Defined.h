
// Variables for beginning and ending LT SPI transmission
#define LT_SPI_START    DigitalOut(p8,0);
#define LT_SPI_END      DigitalOut(p8,1);
#define SPI_CLOCK_DIV16   0x01
 /*! 
 
  |MD| Dec  | ADC Conversion Model|
  |--|------|---------------------|
  |01| 1    | Fast                |
  |10| 2    | Normal              |
  |11| 3    | Filtered            |
*/
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3


 /*! 
 |CH | Dec  | Channels to convert |
 |---|------|---------------------|
 |000| 0    | All Cells           |
 |001| 1    | Cell 1 and Cell 7   |
 |010| 2    | Cell 2 and Cell 8   |
 |011| 3    | Cell 3 and Cell 9   |
 |100| 4    | Cell 4 and Cell 10  |
 |101| 5    | Cell 5 and Cell 11  |
 |110| 6    | Cell 6 and Cell 12  |
*/

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6


/*!

  |CHG | Dec  |Channels to convert   | 
  |----|------|----------------------|
  |000 | 0    | All GPIOS and 2nd Ref| 
  |001 | 1    | GPIO 1               |
  |010 | 2    | GPIO 2               |
  |011 | 3    | GPIO 3               |
  |100 | 4    | GPIO 4               |
  |101 | 5    | GPIO 5               |
  |110 | 6    | Vref2                |
*/

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

//uint8_t CHG = 0; //!< aux channels to be converted
 /*!****************************************************
  \brief Controls if Discharging transitors are enabled
  or disabled during Cell conversions.
  
 |DCP | Discharge Permitted During conversion |
 |----|----------------------------------------|
 |0   | No - discharge is not permitted         |
 |1   | Yes - discharge is permitted           |
 
********************************************************/
#define DCP_DISABLED 0
#define DCP_ENABLED 1

typedef unsigned char       byte;

