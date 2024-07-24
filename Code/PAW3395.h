#ifndef PAW3395_H_
#define PAW3395_H_
#include "zf_common_headfile.h"

#define paw3395_SPI_SPEED                (10 * 1000 * 1000  )                     // 硬件 SPI 速率
#define paw3395_SPI                      (SPI_3             )                    // 硬件 SPI 号
#define paw3395_SCL_PIN_SPI              (SPI3_MAP0_SCK_B3 )                     // 硬件 SPI SCK 引脚
#define paw3395_SDA_PIN_SPI              (SPI3_MAP0_MOSI_B5)                     // 硬件 SPI MOSI 引脚
#define paw3395_SDO_PIN_SPI              (SPI3_MAP0_MISO_B4)
#define paw3395_rst_pin                  (E9)
#define paw3395_CS_PIN_SPI               (E10)

#define paw3395_motion                   (C12)

#define CS_High                         gpio_set_level(paw3395_CS_PIN_SPI,1)            //
#define CS_Low                          gpio_set_level(paw3395_CS_PIN_SPI,0)


#define PAW3395_SPIREGISTER_MOTION           0x02
#define PAW3395_SPIREGISTER_MotionBurst     0x16
#define PAW3395_SPIREGISTER_POWERUPRESET    0x3A
//Register Value
#define PAW3395_POWERUPRESET_POWERON    0x5A
// Registers bits
#define PAW3395_OP_MODE0                                         0
#define PAW3395_OP_MODE1                                         1

#define PAW3395_PG_FIRST                                         6
#define PAW3395_PG_VALID                                         7

#define PAW3395_TIMINGS_SRAD 2
#define PAW3395_TIMINGS_SRWSRR 2
#define PAW3395_TIMINGS_SWW 5
#define PAW3395_TIMINGS_SWR 5
#define PAW3395_TIMINGS_NCS_SCLK 1
#define PAW3395_TIMINGS_BEXIT 4
#define PAW3395_SPI_WRITE     0x80

#define SPI_I2S_FLAG_RXNE               ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE                ((uint16_t)0x0002)

void SPI_switch_to_PAW3395(void);

uint8_t SPI_SendReceive(uint8_t data);
uint8_t read_register(uint8_t adress);
void writr_register(uint8_t adress,uint8_t vlue);
void Power_up_sequence(void);
void Motion_Burst(uint8_t *buffer);
void Pixel_Burst_Read(uint8_t* pFrame);
void paw3395_spi_int();

void paw3395_init();
extern uint8 PAW3395_ERR,PAW3395_QUAL;
extern sint16 PAW3395_X,PAW3395_Y;
void PAW3395_READ();

#endif /* PAW3395_H_ */
