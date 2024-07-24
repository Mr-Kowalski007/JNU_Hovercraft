// 这是一个简单的paw3395库
#include "_PAW3395.h"
static void Power_Up_Initializaton_Register_Setting(void);

void SPI_switch_to_PAW3395(void){
    ((SPI_TypeDef *)SPI3_BASE)->CTLR1 |= 0x000B;// 9M MODE3
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SPI工作的语句
// 参数说明     void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t SPI_SendReceive(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET){;}
      SPI_I2S_SendData(SPI3, data);
      while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET){;}
      return  SPI_I2S_ReceiveData(SPI3);

}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     paw3395的spi初始化
// 参数说明     void
// 返回参数     void
// 使用示例     paw3395_spi_int();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void paw3395_spi_int()
{
   spi_init(paw3395_SPI, SPI_MODE3, paw3395_SPI_SPEED, paw3395_SCL_PIN_SPI, paw3395_SDA_PIN_SPI, paw3395_SDO_PIN_SPI,SPI_CS_NULL);
   gpio_init(paw3395_CS_PIN_SPI, GPO, GPIO_LOW, SPEED_50MHZ|GPO_PUSH_PULL);
   gpio_init(paw3395_rst_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
   gpio_init(paw3395_motion, GPI, GPIO_HIGH, GPI_PULL_DOWN);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写寄存器
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void writr_register(uint8_t adress,uint8_t vlue)
{
    CS_Low;
    system_delay_125ns(1);
    SPI_SendReceive(adress+0x80);
    SPI_SendReceive(vlue);
    CS_High;
    system_delay_us(5);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读寄存器
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t read_register(uint8_t adress)
{
    unsigned char temp;
    CS_Low;
    system_delay_125ns(1);
    temp=SPI_SendReceive(adress+0x00);  //
    system_delay_us(5);
    temp=SPI_SendReceive(0xff); //
//  CS_High;
    return temp;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     paw3395的上电的语句
// 参数说明     void
// 返回参数     void
// 使用示例     Power_up_sequence();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Power_up_sequence(void)
{
 system_delay_ms(50);

 CS_Low;
 system_delay_125ns(PAW3395_TIMINGS_NCS_SCLK);
 CS_High;
 system_delay_125ns(PAW3395_TIMINGS_NCS_SCLK);
 CS_Low;
 system_delay_125ns(PAW3395_TIMINGS_NCS_SCLK);

 // Write 0x5A to POWER_UP_Reset register
 writr_register(PAW3395_SPIREGISTER_POWERUPRESET,PAW3395_POWERUPRESET_POWERON);
 //Wait for at least 5ms
 system_delay_ms(5);
 //Load Power-up initialization register setting
 Power_Up_Initializaton_Register_Setting();
 CS_High;
 system_delay_125ns(PAW3395_TIMINGS_NCS_SCLK);
 // read register from 0x02 to 0x06
 for(uint8_t reg_it=0x02; reg_it<=0x06; reg_it++)
 {
   read_register(reg_it);
   system_delay_us(PAW3395_TIMINGS_SRWSRR);
 }
 CS_High;
 system_delay_125ns(PAW3395_TIMINGS_BEXIT);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     paw3395的读位移
// 参数说明     void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Motion_Burst(uint8_t *buffer)
{
    //Lower NCS
    CS_Low;
    //Wait for t(NCS-SCLK)
    system_delay_125ns(PAW3395_TIMINGS_NCS_SCLK);
    //Send Motion_Brust address(0x16)
    SPI_SendReceive(PAW3395_SPIREGISTER_MotionBurst);   //
    //Wait for tSRAD
    system_delay_us(PAW3395_TIMINGS_SRAD);
    //Start reading SPI data continuously up to 12 bytes.
    for(uint8_t i = 0;i < 12;i++)
    {
        buffer[i] = SPI_SendReceive(0x00);
    }
    CS_High;
    system_delay_125ns(PAW3395_TIMINGS_BEXIT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     paw3395的上电写寄存器的语句
// 参数说明     void
// 返回参数     void
// 使用示例     Power_up_sequence();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void Power_Up_Initializaton_Register_Setting(void)
{
    uint8_t read_tmp;
    uint8_t i ;
    writr_register(0x7F ,0x07);
    writr_register(0x40 ,0x41);
    writr_register(0x7F ,0x00);
    writr_register(0x40 ,0x80);
    writr_register(0x7F ,0x0E);
    writr_register(0x55 ,0x0D);
    writr_register(0x56 ,0x1B);
    writr_register(0x57 ,0xE8);
    writr_register(0x58 ,0xD5);
    writr_register(0x7F ,0x14);
    writr_register(0x42 ,0xBC);
    writr_register(0x43 ,0x74);
    writr_register(0x4B ,0x20);
    writr_register(0x4D ,0x00);
    writr_register(0x53 ,0x0E);
    writr_register(0x7F ,0x05);
    writr_register(0x44 ,0x04);
    writr_register(0x4D ,0x06);
    writr_register(0x51 ,0x40);
    writr_register(0x53 ,0x40);
    writr_register(0x55 ,0xCA);
    writr_register(0x5A ,0xE8);
    writr_register(0x5B ,0xEA);
    writr_register(0x61 ,0x31);
    writr_register(0x62 ,0x64);
    writr_register(0x6D ,0xB8);
    writr_register(0x6E ,0x0F);

    writr_register(0x70 ,0x02);
    writr_register(0x4A ,0x2A);
    writr_register(0x60 ,0x26);
    writr_register(0x7F ,0x06);
    writr_register(0x6D ,0x70);
    writr_register(0x6E ,0x60);
    writr_register(0x6F ,0x04);
    writr_register(0x53 ,0x02);
    writr_register(0x55 ,0x11);
    writr_register(0x7A ,0x01);
    writr_register(0x7D ,0x51);
    writr_register(0x7F ,0x07);
    writr_register(0x41 ,0x10);
    writr_register(0x42 ,0x32);
    writr_register(0x43 ,0x00);
    writr_register(0x7F ,0x08);
    writr_register(0x71 ,0x4F);
    writr_register(0x7F ,0x09);
    writr_register(0x62 ,0x1F);
    writr_register(0x63 ,0x1F);
    writr_register(0x65 ,0x03);
    writr_register(0x66 ,0x03);
    writr_register(0x67 ,0x1F);
    writr_register(0x68 ,0x1F);
    writr_register(0x69 ,0x03);
    writr_register(0x6A ,0x03);
    writr_register(0x6C ,0x1F);

    writr_register(0x6D ,0x1F);
    writr_register(0x51 ,0x04);
    writr_register(0x53 ,0x20);
    writr_register(0x54 ,0x20);
    writr_register(0x71 ,0x0C);
    writr_register(0x72 ,0x07);
    writr_register(0x73 ,0x07);
    writr_register(0x7F ,0x0A);
    writr_register(0x4A ,0x14);
    writr_register(0x4C ,0x14);
    writr_register(0x55 ,0x19);
    writr_register(0x7F ,0x14);
    writr_register(0x4B ,0x30);
    writr_register(0x4C ,0x03);
    writr_register(0x61 ,0x0B);
    writr_register(0x62 ,0x0A);
    writr_register(0x63 ,0x02);
    writr_register(0x7F ,0x15);
    writr_register(0x4C ,0x02);
    writr_register(0x56 ,0x02);
    writr_register(0x41 ,0x91);
    writr_register(0x4D ,0x0A);
    writr_register(0x7F ,0x0C);
    writr_register(0x4A ,0x10);
    writr_register(0x4B ,0x0C);
    writr_register(0x4C ,0x40);
    writr_register(0x41 ,0x25);
    writr_register(0x55 ,0x18);
    writr_register(0x56 ,0x14);
    writr_register(0x49 ,0x0A);
    writr_register(0x42 ,0x00);
    writr_register(0x43 ,0x2D);
    writr_register(0x44 ,0x0C);
    writr_register(0x54 ,0x1A);
    writr_register(0x5A ,0x0D);
    writr_register(0x5F ,0x1E);
    writr_register(0x5B ,0x05);
    writr_register(0x5E ,0x0F);
    writr_register(0x7F ,0x0D);
    writr_register(0x48 ,0xDD);
    writr_register(0x4F ,0x03);
    writr_register(0x52 ,0x49);

    writr_register(0x51 ,0x00);
    writr_register(0x54 ,0x5B);
    writr_register(0x53 ,0x00);

    writr_register(0x56 ,0x64);
    writr_register(0x55 ,0x00);
    writr_register(0x58 ,0xA5);
    writr_register(0x57 ,0x02);
    writr_register(0x5A ,0x29);
    writr_register(0x5B ,0x47);
    writr_register(0x5C ,0x81);
    writr_register(0x5D ,0x40);
    writr_register(0x71 ,0xDC);
    writr_register(0x70 ,0x07);
    writr_register(0x73 ,0x00);
    writr_register(0x72 ,0x08);
    writr_register(0x75 ,0xDC);
    writr_register(0x74 ,0x07);
    writr_register(0x77 ,0x00);
    writr_register(0x76 ,0x08);
    writr_register(0x7F ,0x10);
    writr_register(0x4C ,0xD0);
    writr_register(0x7F ,0x00);
    writr_register(0x4F ,0x63);
    writr_register(0x4E ,0x00);
    writr_register(0x52 ,0x63);
    writr_register(0x51 ,0x00);
    writr_register(0x54 ,0x54);
    writr_register(0x5A ,0x10);
    writr_register(0x77 ,0x4F);
    writr_register(0x47 ,0x01);
    writr_register(0x5B ,0x40);
    writr_register(0x64 ,0x60);
    writr_register(0x65 ,0x06);
    writr_register(0x66 ,0x13);
    writr_register(0x67 ,0x0F);
    writr_register(0x78 ,0x01);
    writr_register(0x79 ,0x9C);
    writr_register(0x40 ,0x00);
    writr_register(0x55 ,0x02);
    writr_register(0x23 ,0x70);
    writr_register(0x22 ,0x01);

    //Wait for 1ms
    system_delay_ms(1);

    for( i = 0 ;i < 60 ;i++)
    {
        read_tmp = read_register(0x6C);
        if(read_tmp == 0x80 )
            break;
        system_delay_ms(1);
    }
    if(i == 60)
    {
        writr_register(0x7F ,0x14);
        writr_register(0x6C ,0x00);
        writr_register(0x7F ,0x00);
    }
    writr_register(0x22 ,0x00);
    writr_register(0x55 ,0x00);
    writr_register(0x7F ,0x07);
    writr_register(0x40 ,0x40);
    writr_register(0x7F ,0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介
// 参数说明     void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Pixel_Burst_Read(uint8_t* pFrame)
{
    uint8_t reg_tmp;
    //Lower NCS
    CS_Low;
    //Wait for t(NCS-SCLK)
    system_delay_125ns(PAW3395_TIMINGS_NCS_SCLK);
    writr_register(0x7F ,0x00);
    writr_register(0x40 ,0x80);

    do
    {
        reg_tmp = read_register(PAW3395_SPIREGISTER_MOTION);
        system_delay_us(PAW3395_TIMINGS_SRWSRR);
    }
    while((reg_tmp & ((1 << PAW3395_OP_MODE0) | (1 << PAW3395_OP_MODE1))) != 0);

    writr_register(0x50 ,0x01);
    writr_register(0x55 ,0x04);
    writr_register(0x58 ,0xFF);

    do
    {
        reg_tmp = read_register(0x59);
        system_delay_us(PAW3395_TIMINGS_SRWSRR);
    }
    while((reg_tmp & ((1 << PAW3395_PG_FIRST) | (1 << PAW3395_PG_VALID)))
        != ((1 << PAW3395_PG_FIRST) | (1 << PAW3395_PG_VALID)));

    pFrame[35*2]=read_register(0x58);//Read the first rawdata from register 0x58
    system_delay_us(PAW3395_TIMINGS_SRWSRR);
    for(uint8_t width = 0;width < 36;width++)
    {
        for(uint8_t height = 0;height < 36;height++)
        {
            if((width == 0)&&(height == 0))
                continue;
            do
            {
                reg_tmp = read_register(0x59);
                system_delay_us(PAW3395_TIMINGS_SRWSRR);
            }
            while(!((reg_tmp >> PAW3395_PG_VALID) & 0x01));
            pFrame[(height * 36 + (35-width)) * 2] = read_register(0x58);
            system_delay_us(PAW3395_TIMINGS_SRWSRR);
        }
    }

    writr_register(0x40 ,0x00);
    writr_register(0x50 ,0x00);
    writr_register(0x55 ,0x00);
    CS_High;
    system_delay_125ns(PAW3395_TIMINGS_BEXIT);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     paw3395的初始化加工作
// 参数说明     void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void paw3395_init()
{
    gpio_init(paw3395_CS_PIN_SPI, GPO, GPIO_LOW, SPEED_50MHZ|GPO_PUSH_PULL);
    gpio_init(paw3395_rst_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(paw3395_motion, GPI, GPIO_HIGH, GPI_PULL_DOWN);
    Power_up_sequence();
}

uint8 PAW3395_ERR=0,PAW3395_QUAL=0;
sint16 PAW3395_X,PAW3395_Y;
void PAW3395_READ(){
    static uint8 motion_burst_data[12],mask=8;
    Motion_Burst(motion_burst_data);
    PAW3395_ERR=(motion_burst_data[0]&mask);
    PAW3395_QUAL=motion_burst_data[6];
//    if(PAW3395_OK)
//    {
        PAW3395_X = (int16_t)(motion_burst_data[2] + (motion_burst_data[3] << 8));
        PAW3395_Y = (int16_t)(motion_burst_data[4] + (motion_burst_data[5] << 8));
//    }else{
//        PAW3395_X=0;
//        PAW3395_Y=0;
//    }
}
