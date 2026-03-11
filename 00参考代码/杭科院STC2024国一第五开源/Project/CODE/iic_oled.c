#include "SystemConfig.h"
#include "iic_oled.h"
#include "stdlib.h"
// #include "oledfont.h"

/**********************************************
//IIC Start
**********************************************/
void IIC_Start()
{

    OLED_SCLK_Set();
    OLED_SDIN_Set();
    OLED_SDIN_Clr();
    OLED_SCLK_Clr();
}

/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop()
{
    OLED_SCLK_Set();
    //	OLED_SCLK_Clr();
    OLED_SDIN_Clr();
    OLED_SDIN_Set();
}

void IIC_Wait_Ack()
{

    // GPIOB->CRH &= 0XFFF0FFFF;	//????PB12???????????
    // GPIOB->CRH |= 0x00080000;
    //	OLED_SDA = 1;
    //	delay_us(1);
    // OLED_SCL = 1;
    // delay_us(50000);
    /*	while(1)
        {
            if(!OLED_SDA)				//?ж?????????OLED ??????
            {
                //GPIOB->CRH &= 0XFFF0FFFF;	//????PB12??????????????
                //GPIOB->CRH |= 0x00030000;
                return;
            }
        }
    */
    OLED_SCLK_Set();
    OLED_SCLK_Clr();
}
/**********************************************
// IIC Write byte
**********************************************/
void Write_IIC_Byte(unsigned char IIC_Byte)
{
    unsigned char i;
    unsigned char m, da;
    da = IIC_Byte;
    OLED_SCLK_Clr();
    for (i = 0; i < 8; i++)
    {
        m = da;
        //	OLED_SCLK_Clr();
        m = m & 0x80;
        if (m == 0x80)
        {
            OLED_SDIN_Set();
        }
        else
            OLED_SDIN_Clr();
        da = da << 1;
        OLED_SCLK_Set();
        OLED_SCLK_Clr();
    }
}
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
    IIC_Start();
    Write_IIC_Byte(0x78); // Slave address,SA0=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x00); // write command
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Command);
    IIC_Wait_Ack();
    IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    IIC_Start();
    Write_IIC_Byte(0x78); // D/C#=0; R/W#=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x40); // write data
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Data);
    IIC_Wait_Ack();
    IIC_Stop();
}
void OLED_WR_Byte(unsigned dat, unsigned cmd)
{
    if (cmd)
    {

        Write_IIC_Data(dat);
    }
    else
    {
        Write_IIC_Command(dat);
    }
}

/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
    unsigned char m, n;
    for (m = 0; m < 8; m++)
    {
        OLED_WR_Byte(0xb0 + m, 0); // page0-page1
        OLED_WR_Byte(0x00, 0);     // low column start address
        OLED_WR_Byte(0x10, 0);     // high column start address
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(fill_Data, 1);
        }
    }
}

/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
    unsigned int m;
    for (; Del_50ms > 0; Del_50ms--)
        for (m = 6245; m > 0; m--)
            ;
}

void Delay_1ms(unsigned int Del_1ms)
{
    unsigned char j;
    while (Del_1ms--)
    {
        for (j = 0; j < 123; j++)
            ;
    }
}

// 设定起始坐标点
void MyOLED_Set_Pos(uint8  x, uint8  y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

// 将OLED从休眠中唤醒
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC????
    OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
// 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC????
    OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
// OLED清屏
void OLED_Clear(void)
{
    uint8 i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); 
        OLED_WR_Byte(0x00, OLED_CMD);    
        OLED_WR_Byte(0x10, OLED_CMD);     
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    } 
}
void OLED_On(void)
{
    uint8 i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); 
        OLED_WR_Byte(0x00, OLED_CMD);   
        OLED_WR_Byte(0x10, OLED_CMD);    
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(1, OLED_DATA);
    } 
}
// OLED 显示字符
// x:   行坐标 0~127
// y:   列坐标 0~63
// chr：ASCLL字符
// size:字符大小 16或12
void OLED_ShowChar(uint8 x, uint8 y, uint8 chr, uint8 Char_Size)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; 
    if (x > Max_Column - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16)
    {
        MyOLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(oled_8x16[c * 16 + i], OLED_DATA);
        MyOLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(oled_8x16[c * 16 + i + 8], OLED_DATA);
    }
    else
    {
        MyOLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WR_Byte(oled_6x8[c][i], OLED_DATA);
    }
}
static uint32 oled_pow(uint8 m, uint8 n)
{
    uint32 result = 1;
    while (n--)
        result *= m;
    return result;
}
// OLED显示数字
// x:   行坐标 0~127
// y:   列坐标 0~63
// num：数字内容
// len：数字长度
// size:字符大小 16或12
void OLED_ShowNumber(uint8 x, uint8 y, uint8 num, uint8 len, uint8 size)
{
    uint8 t, temp;
    uint8 enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size);
    }
}
// OLED显示字符串
// x:   行坐标 0~127
// y:   列坐标 0~63
// chr：ASCLL字符串
// size:字符大小 16或12
void OLED_ShowString(uint8 x, uint8 y, char *chr, uint8 Char_Size)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}
// OLED显示汉字
// x:   行坐标
// y:   列坐标 
// no： 汉字序号（Hzk[]中）
void OLED_ShowCHinese(uint8 x, uint8 y)//, uint8 no
{
    uint8 t, adder = 0;
    MyOLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        // OLED_WR_Byte(Hzk[2 * no][t], OLED_DATA);
        adder += 1;
    }
    MyOLED_Set_Pos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        // OLED_WR_Byte(Hzk[2 * no + 1][t], OLED_DATA);
        adder += 1;
    }
}
// OLED显示图片
// x0:   行起始坐标
// y0:   列起始坐标 
// x1:   行终止坐标
// y1:   列终止坐标 
// BMP： 图片内容
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    unsigned int j = 0;
    unsigned char x, y;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++)
    {
        MyOLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

// OLED 初始化
void MyOLED_Init(void)
{
    gpio_mode(P25,GPO_PP);
    gpio_mode(P24,GPO_PP);

        // delay_ms(800);
    OLED_WR_Byte(0xAE, OLED_CMD); //--display off
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
    OLED_WR_Byte(0xB0, OLED_CMD); //--set page address
    OLED_WR_Byte(0x81, OLED_CMD); // contract control
    OLED_WR_Byte(0xFF, OLED_CMD); //--128
    OLED_WR_Byte(0xA1, OLED_CMD); // set segment remap
    OLED_WR_Byte(0xA6, OLED_CMD); //--normal / reverse
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3F, OLED_CMD); //--1/32 duty
    OLED_WR_Byte(0xC8, OLED_CMD); // Com scan direction
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
    OLED_WR_Byte(0x00, OLED_CMD); //

    OLED_WR_Byte(0xD5, OLED_CMD); // set osc division
    OLED_WR_Byte(0x80, OLED_CMD); //

    OLED_WR_Byte(0xD8, OLED_CMD); // set area color mode off
    OLED_WR_Byte(0x05, OLED_CMD); //

    OLED_WR_Byte(0xD9, OLED_CMD); // Set Pre-Charge Period
    OLED_WR_Byte(0xF1, OLED_CMD); //

    OLED_WR_Byte(0xDA, OLED_CMD); // set com pin configuartion
    OLED_WR_Byte(0x12, OLED_CMD); //

    OLED_WR_Byte(0xDB, OLED_CMD); // set Vcomh
    OLED_WR_Byte(0x30, OLED_CMD); //

    OLED_WR_Byte(0x8D, OLED_CMD); // set charge pump enable
    OLED_WR_Byte(0x14, OLED_CMD); //

    OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel

}