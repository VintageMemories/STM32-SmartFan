/*****************************************************************************
 * 			4接口硬件 IIC OLED 例程
 *			接线说明:
 *              ----------------------------------------------------------------
 *              GND    电源地
 *              VCC     接5V或3.3v电源
  *             SCK     PB6
  *             SDA     PB7
 *              ----------------------------------------------------------------
 *			汉字取模格式为：
 *  				阴码、列行式、逆向
 * 库版本  ：V3.5.0
******************************************************************************/
#include "iic_oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"

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
    OLED_SDIN_Clr();
    OLED_SDIN_Set();
}

void IIC_Wait_Ack()
{
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
void IIC_OLED_WR_Byte(uint8_t dat, uint8_t cmd)
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
// 设定起始坐标点
void IIC_OLED_Set_Pos(unsigned char x, unsigned char y)
{
    IIC_OLED_WR_Byte(0xb0 + y, OLED_CMD);
    IIC_OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    IIC_OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

// 将OLED从休眠中唤醒
void IIC_OLED_Display_On(void)
{
    IIC_OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC
    IIC_OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
    IIC_OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
// 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
void IIC_OLED_Display_Off(void)
{
    IIC_OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC
    IIC_OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
    IIC_OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
// OLED清屏
void IIC_OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        IIC_OLED_WR_Byte(0xb0 + i, OLED_CMD); 
        IIC_OLED_WR_Byte(0x00, OLED_CMD);    
        IIC_OLED_WR_Byte(0x10, OLED_CMD);     
        for (n = 0; n < 128; n++)
            IIC_OLED_WR_Byte(0, OLED_DATA);
    } 
}
void IIC_OLED_On(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        IIC_OLED_WR_Byte(0xb0 + i, OLED_CMD); 
        IIC_OLED_WR_Byte(0x00, OLED_CMD);   
        IIC_OLED_WR_Byte(0x10, OLED_CMD);    
        for (n = 0; n < 128; n++)
            IIC_OLED_WR_Byte(1, OLED_DATA);
    } 
}
// OLED 显示字符
// x:   行坐标 0~127
// y:   列坐标 0~63
// chr：ASCLL字符
void IIC_OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr)
{
    uint8_t t = 0, c=0;	  
    c = chr - ' '; 
    if (x > Max_Column - 1)
    {
        x = 0;
        y = y + 2;
    }
    IIC_OLED_Set_Pos(x,y);	
    //自左到右循环输入
    for(t=0;t<8;t++)
        IIC_OLED_WR_Byte(ASCII_1608[c][t],OLED_DATA);
    IIC_OLED_Set_Pos(x,y+1);	
    //自左到右循环输入
    for(t=0;t<8;t++)
        IIC_OLED_WR_Byte(ASCII_1608[c][8+t],OLED_DATA);
}
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}
// OLED显示数字
// x:   行坐标 0~127
// y:   列坐标 0~63
// num：数字内容
// len：数字长度
void IIC_OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                IIC_OLED_ShowChar(x + 8 * t, y, ' ');
                continue;
            }
            else
                enshow = 1;
        }
        IIC_OLED_ShowChar(x + 8*t, y, temp + '0');
    }
}
// OLED显示字符串
// x:   行坐标 0~127
// y:   列坐标 0~63
// chr：ASCLL字符串
// size:字符大小 16
void IIC_OLED_ShowString(uint8_t x, uint8_t y, char *chr)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        IIC_OLED_ShowChar(x, y, chr[j]);
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
void IIC_OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t, adder = 0;
    IIC_OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        IIC_OLED_WR_Byte(Hzk[2 * no][t], OLED_DATA);
        adder += 1;
    }
    IIC_OLED_Set_Pos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        IIC_OLED_WR_Byte(Hzk[2 * no + 1][t], OLED_DATA);
        adder += 1;
    }
}
// OLED显示图片
// x0:   行起始坐标
// y0:   列起始坐标 
// x1:   行终止坐标
// y1:   列终止坐标 
// BMP： 图片内容
void IIC_OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    unsigned int j = 0;
    unsigned char x, y;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++)
    {
        IIC_OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            IIC_OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}
/******************************************************************
函数名：	OLED_DrawFont16
功能：	显示单个16X16中文字体
参数：	x,y :起点坐标 
		s:字符串地址
返回值：	无
******************************************************************/
void IIC_OLED_DrawFont16(uint16_t x, uint16_t y, char *s)
{
	uint8_t x0 = 0, y0 = 0;
	uint16_t k = 0;
	uint16_t HZnum = 0;
	
	//自动统计汉字数目
	HZnum=sizeof(Tfont16)/sizeof(typFNT_GB16);	
	
	//循环寻找匹配的Index[2]成员值
	for (k=0;k<HZnum;k++)
	{
		//对应成员值匹配
		if((Tfont16[k].Index[0]==*(s))&&(Tfont16[k].Index[1]==*(s+1)))
		{ 	
			//x方向循环执行写16行，逐行式输入
			for(y0=0;y0<2;y0++)
			{
				IIC_OLED_Set_Pos(x,y+y0);
				//每行写入两个字节，自左到右
				for(x0=0;x0<16;x0++)
				{	
					//一次写入1字节
					IIC_OLED_WR_Byte(Tfont16[k].Msk[y0*16+x0],OLED_DATA);
				}
			}
		//查找到对应点阵关键字完成绘字后立即break退出for循环，防止多个汉字重复取模显示		
		break; 
		}
	}
}

/******************************************************************
函数名：	OLED_DrawFont32
功能：	显示单个32X32中文字体
参数：	x,y :起点坐标	 
		s:字符串地址
返回值：	无
******************************************************************/ 
void IIC_OLED_DrawFont32(uint16_t x, uint16_t y, char *s)
{
	uint8_t x0 = 0, y0 = 0;
	uint16_t k = 0;
	uint16_t HZnum = 0;
	
	//自动统计汉字数目
	HZnum=sizeof(Tfont32)/sizeof(typFNT_GB32);	
	
	//循环寻找匹配的Index[2]成员值
	for (k=0;k<HZnum;k++)
	{
		//对应成员值匹配
		if((Tfont16[k].Index[0]==*(s))&&(Tfont16[k].Index[1]==*(s+1)))
		{ 	
			//x方向循环执行写16行，逐行式输入
			for(y0=0;y0<4;y0++)
			{
				IIC_OLED_Set_Pos(x,y+y0);
				//每行写入两个字节，自左到右
				for(x0=0;x0<32;x0++)
				{	
					//一次写入1字节
					IIC_OLED_WR_Byte(Tfont32[k].Msk[y0*32+x0],OLED_DATA);
				}
			}
		//查找到对应点阵关键字完成绘字后立即break退出for循环，防止多个汉字重复取模显示		
		break; 
		}
	}
} 

/******************************************************************
函数名： OLED_Show_Str
功能：	显示一个字符串,包含中英文显示
参数：	x,y :起点坐标
		str :字符串	 
		size:字体大小 16或32
返回值：	无
******************************************************************/ 
void IIC_OLED_Show_Str(uint16_t x, uint16_t y, char *str,uint8_t size)
{					
	uint16_t x0 = x;
  	uint8_t bHz = 0;				//字符或者中文，首先默认是字符
	
	if(size!=32)
		size=16;					//默认1608
	while(*str!=0)					//判断为否为结束符
	{
		if(!bHz)					//判断是字符
		{
			//如果x，y坐标超出预设lcd屏大小则换行显示
			if(x>(128-size/2)) 
			{
				//显示靠前
				x=0;
				//显示换行
				y=(y+size/8)%8;	
			}
			if(y>(8-size/8))
			{
				y=0;
			}
			if((uint8_t)*str>0x80)	//对显示的字符检查，判断是否为中文
			{
				bHz=1;				//判断为中文，则跳过显示字符改为显示中文
			}
			else			  		//确定为字符
			{		  
				if(*str==0x0D)		//判断是换行符号
				{
					y+=size;		//下一个显示的坐标换行
					x=x0;			//显示靠前
					str++;			//准备下一个字符
				}
				else				//判断不是换行符
				{
					//显示对应尺寸字符
					IIC_OLED_ShowChar(x,y,*str);
					//显示完后右移起始显示横坐标准备下次显示
					x+=size/2;
				}
				//显示地址自增，准备下一个字符
				str++; 
			}
		}
		else						//判断是中文
		{   
			//如果x，y坐标超出预设lcd屏大小则换行显示
			if(x>(128-size)) 
			{
				//显示靠前
				x=0;
				//显示换行
				y=(y+size/8)%8;	
			}
			if(y>(8-size/8))
			{
				y=0;
			}
			bHz=0;					//改为默认字符用于下次字符判断 
			if(size==32)			//判断是否为32X32大小的中文
				//显示32X32大小的中文
				IIC_OLED_DrawFont32(x,y,str);	 	
			else if(size==16)		//否则为16X16大小的中文
				//显示16X16大小的中文
			IIC_OLED_DrawFont16(x,y,str);
			//由于显示为中文，需要自增2个地址	
			str+=2;	
			//显示完后右移起始显示横坐标准备下次显示
			x+=size;			
		}
	}
}
// OLED 初始化
void IIC_OLED_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);

    IIC_OLED_WR_Byte(0xAE, OLED_CMD); //--display off
    IIC_OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    IIC_OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    IIC_OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
    IIC_OLED_WR_Byte(0xB0, OLED_CMD); //--set page address
    IIC_OLED_WR_Byte(0x81, OLED_CMD); // contract control
    IIC_OLED_WR_Byte(0xFF, OLED_CMD); //--128
    IIC_OLED_WR_Byte(0xA1, OLED_CMD); // set segment remap
    IIC_OLED_WR_Byte(0xA6, OLED_CMD); //--normal / reverse
    IIC_OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    IIC_OLED_WR_Byte(0x3F, OLED_CMD); //--1/32 duty
    IIC_OLED_WR_Byte(0xC8, OLED_CMD); // Com scan direction
    IIC_OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
    IIC_OLED_WR_Byte(0x00, OLED_CMD); //

    IIC_OLED_WR_Byte(0xD5, OLED_CMD); // set osc division
    IIC_OLED_WR_Byte(0x80, OLED_CMD); //

    IIC_OLED_WR_Byte(0xD8, OLED_CMD); // set area color mode off
    IIC_OLED_WR_Byte(0x05, OLED_CMD); //

    IIC_OLED_WR_Byte(0xD9, OLED_CMD); // Set Pre-Charge Period
    IIC_OLED_WR_Byte(0xF1, OLED_CMD); //

    IIC_OLED_WR_Byte(0xDA, OLED_CMD); // set com pin configuartion
    IIC_OLED_WR_Byte(0x12, OLED_CMD); //

    IIC_OLED_WR_Byte(0xDB, OLED_CMD); // set Vcomh
    IIC_OLED_WR_Byte(0x30, OLED_CMD); //

    IIC_OLED_WR_Byte(0x8D, OLED_CMD); // set charge pump enable
    IIC_OLED_WR_Byte(0x14, OLED_CMD); //

    IIC_OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel
}
