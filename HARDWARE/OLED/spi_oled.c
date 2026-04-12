/*****************************************************************************
 * 			7接口硬件 SPI OLED 例程
 * 			配置硬件spi驱动oled         
 * 硬件连接：
 *              GND    电源地
 *              VCC  接5V或3.3v电源
 *              D0   接PA5（SCL或CLK）
 *              D1   接PA7（SDA或MOSI）
 *              RES  接PB2
 *              CS   接PB1
 *              DC   接PB0
 *              ----------------------------------------------------------------
 *			汉字取模格式为：
 *  				阴码、列行式、逆向
 * 库版本  ：V3.5.0
******************************************************************************/

#include "spi_oled.h"
#include "oledfont.h"
#include "stdio.h"
#include "stdlib.h"
#include "delay.h"

/**************************************************************
*功  能：OLED端口引脚初始化
*参  数: 无
*返回值: 无
**************************************************************/
void SPI_OLED_Hardware_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_7); // PA5/6/7上拉

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	// Enable SPIx
	SPI_Cmd(SPI1, ENABLE);
}

// SPI 速度设置函数
// SpeedSet:
// SPI_BaudRatePrescaler_2   2分频
// SPI_BaudRatePrescaler_8   8分频
// SPI_BaudRatePrescaler_16  16分频
// SPI_BaudRatePrescaler_256 256分频
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1 &= 0XFFC7;
	SPI1->CR1 |= SPI_BaudRatePrescaler; //设置SPI1速度
	SPI_Cmd(SPI1, ENABLE);
}

// SPIx 写一个字节
// TxData:要写入的字节
//返回值:无
uint8_t SPI1_WriteByte(uint8_t TxData)
{
	uint8_t retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if (retry > 200)
			return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if (retry > 200)
			return 0;
	}
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
}

//向SSD1106写入一个字节。
// dat:要写入的数据/命令
// cmd:数据/命令标志 0,表示命令;1,表示数据;
void SPI_OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
	//	uint8_t i;
	if (cmd)
		SPI_OLED_DC_Set();
	else
		SPI_OLED_DC_Clr();
	SPI_OLED_CS_Clr();
	SPI1_WriteByte(dat);

	SPI_OLED_CS_Set();
	SPI_OLED_DC_Set();
}
void SPI_OLED_Set_Pos(unsigned char x, unsigned char y)
{
	SPI_OLED_WR_Byte(0xb0 + y, OLED_CMD);
	SPI_OLED_WR_Byte((((x)&0xf0) >> 4) | 0x10, OLED_CMD);
	SPI_OLED_WR_Byte(((x)&0x0f), OLED_CMD);
}
//开启OLED显示
void SPI_OLED_Display_On(void)
{
	SPI_OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
	SPI_OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
	SPI_OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
//关闭OLED显示
void SPI_OLED_Display_Off(void)
{
	SPI_OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
	SPI_OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
	SPI_OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void SPI_OLED_Clear(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		SPI_OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
		SPI_OLED_WR_Byte(0x00, OLED_CMD);	  //设置显示位置—列低地址
		SPI_OLED_WR_Byte(0x10, OLED_CMD);	  //设置显示位置—列高地址
		for (n = 0; n < 128; n++)
			SPI_OLED_WR_Byte(0, OLED_DATA);
	} //更新显示
}
// // m^n函数
// static uint32_t oled_pow(uint8_t m, uint8_t n)
// {
// 	uint32_t result = 1;
// 	while (n--)
// 		result *= m;
// 	return result;
// }
//显示汉字
void SPI_OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t t, adder = 0;
	SPI_OLED_Set_Pos(x, y);
	for (t = 0; t < 16; t++)
	{
		SPI_OLED_WR_Byte(Hzk[2 * no][t], OLED_DATA);
		adder += 1;
	}
	SPI_OLED_Set_Pos(x, y + 1);
	for (t = 0; t < 16; t++)
	{
		SPI_OLED_WR_Byte(Hzk[2 * no + 1][t], OLED_DATA);
		adder += 1;
	}
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void SPI_OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
	unsigned int j = 0;
	unsigned char x, y;

	if (y1 % 8 == 0)
		y = y1 / 8;
	else
		y = y1 / 8 + 1;
	for (y = y0; y < y1; y++)
	{
		SPI_OLED_Set_Pos(x0, y);
		for (x = x0; x < x1; x++)
		{
			SPI_OLED_WR_Byte(BMP[j++], OLED_DATA);
		}
	}
}

void SPI_OLED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// PB0作为数据/指令信号引脚,PB1作为片选信号,PB2作为复位信号引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //速度50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2); // PB0,PB1,PB2输出高

	SPI_OLED_Hardware_Init();

	SPI_OLED_RST_Set();
	delay_ms(10);
	SPI_OLED_RST_Clr();
	delay_ms(20);
	SPI_OLED_RST_Set();

	SPI_OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
	SPI_OLED_WR_Byte(0x02, OLED_CMD); //---set low column address
	SPI_OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
	SPI_OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	SPI_OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
	SPI_OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
	SPI_OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	SPI_OLED_WR_Byte(0xC8, OLED_CMD); // Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	SPI_OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
	SPI_OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
	SPI_OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
	SPI_OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	SPI_OLED_WR_Byte(0x00, OLED_CMD); //-not offset
	SPI_OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
	SPI_OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
	SPI_OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
	SPI_OLED_WR_Byte(0xF1, OLED_CMD); // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	SPI_OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
	SPI_OLED_WR_Byte(0x12, OLED_CMD);
	SPI_OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
	SPI_OLED_WR_Byte(0x40, OLED_CMD); // Set VCOM Deselect Level
	SPI_OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
	SPI_OLED_WR_Byte(0x02, OLED_CMD); //
	SPI_OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
	SPI_OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
	SPI_OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
	SPI_OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
	SPI_OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel

	SPI_OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
	SPI_OLED_Clear();
	SPI_OLED_Set_Pos(0, 0);
}


/******************************************************************
函数名：	OLED_ShowChar
功能：	显示单个英文字符
参数：	x,y:字符显示位置起始坐标(y需要为8的倍数)
		num:数值（0-94）
返回值：	无
******************************************************************/ 
void SPI_OLED_ShowChar(uint16_t x,uint16_t y, uint8_t num)
{  
	uint8_t pos = 0 , t = 0;	  
	
	num=num-' ';					//得到字符偏移后的值

	//自上到下循环输入
	for(pos=0;pos<2;pos++)
	{
		SPI_OLED_Set_Pos(x,y+pos);	
		//自左到右循环输入
		for(t=0;t<8;t++)
		{
			SPI_OLED_WR_Byte(ASCII_1608[num][pos*8+t],OLED_DATA);
		}
	}	   	 	  
}

/******************************************************************
函数名：	OLED_ShowString
功能：	显示英文字符串
参数：	x,y :起点坐标
		*p:字符串起始地址
返回值：	无
******************************************************************/  
void SPI_OLED_ShowString(uint16_t x,uint16_t y,char *p)
{	 
	//判断是不是非法字符
	while((*p<='~')&&(*p>=' '))	
	{   
		//如果x，y坐标超出预设lcd屏大小则换行显示
		if(x>(128-8)) 
		{
			//显示靠前
			x=0;
			//显示换行
			y=(y+2)%8;	
		}
		if(y>(8-2))
		{
			y=0;
		}
		//循环显示字串符，坐标（x，y）,使用画笔色和背景色，尺寸和模式由参数决定
		SPI_OLED_ShowChar(x,y,*p);
		//写完一个字符，横坐标加size/2
		x+=8;
		//地址自增
		p++;
	}  
} 

/******************************************************************
函数名：	OLED_ShowInt32Num
功能：	显示数字变量值，非32位的变量可以用强制转换（int32_t）输入数值
参数：	x,y :起点坐标
		num:数值范围(-2147483648~2147483647)
		len;显示的最少位数，若显示值的长度大于len正常显示，否则补位显示
返回值：	无
******************************************************************/
void SPI_OLED_ShowInt32Num(uint16_t x,uint16_t y, int32_t num, uint8_t len)
{		 	
	char show_len[8]={0},show_num[12]={0};
	uint8_t t = 0;
	
	if(len>32)						//len值过大则退出
		return;
	//len设置最少显示位数，例：输出 %6d 
	sprintf(show_len,"%%%dd",len);
	//代入字串符换算，最终完成num2char转换
	sprintf(show_num,show_len,num);
	
	while(*(show_num+t)!=0)			//循环判断是否为结束符
	{
		//显示字串符的数值
		SPI_OLED_ShowChar(x+(8)*t,y,show_num[t]);
		//指针偏移地址值自增
		t++;
	}
} 

/******************************************************************
函数名：	OLED_DrawFont16
功能：	显示单个16X16中文字体
参数：	x,y :起点坐标 
		s:字符串地址
返回值：	无
******************************************************************/
void SPI_OLED_DrawFont16(uint16_t x, uint16_t y, char *s)
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
				SPI_OLED_Set_Pos(x,y+y0);
				//每行写入两个字节，自左到右
				for(x0=0;x0<16;x0++)
				{	
					//一次写入1字节
					SPI_OLED_WR_Byte(Tfont16[k].Msk[y0*16+x0],OLED_DATA);
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
void SPI_OLED_DrawFont32(uint16_t x, uint16_t y, char *s)
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
				SPI_OLED_Set_Pos(x,y+y0);
				//每行写入两个字节，自左到右
				for(x0=0;x0<32;x0++)
				{	
					//一次写入1字节
					SPI_OLED_WR_Byte(Tfont32[k].Msk[y0*32+x0],OLED_DATA);
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
void SPI_OLED_Show_Str(uint16_t x, uint16_t y, char *str,uint8_t size)
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
					SPI_OLED_ShowChar(x,y,*str);
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
				SPI_OLED_DrawFont32(x,y,str);	 	
			else if(size==16)		//否则为16X16大小的中文
				//显示16X16大小的中文
				SPI_OLED_DrawFont16(x,y,str);
			//由于显示为中文，需要自增3个地址	
			str+=3;	
			//显示完后右移起始显示横坐标准备下次显示
			x+=size;			
		}						 
	}   
}

