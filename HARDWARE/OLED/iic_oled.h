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
#ifndef __OLED_IIC_H
#define	__OLED_IIC_H
#include "sys.h"
#include "stdlib.h"	    	
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口操作----------------  					   

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_6)//SCL
#define OLED_SCLK_Set() GPIO_SetBits(GPIOB,GPIO_Pin_6)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_7)//SDA
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB,GPIO_Pin_7)

 		     
#define OLED_CMD  0	//命令模式
#define OLED_DATA 1	//数据模式

//OLED操作方法
void IIC_Start();
void IIC_Stop();
void IIC_Wait_Ack();
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);
void IIC_OLED_WR_Byte(uint8_t dat,uint8_t cmd);  
void IIC_OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);

void IIC_OLED_Display_On(void);
void IIC_OLED_Display_Off(void);	   							   		    
void IIC_OLED_Init(void);
void IIC_OLED_Clear(void);

void IIC_OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void IIC_OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len);
void IIC_OLED_ShowString(uint8_t x,uint8_t y, char *p);	 
void IIC_OLED_Set_Pos(unsigned char x, unsigned char y);
void IIC_OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void IIC_OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void IIC_OLED_DrawFont16(uint16_t x, uint16_t y, char *s);
void IIC_OLED_DrawFont32(uint16_t x, uint16_t y, char *s);
void IIC_OLED_Show_Str(uint16_t x, uint16_t y, char *str,uint8_t size);

#endif  
