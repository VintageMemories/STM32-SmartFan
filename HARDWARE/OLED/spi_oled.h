#ifndef __SPI_OLED_H
#define __SPI_OLED_H			  	 
#include "sys.h"
#include "stdlib.h"	 

//-----------------OLED端口定义----------------  					   

#define SPI_OLED_SCLK_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_5)//CLK
#define SPI_OLED_SCLK_Set() GPIO_SetBits(GPIOA,GPIO_Pin_5)

#define SPI_OLED_SDIN_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_7)//DIN
#define SPI_OLED_SDIN_Set() GPIO_SetBits(GPIOA,GPIO_Pin_7)

#define SPI_OLED_RST_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_2)//RES
#define SPI_OLED_RST_Set() GPIO_SetBits(GPIOB,GPIO_Pin_2)

#define SPI_OLED_DC_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_0)//DC
#define SPI_OLED_DC_Set() GPIO_SetBits(GPIOB,GPIO_Pin_0)

#define SPI_OLED_CS_Clr()  GPIO_ResetBits(GPIOB,GPIO_Pin_1)//CS
#define SPI_OLED_CS_Set()  GPIO_SetBits(GPIOB,GPIO_Pin_1)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

//OLED控制用函数
void SPI_OLED_Init(void);
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler);
uint8_t SPI1_WriteByte(uint8_t TxData);
void SPI_OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void SPI_OLED_Set_Pos(unsigned char x, unsigned char y);
void SPI_OLED_Display_On(void);
void SPI_OLED_Display_Off(void);
void SPI_OLED_Clear(void);

void SPI_OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void SPI_OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void SPI_OLED_ShowChar(uint16_t x,uint16_t y, uint8_t num);
void SPI_OLED_ShowString(uint16_t x,uint16_t y,char *p);
void SPI_OLED_ShowInt32Num(uint16_t x,uint16_t y, int32_t num, uint8_t len);
void SPI_OLED_DrawFont16(uint16_t x, uint16_t y, char *s);
void SPI_OLED_DrawFont32(uint16_t x, uint16_t y, char *s);
void SPI_OLED_Show_Str(uint16_t x, uint16_t y, char *str,uint8_t size);

#endif  
