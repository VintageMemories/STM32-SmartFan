#include "sys.h"
#include "usart.h"

// 蓝牙接收标志（用于单字符立即处理）
volatile uint8_t rx_flag = 0;
volatile uint8_t rx_byte = 0;

#if 1


#pragma GCC diagnostic ignored "-Wunknown-pragmas"
//确保程序中不包含任何半主机的函数
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}

#endif

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA = 0; //接收状态标记

void uart_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  //PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //初始化GPIOA.10

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;										//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART1, &USART_InitStructure);	   //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启串口接受中断
	USART_Cmd(USART1, ENABLE);					   //使能串口1
}

void Usart_Int2(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	/////////////////////////////////
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//USART1_TX   PA.2
	GPIO_usartx.GPIO_Pin = GPIO_Pin_2;
	GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_usartx);
	//USART1_RX	  PA.3
	GPIO_usartx.GPIO_Pin = GPIO_Pin_3;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_usartx);

	Usart_X.USART_BaudRate = BaudRatePrescaler;
	Usart_X.USART_WordLength = USART_WordLength_8b; //8位数据格式
	Usart_X.USART_StopBits = USART_StopBits_1;		//1位停止位
	Usart_X.USART_Parity = USART_Parity_No;
	Usart_X.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	Usart_X.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &Usart_X);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //开启接收中断
	USART_Cmd(USART2, ENABLE);
	/////////////////////////////////
}

void USART1_IRQHandler(void) //串口1中断服务程序
{
    u8 res;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
    {
        res = USART_ReceiveData(USART1); // 读取接收到的数据
        
        // ========== 设置蓝牙接收标志（用于单字符立即处理）==========
        rx_byte = res;
        rx_flag = 1;
        // ==========================================================
        
        // 接收未完成
        if ((USART_RX_STA & 0x8000) == 0)
        {
            if (USART_RX_STA & 0x4000) // 接收到了回车
            {
                if (res != 0x0a) // 不是换行
                {
                    USART_RX_STA = 0; // 接收错误，重新开始
                }
                else
                {
                    USART_RX_STA |= 0x8000; // 接收完成
                }
            }
            else // 还没收到回车
            {
                if (res == 0x0d) // 收到回车
                {
                    USART_RX_STA |= 0x4000;
                }
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0x3FFF] = res;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                    {
                        USART_RX_STA = 0; // 超出长度，重新接收
                    }
                }
            }
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
