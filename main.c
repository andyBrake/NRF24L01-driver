#include <reg52.h>
#include <intrins.h>

#define MODE 0  //MODE=1时 为发送代码   MODE=0时  为接收代码

typedef unsigned char uchar;
//****************************************IO端口定义***************************************
sbit  MISO =P2^3;
sbit  MOSI =P2^2;
sbit SCK =P2^1;
sbit CE    =P2^5;
sbit CSN =P2^0;
sbit IRQ =P3^2;

sbit led = P1^2;
//******************************************************************************************
uchar  bdata sta;   //状态标志
sbit RX_DR =sta^6;
sbit TX_DS =sta^5;
sbit MAX_RT =sta^4;
//*********************************************NRF24L01*************************************
#define TX_ADR_WIDTH    5    // 5 uints TX address width
#define RX_ADR_WIDTH    5    // 5 uints RX address width
#define TX_PLOAD_WIDTH  32   // 32 uints TX payload
#define RX_PLOAD_WIDTH  32   // 32 uints TX payload
uchar const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01}; //本地地址
uchar const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01}; //接收地址

uchar code Tx_Buf[TX_PLOAD_WIDTH]={0xff,0xee,0x11,0x22,0x33,0xaa,0xbb,0x11,0x22,0x33,0xaa,0xbb,0x11,0x22,
0x33,0xaa,0xbb,0x11,0x22,0x33,0xaa,0xbb,0x11,0x22,0x33,0xaa,0xbb,0x11,0x22,0x33,0xee,0xff};//发送数据
uchar Rx_Buf[RX_PLOAD_WIDTH];//接收数据
//***************************************NRF24L01寄存器指令*******************************************************
#define READ_REG        0x00   // 读寄存器指令
#define WRITE_REG       0x20  // 写寄存器指令
#define RD_RX_PLOAD     0x61   // 读取接收数据指令
#define WR_TX_PLOAD     0xA0   // 写待发数据指令
#define FLUSH_TX        0xE1  // 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2   // 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3   // 定义重复装载数据指令
#define NOP             0xFF   // 保留
//*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

/******************************************延时函数********************************************************/
//长延时
void Delay(unsigned int s)
{
 unsigned int i,j;
for(i=0;i<1000;i++)for(j=0;j<s;j++);
}
//短延时
void delay_ms(unsigned int x)
{
    unsigned int i,j;
    i=0;
    for(i=0;i<x;i++)
    {
       j=108;;
       while(j--);
    }
}

/************************************IO 口模拟SPI总线 代码************************************************/
uchar SPI_RW(uchar byte)

{

   uchar bit_ctr;
 for(bit_ctr=0;bit_ctr<8;bit_ctr++)
 {
   MOSI=(byte&0x80);
   
   byte=(byte<<1);
   SCK=1;
   byte|=MISO;
   //led=MISO;Delay(150);
     SCK=0;
 }
 return(byte);

}

uchar SPI_RW_Reg  (uchar  reg,uchar value) // 向寄存器REG写一个字节，同时返回状态字节
{
    uchar status;
  CSN=0;
  status=SPI_RW(reg);
    SPI_RW(value);
  CSN=1;
  return(status);
}

uchar SPI_Read (uchar  reg )
{
    uchar reg_val;
  CSN=0;
  SPI_RW(reg);
   reg_val=SPI_RW(0);
  CSN=1;
  return(reg_val);

}

uchar SPI_Write_Buf(uchar reg, uchar *pBuf, uchar bytes)
{
 uchar status,byte_ctr;

   CSN = 0;                   // Set CSN low, init SPI tranaction
   status = SPI_RW(reg);    // Select register to write to and read status byte
   for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) // then write all byte in buffer(*pBuf)
    SPI_RW(*pBuf++);
   CSN = 1;                 // Set CSN high again
   return(status);          // return nRF24L01 status byte
}
#if MODE    
/*******************************发*****送*****模*****式*****代*****码*************************************/
void TX_Mode(void)
{
 CE=0;
 
 SPI_RW_Reg(FLUSH_TX,0x00);

 SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
   SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

 SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
   SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
   SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...1a
   SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40 
   SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
 SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为2字节
   SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);

 CE=1;
 delay_ms(100);
}
void Transmit(unsigned char * tx_buf)
{
 CE=0;   //StandBy I模式 
 SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
 SPI_RW_Reg(FLUSH_TX,0x00);
 SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);     // 装载数据 
 SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      // IRQ收发完成中断响应，16位CRC，主发送
 CE=1;   //置高CE，激发数据发送
 delay_ms(150);
}

#else     
/*******************************接*****收*****模*****式*****代*****码*************************************/

uchar SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
 uchar status,uchar_ctr;
 
 CSN = 0;                      // Set CSN low, init SPI tranaction
 status = SPI_RW(reg);         // Select register to write to and read status uchar
 
 for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)
  pBuf[uchar_ctr] = SPI_RW(0);    // 
 
 CSN = 1;       
 return(status);                    // return nRF24L01 status uchar
}
/******************************************************************************************************/
/*函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
/*功能：数据读取后放如rx_buf接收缓冲区中
/******************************************************************************************************/
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
   unsigned char revale=0;
 sta=SPI_Read(STATUS); // 读取状态寄存其来判断数据接收状况
 if(RX_DR)    // 判断是否接收到数据
 {
    //CE = 0;    //SPI使能
  SPI_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
  revale =1;   //读取数据完成标志
  //Delay(100);
 }
 SPI_RW_Reg(WRITE_REG+STATUS,sta);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
 return revale;
}
/****************************************************************************************************/
/*函数：void RX_Mode(void)
/*功能：数据接收配置 
/****************************************************************************************************/

void RX_Mode(void)
{
 CE=0;
 
 SPI_RW_Reg(FLUSH_RX,0x00);
 //SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
   SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
   
 SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
   SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
   //SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...1a
   SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
 SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为2字节
   SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
 SPI_RW_Reg(WRITE_REG + CONFIG, 0x0F);

 CE=1;
 delay_ms(130);
}
//************************************串口初始化*********************************************************
void StartUART( void )
{         //波特率9600
     SCON = 0x50;
     TMOD = 0x20;
     TH1 = 0xFD;
     TL1 = 0xFD;
     PCON = 0x00;
     TR1 = 1;
}
//************************************通过串口将接收到数据发送给PC端**************************************
void R_S_Byte(uchar R_Byte)
{ 
  SBUF = R_Byte;  
    while( TI == 0 );    //查询法
    TI = 0;    
}
#endif
//************************************主函数************************************************************

void main()
{
 int i=0;
 CE=0;
 SCK=0;
 CSN=1;
 P1=0x00;

#if MODE   //发送 模式代码
 TX_Mode();
 //SPI_RW_Reg(FLUSH_RX,0x00);
 while(1)
 { 
  Transmit(Tx_Buf);
  Delay(10);
  sta=SPI_Read(READ_REG +  STATUS);
  if(TX_DS)
  { 
   P1=sta;     //8位LED显示当前STATUS状态  发送中断应使bit5 = 1 灯灭  
   Delay(100);
   SPI_RW_Reg(WRITE_REG + STATUS,sta); 
  }
  if(MAX_RT)  //如果是发送超时 
  {
   P1=0x0f;   //发送超时时 8位LED灯 bit4 = 1 灯灭
   Delay(150);  
   SPI_RW_Reg(WRITE_REG + STATUS,sta);
  }
 
 }

#else     //接收  模式代码
 StartUART();
 RX_Mode();
 Delay(0);//防止编译警告
 
 while(1)
 { 
  if(nRF24L01_RxPacket(Rx_Buf))
  {
   for(i=0;i<TX_PLOAD_WIDTH;i++)
    R_S_Byte(Rx_Buf[i]);
  }
 }
#endif
}
