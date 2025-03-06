/********************************** (C) COPYRIGHT *******************************
* File Name		: main.c
* Author		: Zhiyuan Wan
* License		: MIT
* Version		: V1.0
* Date			: 2018/03/27
* Description		: CH55x做USB-MIDI桥。使用UAC-MIDI协议。此实例是一个简单的回环设备。
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ch554.h>

#include <ch554_usb.h>
#include <debug.h>

__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];	   // Endpoint 0 OUT & IN buffer, must be even address
__xdata __at (0x0040) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];	   // Endpoint 1 upload buffer
__xdata __at (0x0080) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];	//Endpoint 2 IN & OUT buffer, must be an even address

uint16_t SetupLen;
uint8_t   SetupReq,Count,UsbConfig;
const uint8_t *  pDescr;													   //USB configuration flag
USB_SETUP_REQ   SetupReqBuf;												   //Temporary Setup package
#define UsbSetupBuf	 ((PUSB_SETUP_REQ)Ep0Buffer)

/* Device descriptor */
__code uint8_t DevDesc[] = {	
#if 0
	0x12,	//Descriptor size is 18 bytes  
	0x01,  	//DEVICE Descriptor Type  
	0x10,0x01,  // usb1.1
	0x00,  	//Each interface specifies its own class information  
	0x00,  	//Each interface specifies its own Subclass information  
	0x00,  	//No protocols the device basis  
	DEFAULT_ENDP0_SIZE,  	//Maximum packet size for endpoint zero is 8  
	0x08,0x19,  //Vendor ID 
	0x70,0x20,  //Product ID
	0x00,0x01,  //The device release number is 0.01  
	0x01,  	//The manufacturer string descriptor index is 1  
	0x02,  	//The product string descriptor index is 2  
	0x03,  	//The device doesn't have the string descriptor describing the serial number  
	0x01,  	//The device has 1 possible configurations  
#else
	0x12,	//Descriptor size is 18 bytes  
	0x01,  	//DEVICE Descriptor Type  
	0x00,0x02,  // usb2.0
	0xff,  	//Each interface specifies its own class information  
	0x00,  	//Each interface specifies its own Subclass information  
	0x00,  	//No protocols the device basis  
	DEFAULT_ENDP0_SIZE,  	//Maximum packet size for endpoint zero is 8  
	0x34,0x12,  //Vendor ID 
	0x78,0x56,  //Product ID
	0x00,0x01,  //The device release number is 0.01  
	0x01,  	//The manufacturer string descriptor index is 1  
	0x02,  	//The product string descriptor index is 2  
	0x03,  	//The device doesn't have the string descriptor describing the serial number  
	0x01,  	//The device has 1 possible configurations  
#endif
	};

	/* BULK 传输端点配置 */
	#define BULK_EP_IN   0x82  // 端点2 IN
	#define BULK_EP_OUT  0x02  // 端点2 OUT
	#define BULK_PACKET_SIZE 64  // 全速USB标准包大小


__code uint8_t CfgDesc[] ={
	0x09,  	 //Descriptor size is 9 bytes  
	0x02,  	 //CONFIGURATION Descriptor Type  
	sizeof(CfgDesc) & 0xff,sizeof(CfgDesc) >> 8,
	0x01,  	 //This configuration supports 1 interfaces
	0x01,  	 //The value 1 should be used to select this configuration  一个接口
	0x00,  	 //The device doesn't have the string descriptor describing this configuration
	0x80,//0xC0,  	 //Configuration characteristics : Bit 7: Reserved (set to one) 1 Bit 6: Self-powered 1 Bit 5: Remote Wakeup 0
	0x32,  	 //Maximum power consumption of the device in this configuration is 100 mA
	
	//以下为接口0（音频接口）描述符
	0x09,  	 //Descriptor size is 9 bytes  
	0x04,  	 //INTERFACE Descriptor Type  
	0x00,  	 //The number of this interface is 0.  
	0x00,  	 //The value used to select the alternate setting for this interface is 0  
	0x02,  	 //The number of endpoints used by this interface is 0 (excluding endpoint zero)  
	0xff,  	 //The interface implements the Audio Interface class  
	0x00,  	 //
	0x00,  	 //The interface uses the IP_VERSION_01_00 Protocol  
	0x00,  	 //The device doesn't have a string descriptor describing this iInterface 

	// 端点描述符 (OUT)
	0x07,  	 //Size of the descriptor, in bytes  
	0x05,  	 //CS_INTERFACE Descriptor Type  
	BULK_EP_OUT,  	 // EndpointAddress
	0x02,  	          // TransferType=Bulk
	BULK_PACKET_SIZE, 0x00, // pack size
	0x00,

	// 端点描述符 (IN)
    0x07,       //Size of the descriptor, in bytes  
	0x05,       //CS_INTERFACE Descriptor Type  
	BULK_EP_IN, // EndpointAddress
	0x02,		// TransferType=Bulk
    BULK_PACKET_SIZE, 0x00,
	0x00
};


#if 0
/*字符串描述符*/
unsigned char  __code LangDes[]={0x04,0x03,0x09,0x04};		   //语言描述符
unsigned char  __code SerDes[]={								 // Serial number string descriptor
	0x1E,0x03,0x32,0x00,0x30,0x00,0x32,0x00,0x31,0x00,0x30,0x00,0x37,0x00,0x32,0x00,
	0x36,0x00,0x39,0x00,0x30,0x00,0x35,0x00,0x39,0x00,0x32,0x00,0x36,0x00
};
unsigned char  __code Prod_Des[]={								// Product string descriptor
	0x20,0x03,0x41,0x00,0x42,0x00,0x31,0x00,0x33,0x00,0x58,0x00,0x20,0x00,0x55,0x00,
	0x53,0x00,0x42,0x00,0x20,0x00,0x41,0x00,0x75,0x00,0x64,0x00,0x69,0x00,0x6F,0x00
};
unsigned char  __code Manuf_Des[]={
	0x10, 0x03, 0x47, 0x00, 0x65, 0x00, 0x6E, 0x00, 0x65, 0x00, 0x72, 0x00, 0x69, 0x00, 0x63, 0x00 
};
#else
/*字符串描述符*/
unsigned char  __code LangDes[]={0x04,0x03,0x09,0x04};		   //语言描述符
unsigned char  __code SerDes[]={								 // Serial number string descriptor
					0x14,0x03,
					'2',0x00,'0',0x00,'1',0x00,'8',0x00,'-',0x00,
					'3',0x00,'-',0x00,
					'2',0x00,'7',0x00
							   };
unsigned char  __code Prod_Des[]={								// Product string descriptor
					0x14,0x03,
					'C',0x00,'H',0x00,'5',0x00,'5',0x00,'x',0x00,'M',0x00,
					'I',0x00,'D',0x00,'I',0x00,
								 };
unsigned char  __code Manuf_Des[]={
	0x0A,0x03,
	0x5F,0x6c,0xCF,0x82,0x81,0x6c,0x52,0x60,
};
#endif

//MIDI参数


#define MIDI_REV_LEN  64				 //串口接收缓冲区大小
__idata uint8_t Receive_Midi_Buf[MIDI_REV_LEN];   //串口接收缓冲区
volatile __idata uint8_t MidiByteCount = 0;	  //当前缓冲区剩余待取字节数


volatile __idata uint8_t USBByteCount = 0;	  //代表USB端点接收到的数据
volatile __idata uint8_t USBBufOutPoint = 0;	//取数据指针

volatile __idata uint8_t UpPoint2_Busy  = 0;   //上传端点是否忙标志


/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description	: USB设备模式配置
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceCfg()
{
	USB_CTRL = 0x00;														   //清空USB控制寄存器
	USB_CTRL &= ~bUC_HOST_MODE;												//该位为选择设备模式
	USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;					//USB设备和内部上拉使能,在中断期间中断标志未清除前自动返回NAK
	USB_DEV_AD = 0x00;														 //设备地址初始化
	//	 USB_CTRL |= bUC_LOW_SPEED;
	//	 UDEV_CTRL |= bUD_LOW_SPEED;												//选择低速1.5M模式
	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED;											 //选择全速12M模式，默认方式
	UDEV_CTRL = bUD_PD_DIS;  // 禁止DP/DM下拉电阻
	UDEV_CTRL |= bUD_PORT_EN;												  //使能物理端口
}
/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description	: USB设备模式中断初始化
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceIntCfg()
{
	USB_INT_EN |= bUIE_SUSPEND;											   //使能设备挂起中断
	USB_INT_EN |= bUIE_TRANSFER;											  //使能USB传输完成中断
	USB_INT_EN |= bUIE_BUS_RST;											   //使能设备模式USB总线复位中断
	USB_INT_FG |= 0x1F;													   //清中断标志
	IE_USB = 1;															   //使能USB中断
	EA = 1;																   //允许单片机中断
}
/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description	: USB设备模式端点配置，模拟兼容HID设备，除了端点0的控制传输，还包括端点2批量上下传
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceEndPointCfg()
{
	UEP1_DMA = (uint16_t) Ep1Buffer;													  //端点1 发送数据传输地址
	UEP2_DMA = (uint16_t) Ep2Buffer;													  //端点2 IN数据传输地址
	UEP2_3_MOD = 0xCC;														 //端点2/3 单缓冲收发使能
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;		//端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK

	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;				//端点1自动翻转同步标志位，IN事务返回NAK
	UEP0_DMA = (uint16_t) Ep0Buffer;													  //端点0数据传输地址
	UEP4_1_MOD = 0X40;														 //端点1上传缓冲区；端点0单64字节收发缓冲区
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;				//手动翻转，OUT事务返回ACK，IN事务返回NAK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description	:CH55xUSB interrupt processing function
*******************************************************************************/
void DeviceInterrupt(void) __interrupt (INT_NO_USB)					   //USB中断服务程序,使用寄存器组1
{
	uint16_t len;
	if(UIF_TRANSFER)															//USB传输完成标志
	{
		switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
		{
		case UIS_TOKEN_IN | 1:												  //endpoint 1# 端点中断上传
			printf("yk debug endpoint 1 send data！！ UIS_TOKEN_IN\r\n");
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		   //默认应答NAK
			break;
		case UIS_TOKEN_IN | 2:												  //endpoint 2# 端点批量上传
		{
			printf("yk debug endpoint 2 send data！！ UIS_TOKEN_IN\r\n");
			UEP2_T_LEN = 0;													//预使用发送长度一定要清空
			UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		   //默认应答NAK
			UpPoint2_Busy = 0;												  //清除忙标志
		}
			break;
		case UIS_TOKEN_OUT | 2:											 //endpoint 3# 端点批量下传
			if ( U_TOG_OK )													 // 不同步的数据包将丢弃
			{
				printf("yk debug endpoint 2 recv data！！ UIS_TOKEN_OUT\r\n");
				USBByteCount = USB_RX_LEN;
				USBBufOutPoint = 0;											 //取数据指针复位
				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;	   //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
			}
			break;
		case UIS_TOKEN_OUT | 1:											 // yk endpoint 1# 端点批量下传
			printf("0 yk debug endpoint 1 recv data U_TOG_OK = %d！！ %d\r\n", U_TOG_OK, USB_RX_LEN);
			if ( U_TOG_OK )													 // 不同步的数据包将丢弃
			{
				printf("yk debug endpoint 1 recv data！！ %d\r\n", USB_RX_LEN);
				USBByteCount = USB_RX_LEN;
				USBBufOutPoint = 0;											 //取数据指针复位
				UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;	   //收到一包数据， 回复ack
			}
			break;
		case UIS_TOKEN_SETUP | 0:												//SETUP事务
			len = USB_RX_LEN;
			if(len == (sizeof(USB_SETUP_REQ)))
			{
				SetupLen = ((uint16_t)UsbSetupBuf->wLengthH<<8) | (UsbSetupBuf->wLengthL);
				len = 0;													  // 默认为成功并且上传0长度
				SetupReq = UsbSetupBuf->bRequest;
				if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )//非标准请求
				{
					printf("not stand req\r\n");
					switch( SetupReq )
					{
					
					default:
						len = 0xFF;  								 									 /*命令不支持*/
						break;
					}
				}
				else															 //标准请求
				{
					printf("stand req SetupReq=%d\r\n", SetupReq);
					switch(SetupReq)											 //请求码
					{
					case USB_GET_DESCRIPTOR: // 6
						switch(UsbSetupBuf->wValueH)
						{
						case 1:													   //设备描述符
							pDescr = DevDesc;										 //把设备描述符送到要发送的缓冲区
							len = sizeof(DevDesc);
							printf("0setupreq descriptor dev %d %d\r\n", SetupLen, len);
							break;
						case 2:														//配置描述符
							pDescr = CfgDesc;										  //把设备描述符送到要发送的缓冲区
							len = sizeof(CfgDesc);
							printf("0setupreq descriptor conf %d %d\r\n", SetupLen, len);
							break;
						case 3:
							if(UsbSetupBuf->wValueL == 0)
							{
								pDescr = LangDes;
								len = sizeof(LangDes);
							}
							else if(UsbSetupBuf->wValueL == 1)
							{
								pDescr = Manuf_Des;
								len = sizeof(Manuf_Des);
							}
							else if(UsbSetupBuf->wValueL == 2)
							{
								pDescr = Prod_Des;
								len = sizeof(Prod_Des);
							}
							else
							{
								pDescr = SerDes;
								len = sizeof(SerDes);
							}
							printf("0setupreq descriptor langue:%d==> %d %d\r\n", UsbSetupBuf->wValueL, SetupLen, len);
							break;
						default:
							len = 0xff;												//不支持的命令或者出错
							break;
						}
						if ( SetupLen > len )
						{
							SetupLen = len;	//限制总长度
						}
						len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;							//本次传输长度
						printf("len=%d, SetupLen=%d, SetupReq=%d  %d %d\r\n", len, SetupLen, SetupReq, UsbSetupBuf->wValueH, UsbSetupBuf->wValueL);
						memcpy(Ep0Buffer,pDescr,len);								  //加载上传数据
						printf("%02x %02x %02x %02x   %02x %02x %02x %02x\r\n", Ep0Buffer[0], Ep0Buffer[1], Ep0Buffer[2], Ep0Buffer[3], 
						Ep0Buffer[4], Ep0Buffer[5], Ep0Buffer[6], Ep0Buffer[7]);
						SetupLen -= len;
						pDescr += len;
						break;
					case USB_SET_ADDRESS:   // 5
						printf("===> set address:%d\r\n", UsbSetupBuf->wValueL);
						SetupLen = UsbSetupBuf->wValueL;							  //暂存USB设备地址
						break;
					case USB_GET_CONFIGURATION:  // 8
						printf("get configuration!\r\n");
						Ep0Buffer[0] = UsbConfig;
						if ( SetupLen >= 1 )
						{
							len = 1;
						}
						break;
					case USB_SET_CONFIGURATION: // 9
						printf("yk debug USB_SET_CONFIGURATION:%d\r\n", UsbSetupBuf->wValueL);
						UsbConfig = UsbSetupBuf->wValueL;
						break;
					case USB_GET_INTERFACE:    // 0xa
						printf("get interface!\r\n");
						break;
					case USB_SET_INTERFACE:    // 0xb
						printf("===> set interface!\r\n");
						break;
					case USB_CLEAR_FEATURE:		// 0x1									//Clear Feature
						printf("===> clean feature!\r\n");
						if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )				  /* 清除设备 */
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
							{
								if( CfgDesc[ 7 ] & 0x20 )
								{
									/* 唤醒 */
								}
								else
								{
									len = 0xFF;										/* 操作失败 */
								}
							}
							else
							{
								len = 0xFF;											/* 操作失败 */
							}
						}
						else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
						{
							switch( UsbSetupBuf->wIndexL )
							{
							case 0x83:
								UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								break;
							case 0x03:
								UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								break;
							case 0x82:
								UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								break;
							case 0x02:
								UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								break;
							case 0x81:
								// UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;// Set endpoint1 IN STALL 
								break;
							case 0x01:
								// UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;// Set endpoint1 OUT Stall
								break;
							default:
								len = 0xFF;										 // 不支持的端点
								break;
							}
						}
						else
						{
							len = 0xFF;												// 不是端点不支持
						}
						break;
					case USB_SET_FEATURE:  // 0x3										  /* Set Feature */
						printf("===> set feature!\r\n");
						if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )				  /* 设置设备 */
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
							{
								if( CfgDesc[ 7 ] & 0x20 )
								{
									/* 休眠 */
#ifdef DE_PRINTF
									printf( "----> suspend\r\n" );															 //睡眠状态
#endif
									
								}
								else
								{
									len = 0xFF;										/* 操作失败 */
								}
							}
							else
							{
								len = 0xFF;											/* 操作失败 */
							}
						}
						else if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_ENDP )			 /* 设置端点 */
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
							{
								switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
								{
								case 0x83:
									UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
									break;
								case 0x03:
									UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
									break;
								case 0x82:
									UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
									break;
								case 0x02:
									UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
									break;
								case 0x81:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
									break;
								case 0x01:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点1 OUT Stall */
								default:
									len = 0xFF;									/* 操作失败 */
									break;
								}
							}
							else
							{
								len = 0xFF;									  /* 操作失败 */
							}
						}
						else
						{
							printf("openerate fail\r\n");
							len = 0xFF;										  /* 操作失败 */
						}
						break;
					case USB_GET_STATUS:  // 0x0
						printf("===> get status!\n");
						Ep0Buffer[0] = 0x00;
						Ep0Buffer[1] = 0x00;
						if ( SetupLen >= 2 )
						{
							len = 2;
						}
						else
						{
							len = SetupLen;
						}
						break;
					default:
						len = 0xff;													//操作失败
						break;
					}
				}
			}
			else
			{
				printf("===> set 0xff!\n");
				len = 0xff;														 //包长度错误
			}
			
			printf("---->out break len=%d\r\n", len);
			if(len == 0xff)
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
			}
			else if(len <= DEFAULT_ENDP0_SIZE)													   //上传数据或者状态阶段返回0长度包
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
			}
			else
			{
				UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
			}
			break;
		case UIS_TOKEN_IN | 0:													  //endpoint0 IN
			switch(SetupReq)
			{
			case USB_GET_DESCRIPTOR:
				len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;								 //本次传输长度
				// printf("UIS_TOKEN_IN get descript:SetupLen = %d len=%d \r\n", SetupLen, len);
				memcpy( Ep0Buffer, pDescr, len );								   //加载上传数据
				// printf("%02x %02x %02x %02x   %02x %02x %02x %02x\r\n", Ep0Buffer[0], Ep0Buffer[1], Ep0Buffer[2], Ep0Buffer[3], 
				// Ep0Buffer[4], Ep0Buffer[5], Ep0Buffer[6], Ep0Buffer[7]);
				SetupLen -= len;
				pDescr += len;
				UEP0_T_LEN = len;
				if(len != 0)
					UEP0_CTRL ^= bUEP_T_TOG;											 //同步标志位翻转
				else
					UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
				break;
			case USB_SET_ADDRESS:
				USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			default:
				UEP0_T_LEN = 0;													  //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			break;
		case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
			/*if(SetupReq ==SET_LINE_CODING)  //设置串口属性
			{
				if( U_TOG_OK )
				{
				//	memcpy(LineCoding,UsbSetupBuf,USB_RX_LEN);
				//	Config_Uart1(LineCoding);
					UEP0_T_LEN = 0;
					UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // 准备上传0包
				}
			}
			else
			{*/
				UEP0_T_LEN = 0;
				UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  //状态阶段，对IN响应NAK
			//}
			break;



		default:
			break;
		}
		UIF_TRANSFER = 0;														   //写0清空中断
	}
	if(UIF_BUS_RST)																 //设备模式USB总线复位中断
	{
#ifdef DE_PRINTF
		printf( "reset\r\n" );															 //睡眠状态
#endif
		UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK|UEP_R_RES_ACK;
		UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;															 //清中断标志
		MidiByteCount = 0;	  //当前缓冲区剩余待取字节数
		USBByteCount = 0;	   //USB端点收到的长度
		UsbConfig = 0;		  //清除配置值
		UpPoint2_Busy = 0;
	}
	if (UIF_SUSPEND)																 //USB总线挂起/唤醒完成
	{
		UIF_SUSPEND = 0;
		if ( USB_MIS_ST & bUMS_SUSPEND )											 //挂起
		{
#ifdef DE_PRINTF
			printf( "suspend\r\n" );															 //睡眠状态
#endif
			while ( XBUS_AUX & bUART0_TX )
			{
				;	//等待发送完成
			}
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					  //USB或者RXD0/1有信号时可被唤醒
			PCON |= PD;																 //睡眠
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
		}
	}
	else {																			 //意外的中断,不可能发生的情况
		USB_INT_FG = 0xFF;															 //清中断标志

	}
}

//主函数
main()
{
	uint8_t length = 0;
	uint8_t Midi_Timeout = 0;
	CfgFsys( );														   //CH559时钟选择配置
	mDelaymS(5);														  //修改主频等待内部晶振稳定,必加
	mInitSTDIO( );														//串口0,可以用于调试

#ifdef DE_PRINTF
	printf("start ...\r\n");
#endif
	
	USBDeviceCfg();
	USBDeviceEndPointCfg();											   //端点配置
	USBDeviceIntCfg();													//中断初始化
	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0;													   //预使用发送长度一定要清空
	UEP2_T_LEN = 0;													   //预使用发送长度一定要清空

	while(1)
	{
		if(UsbConfig)
		{
			//printf("send usbconfig 1 USBByteCount=%d\r\n", USBByteCount);
			if(USBByteCount)   //USB接收端点有数据
			{
				printf("send usbconfig 2 and usbbytecount=%d\r\n", USBByteCount);
				//CH554UART1SendByte(Ep2Buffer[USBBufOutPoint++]);
				//USBByteCount--;
				//if(USBByteCount==0)
				memcpy(Receive_Midi_Buf, Ep2Buffer, USBByteCount);
				#ifdef DE_PRINTF
				printf("R=%d, %02x %02x %02x %02x\r\n", USBByteCount, Receive_Midi_Buf[0], Receive_Midi_Buf[1], Receive_Midi_Buf[2], Receive_Midi_Buf[3]);
				#endif
				
				// UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
				// length = USBByteCount;

				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
				USBByteCount = 0;

				mDelaymS(100);
				memcpy(Ep2Buffer + MAX_PACKET_SIZE, Receive_Midi_Buf, 9);
				// send
				UEP2_T_LEN = 9; // Pre-use send length must be cleared;
				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; // Answer ACK
				UpPoint2_Busy = 1;

			}
		}
	}
}
