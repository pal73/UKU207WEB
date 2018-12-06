#include "stdint.h"
#include "uart1.h"
#include "LPC17XX.H" 
#include "main.h"
#include "25lc640.h"
#include "eeprom_map.h"
#include "control.h"
#include "uart0.h"
#include "cmd.h"
#include "gran.h"
#include "common_func.h"
#include "avar_hndl.h"


char bRXIN1;
char UIB1[20];
char flag1;
char rx_buffer1[RX_BUFFER_SIZE1];
char tx_buffer1[TX_BUFFER_SIZE1];
unsigned short rx_wr_index1,rx_rd_index1,rx_counter1;
unsigned short tx_wr_index1,tx_rd_index1,tx_counter1;
char rx_buffer_overflow1;
char plazma_uart1;
char uart1_mess[10];
char data_rs1[40];
enum_usart1_router_stat usart1_router_stat;
char usart1_router_wrk;
char memo_out1[100];
//char suzz[4];
char UIB10[30];
char usart1_router_cnt;
char plazma_suz[5];
volatile uint32_t UART1Status;
volatile uint8_t UART1TxEmpty = 1;
//-----------------------------------------------
void putchar1(char c)
{
while (tx_counter1 == TX_BUFFER_SIZE1);
if (tx_counter1 || ((LPC_UART1->LSR & 0x60)==0))
   {
   tx_buffer1[tx_wr_index1]=c;
   if (++tx_wr_index1 == TX_BUFFER_SIZE1) tx_wr_index1=0;
   ++tx_counter1;
   }
else
	{
	LPC_PINCON->PINSEL4 &= ~0x0000c000;//!!!!!!!!!!!
	LPC_PINCON->PINSEL4 |= 0x00000000; //!!!!!!!!!!! 
	LPC_GPIO2->FIODIR|=(1UL<<2);
	LPC_GPIO2->FIOPIN|=(1UL<<2);
	LPC_UART1->THR=c;
	} 
}

//-----------------------------------------------
void uart_out1 (char num,char data0,char data1,char data2,char data3,char data4,char data5)
{                
char i,t=0;
//char *ptr=&data1;
char UOB1[16]; 
UOB1[0]=data0;
UOB1[1]=data1;
UOB1[2]=data2;
UOB1[3]=data3;
UOB1[4]=data4;
UOB1[5]=data5;

for (i=0;i<num;i++)
	{
	t^=UOB1[i];
	}    
UOB1[num]=num;
t^=UOB1[num];
UOB1[num+1]=t;
UOB1[num+2]=END;

for (i=0;i<num+3;i++)
	{
	putchar1(UOB1[i]);
	}   	
}

//-----------------------------------------------
void uart_out_adr1 (char *ptr, unsigned char len)
{
char UOB[300]/*={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}*/;
unsigned char i,t=0;

//suzz[3]++;

for(i=0;i<len;i++)
	{
	UOB[i]=ptr[i];
	t^=UOB[i];
	}

UOB[len]=len;
t^=len;	
UOB[len+1]=t;	
UOB[len+2]=END;
 
for (i=0;i<len+3;i++)
	{
	putchar1(UOB[i]);
	}   
}

//-----------------------------------------------
void uart_out__adr1 (char *ptr, unsigned char len)
{
char UOB[300]/*={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}*/;
unsigned char i,t=0;

//suzz[3]++;

for(i=0;i<len;i++)
	{
	UOB[i]=ptr[i];
	t^=UOB[i];
	}
/*
UOB[len]=len;
t^=len;	
UOB[len+1]=t;	
UOB[len+2]=END;*/
 
for (i=0;i<len;i++)
	{
	putchar1(UOB[i]);
	}   
}
//-----------------------------------------------
uint32_t uart1_init(uint32_t baudrate)
{
uint32_t Fdiv;
uint32_t pclkdiv, pclk;

LPC_PINCON->PINSEL4 &= ~0x0000000F;
LPC_PINCON->PINSEL4 |= 0x0000000A;	/* Enable RxD1 P2.1, TxD1 P2.0 */

LPC_PINCON->PINSEL4 &= ~0x00000030;//!!!!!!!!!!!
LPC_PINCON->PINSEL4 |= 0x00000000; //!!!!!!!!!!! 
LPC_GPIO2->FIODIR|=(1UL<<2);
LPC_GPIO2->FIOPIN&=~(1UL<<2);

pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
switch ( pclkdiv )
	{
	case 0x00:
	default:
     pclk = SystemFrequency/4;
	break;
	case 0x01:
	pclk = SystemFrequency;
	break; 
	case 0x02:
	pclk = SystemFrequency/2;
	break; 
	case 0x03:
	pclk = SystemFrequency/8;
	break;
	}

LPC_UART1->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
LPC_UART1->DLM = Fdiv / 256;							
LPC_UART1->DLL = Fdiv % 256;
LPC_UART1->LCR = 0x03;		/* DLAB = 0 */
LPC_UART1->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

NVIC_EnableIRQ(UART1_IRQn);

LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART1 interrupt */
return (TRUE);


}


//-----------------------------------------------
char getchar1(void)
{
char data;
while (rx_counter1==0);
data=rx_buffer1[rx_rd_index1];
if (++rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;
--rx_counter1;
return data;
}

//***********************************************
void UART1_IRQHandler (void) 
{
uint8_t IIRValue, LSRValue;
uint8_t Dummy = Dummy;
char /*status,u2iir,*/data;

			
IIRValue = LPC_UART1->IIR;
    
IIRValue >>= 1;			/* skip pending bit in IIR */
IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  	{
	LSRValue = LPC_UART1->LSR;
	/* Receive Line Status */
	
	
	
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
		{
	  	/* There are errors or break interrupt */
	  	/* Read LSR will clear the interrupt */
	  	UART1Status = LSRValue;
	  	Dummy = LPC_UART1->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  	return;
		}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
		{
		
		data=LPC_UART1->RBR;
		rx_buffer1[rx_wr_index1]=data;
   		bRXIN1=1;
   		if (++rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0;
   		if (++rx_counter1 == RX_BUFFER_SIZE1)
      		{
      		rx_counter1=0;
      		rx_buffer_overflow1=1;
      		}


		}

  	}
else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  	{
	
	//plazma_uart1++;
	data=LPC_UART1->RBR;
	rx_buffer1[rx_wr_index1]=data;
   	bRXIN1=1;

   	if (++rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0;
   	if (++rx_counter1 == RX_BUFFER_SIZE1)
      	{
      	rx_counter1=0;
      	rx_buffer_overflow1=1;
      	}
	//modbus_rx_buffer[modbus_rx_buffer_ptr]=data;
	//modbus_rx_buffer_ptr++;
//	modbus_timeout_cnt=0;

  	}
else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  	{
	/* Character Time-out indicator */
	UART1Status |= 0x100;		/* Bit 9 as the CTI error */
  	}
else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  	{
	/* THRE interrupt */
	
	LSRValue = LPC_UART1->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
		{
	  	UART1TxEmpty = 1;
		if (tx_counter1)
   			{
   			--tx_counter1;
   			LPC_UART1->THR=tx_buffer1[tx_rd_index1];
   			if (++tx_rd_index1 == TX_BUFFER_SIZE1) tx_rd_index1=0;
   			}
		else LPC_GPIO2->FIOPIN&=~(1UL<<2);
		}
	else
		{
	  	UART1TxEmpty = 0;
		}
  	}
}
 

//-----------------------------------------------
void uart_in_an1(void)
{
char i;
//motor_ind[0]^=0b0001000;
plazma_suz[0]++;

if(UIB1[1]!=0x51)
{
for(i=0;i<24;i++)
	{
	UIB10[i]=UIB1[i];
	}
}
uart1_mess[0]++;

if((UIB1[0]==4)&&(UIB1[1]==0)&&(UIB1[2]==2)&&(UIB1[3]==0)&&(UIB1[4]==1) && (ICA_EN==0))
	{
	
	uart_out1(5,4,1,2,(char)bps_I,(char)(bps_I/256),0);
	plazma_uart1++;
	}


if((UIB1[0]==6)&&(UIB1[1]==0)&&(UIB1[2]==100) && (ICA_EN==0))
	{
	short tempSSSS;
	tempSSSS=(short)UIB1[4] + ((short)UIB1[3])*256;

	//plazma_ica2=tempSSSS;

	if(tempSSSS&0x4000)
		{
		tempSSSS&=0x3fff;
		if((tempSSSS>0)&&(tempSSSS<5))tempSSSS=0;
		else if(tempSSSS>=60)tempSSSS=60;
		if(TBAT!=tempSSSS)lc640_write_int(EE_TBAT,tempSSSS);

		//main_kb_cnt=(tempSSSS*60)-20;
		}
	else ica_cntrl_hndl=tempSSSS;

	ica_cntrl_hndl_cnt=200;
	}


else if((UIB1[0]==4)&&(UIB1[1]==1)&&(UIB1[2]==2) && (ICA_EN==1) && (ICA_CH==2) )
	{
	
	ica_your_current=(short)UIB1[3]+((short)UIB1[4]*256);
	}
else if((UIB1[0]==CMND)&&(UIB1[1]==1))
	{
//	adc_buff_out_[0]=UIB1[2]+(UIB1[3]*256);
//	adc_buff_out_[1]=UIB1[4]+(UIB1[5]*256);
	}

else if((UIB1[0]==CMND)&&(UIB1[1]==2))
	{
//	adc_buff_out_[2]=UIB1[2]+(UIB1[3]*256);
//	in_stat_out[0]=UIB1[4];
//	in_stat_out[1]=UIB1[5];
	}
	
			
}
 
//-----------------------------------------------
char index_offset1 (signed char index,signed char offset)
{
index=index+offset;
if(index>=RX_BUFFER_SIZE1) index-=RX_BUFFER_SIZE1; 
if(index<0) index+=RX_BUFFER_SIZE1;
return index;
}

//-----------------------------------------------
char control_check1(char index)
{
char i=0,ii=0,iii;

if(rx_buffer1[index]!=END) goto error_cc;

ii=rx_buffer1[index_offset1(index,-2)];
iii=0;
for(i=0;i<=ii;i++)
	{
	iii^=rx_buffer1[index_offset1(index,-2-ii+i)];
	}
if (iii!=rx_buffer1[index_offset1(index,-1)]) goto error_cc;	


//success_cc:
return 1;
//goto end_cc;
error_cc:
return 0;
//goto end_cc;

//end_cc:
//__nop();
}

//-----------------------------------------------
void uart_in1(void)
{
char temp,i/*,count*/;

__disable_irq();

if(rx_buffer_overflow1)
	{
	rx_wr_index1=0;
	rx_rd_index1=0;
	rx_counter1=0;
	rx_buffer_overflow1=0;
	}    
usart1_router_wrk=1;	
if(rx_counter1&&(rx_buffer1[index_offset1(rx_wr_index1,-1)])==END)
	{
	
     temp=rx_buffer1[index_offset1(rx_wr_index1,-3)];
    	if(temp<20) 
    		{
    		if(control_check1(index_offset1(rx_wr_index1,-1)))
    			{
    		
    			rx_rd_index1=index_offset1(rx_wr_index1,-3-temp);
    			for(i=0;i<temp;i++)
				{
				UIB1[i]=rx_buffer1[index_offset1(rx_rd_index1,i)];
				} 
			rx_rd_index1=rx_wr_index1;
			rx_counter1=0;
			uart_in_an1();
			
			if(usart1_router_stat==ursMEGA)usart1_router_wrk=0;
    			}
 	
    		} 
    	}
//rx_read_power_cnt_plazma++;
#ifdef CE102M_ENABLED
	if(rx_read_power_cnt_phase==1)
		{
		
		if((rx_buffer1[rx_wr_index1-1]==0x0a)/*&&(rx_buffer1[6]==0xc5)*/)
			{
			rx_read_power_cnt_plazma++;
			rx_read_power_cnt_phase=2;
			ce102m_delayCnt=200;
			}
		}
else if(rx_read_power_cnt_phase==3)
		{
		
		if(/*(rx_buffer1[6]==0x81)&&*/(rx_buffer1[rx_wr_index1-2]==0x03))
			{
			rx_read_power_cnt_plazma++;
			if(bENERGOMETR_UIP==0) rx_read_power_cnt_phase=4;
			else if(bENERGOMETR_UIP==1) rx_read_power_cnt_phase=8;
			else rx_read_power_cnt_phase=20;
			ce102m_delayCnt=200;
			}
		}
else if((rx_read_power_cnt_phase==5)&&(rx_wr_index1>10))
		{
		
		if(((rx_buffer1[rx_wr_index1-1])&0x7f)=='(')
			{
			rx_read_power_cnt_plazma++;
			rx_read_power_cnt_phase=6;
			ce102m_delayCnt=200;
			rx_wr_index1=0;
			}
		}
else if(rx_read_power_cnt_phase==6)
		{
		char float_buff[20]={0,0,0,0,0,0,0,0,0,0};
		char* float_buff_ptr;
		float volta;
		char i,point_marker;
		unsigned short ii;		
		float_buff_ptr=float_buff;

		if(((rx_buffer1[rx_wr_index1-1])&0x7f)==')')
			{
			ii=rx_wr_index1-1;
			rx_read_power_cnt_plazma=rx_wr_index1-1;
			
			volta_short=0;//rx_wr_index1-1;
			for(i=0;i<ii;i++)
				{
				float_buff[i]=rx_buffer1[i]&0x7f;
				float_buff[i+1]=' ';
 //volta_short+=(rx_buffer1[ii-i]&0x7f);//-0x30)*pow(10,i);
 				if(float_buff[i]=='.')point_marker=i;
				rx_read_power_cnt_plazma++;
				}


		///if(((rx_buffer1[rx_wr_index1-2])&0x7f)=='.')
		///	{
		//	char s [] = "1.23";
		///	ii=rx_wr_index1-1;
		///	rx_read_power_cnt_plazma++;
			rx_read_power_cnt_phase=15;
			ce102m_delayCnt=200;
			//memcpy(float_buff,rx_buffer1,rx_wr_index1-1);
		///	volta_short=0;//rx_wr_index1-1;
		///	for(i=0;i<ii;i++)
				{
		///		float_buff[i]=rx_buffer1[i]&0x7f;
		///		float_buff[i+1]=' ';
				/*if(rx_buffer1[ii-i]=='(')break;
				else*/ //volta_short+=(rx_buffer1[ii-i]&0x7f);//-0x30)*pow(10,i);
				}
			//float_buff[0]='2';
			//float_buff[1]='3';
			//float_buff[2]='4';
			//float_buff[3]='.';
			//float_buff[4]='5';
			//uart_out__adr1(float_buff, 10);
			//volta=atof(s);//atof(float_buff);
			volta_short=((atoi(float_buff))*10)+ ((atoi(&float_buff[point_marker+1]))/10);
			}
		}
else if(rx_read_power_cnt_phase==7)
		{


		if(((rx_buffer1[rx_wr_index1-1])&0x7f)==')')
			{
			rx_read_power_cnt_phase=18;
			rx_wr_index1=0;	
			ce102m_delayCnt=200;
			}
		}
else if((rx_read_power_cnt_phase==9)&&(rx_wr_index1>15))
		{
		
		if(((rx_buffer1[rx_wr_index1-1])&0x7f)=='(')
			{
			rx_read_power_cnt_plazma++;
			rx_read_power_cnt_phase=10;
			
			rx_wr_index1=0;
			}
		}
else if((rx_read_power_cnt_phase==10)/*&&(rx_wr_index1>2)*/)
		{
		char float_buff[20]={0,0,0,0,0,0,0,0,0,0};
		char* float_buff_ptr;
		float curr;
		char i,point_marker;
		unsigned short ii;
		int curr_1,curr_2;		
		float_buff_ptr=float_buff;


		if(((rx_buffer1[rx_wr_index1-1])&0x7f)==')')
			{
		//	char s [] = "1.23";
			ii=rx_wr_index1-1;
			rx_read_power_cnt_plazma=rx_wr_index1-1;
			
			//ce102m_delayCnt=500;
			//memcpy(float_buff,rx_buffer1,rx_wr_index1-1);
			curr_short=0;//rx_wr_index1-1;
			for(i=0;i<ii;i++)
				{
				float_buff[i]=rx_buffer1[i]&0x7f;
				float_buff[i+1]=' ';
 //volta_short+=(rx_buffer1[ii-i]&0x7f);//-0x30)*pow(10,i);
 				if(float_buff[i]=='.')point_marker=i;
				rx_read_power_cnt_plazma++;
				}
			curr_short=ii;
			/*float_buff[0]='2';
			float_buff[1]='.';
			float_buff[2]='4';
			float_buff[3]='3';
			float_buff[4]='5';*/
			curr_1=atoi(float_buff);
			curr_2=atoi(&float_buff[point_marker+1]);
			curr_short=(curr_2/10)+(curr_1*100);
			//curr=0.0;
			//uart_out__adr1(float_buff, 6);
			//curr=atof(float_buff);
			//curr=0.354;
			//uart_out__adr1((char*)&curr, 10);
			//curr*=1000.0;
			//(int)(curr);//volta;
			rx_read_power_cnt_phase=17;
			}
		}

else if((rx_read_power_cnt_phase==21)&&(rx_wr_index1>10))
		{
		
		if(((rx_buffer1[rx_wr_index1-1])&0x7f)=='(')
			{
			rx_read_power_cnt_plazma++;
			rx_read_power_cnt_phase=22;
			
			//rx_read_power_cnt_plazma=rx_wr_index1;
			rx_wr_index1=0;

			}
		}
else if((rx_read_power_cnt_phase==22)&&(rx_wr_index1>2))
		{
		char float_buff[20]={0,0,0,0,0,0,0,0,0,0};
		char* float_buff_ptr;
		float power;
		char i,point_marker;
		unsigned short ii;
		int power_1,power_2;		
		float_buff_ptr=float_buff;


		if(((rx_buffer1[rx_wr_index1-1])&0x7f)==')')
			{
		//	char s [] = "1.23";
			ii=rx_wr_index1-1;
			rx_read_power_cnt_plazma=rx_wr_index1-1;
			
			//ce102m_delayCnt=500;
			//memcpy(float_buff,rx_buffer1,rx_wr_index1-1);
			power_int=0;//rx_wr_index1-1;
			for(i=0;i<ii;i++)
				{
				float_buff[i]=rx_buffer1[i]&0x7f;
				float_buff[i+1]=' ';
 //volta_short+=(rx_buffer1[ii-i]&0x7f);//-0x30)*pow(10,i);
 				if(float_buff[i]=='.')
					{
					point_marker=i;
					//float_buff[i-1]='5';
					}
				rx_read_power_cnt_plazma++;
				//if((point_marker!=0)&&(i==(point_marker+3)))break;
				}
			power_int=ii;
			/*float_buff[0]='2';
			float_buff[1]='.';
			float_buff[2]='4';
			float_buff[3]='3';
			float_buff[4]='5';*/
			power_1=atoi(float_buff);
			power_2=(atoi(&float_buff[point_marker+1]))/1000;
			power_int=power_2+(power_1*1000);
			//power_int=power_2;
			//curr=0.0;
			//float_buff[0]=rx_wr_index1;
			//uart_out__adr1(float_buff, 5);
			//curr=atof(float_buff);
			//curr=0.354;
			//uart_out__adr1((char*)&curr, 10);
			//curr*=1000.0;
			//(int)(curr);//volta;
			rx_read_power_cnt_phase=16;
			}
		}

#endif	


__enable_irq();     
}   	

