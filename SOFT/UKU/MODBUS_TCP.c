
#include <lpc17xx.h>
#include <rtl.h>
#include "main.h"
#include "modbus_tcp.h"
#include "eeprom_map.h"
#include "25lc640.h"

char plazma_modbus_tcp[20];
char modbus_tcp_plazma[20];

char modbus_tcp_func;
char modbus_tcp_unit;
short modbus_tcp_rx_arg0;
short modbus_tcp_rx_arg1;

//#define MODBUS_TCP_PROT	1

char* modbus_tcp_out_ptr;

/*--------------------------- TCP socket ------------------------------------*/

U16 tcp_callback (U8 soc, U8 evt, U8 *ptr, U16 par) 
{
/* This function is called by the TCP module on TCP event */
/* Check the 'Net_Config.h' for possible events.          */
par = par;

if (soc != socket_tcp) {
	return (0);
}

modbus_tcp_plazma[0]++;

switch (evt) 
	{
    case TCP_EVT_DATA:
    /* TCP data frame has arrived, data is located at *par1, */
    /* data length is par2. Allocate buffer to send reply.   */
    //procrec(ptr);
	
	plazma_modbus_tcp[1]=ptr[0];
	plazma_modbus_tcp[2]=ptr[1];
	plazma_modbus_tcp[3]=ptr[2];
	plazma_modbus_tcp[4]=ptr[3];
	plazma_modbus_tcp[5]=ptr[4];
	plazma_modbus_tcp[6]=ptr[5];
	plazma_modbus_tcp[7]=ptr[6];
	plazma_modbus_tcp[8]=ptr[7];
	plazma_modbus_tcp[9]=ptr[8];
	plazma_modbus_tcp[10]=ptr[9];
	plazma_modbus_tcp[11]=ptr[10];
	  //plazma_modbus_tcp[4]=ptr[3];
	  //plazma_modbus_tcp[5]=par;
	  //plazma_modbus_tcp[6]=ptr[5];

   	
	modbus_tcp_func=ptr[7];
	modbus_tcp_unit=ptr[6];

	modbus_tcp_rx_arg0=(((unsigned short)ptr[8])*((unsigned short)256))+((unsigned short)ptr[9]);
	modbus_tcp_rx_arg1=(((unsigned short)ptr[10])*((unsigned short)256))+((unsigned short)ptr[11]);

	if(modbus_tcp_unit==MODBUS_ADRESS)
		{
		char modbus_tcp_tx_buff[200];

		if(modbus_tcp_func==3)		//чтение произвольного кол-ва регистров хранения
			{
			U8 *sendbuf;
			

			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_tcp_func,modbus_tcp_rx_arg0,modbus_tcp_rx_arg1,MODBUS_TCP_PROT);

			sendbuf = tcp_get_buf((modbus_tcp_rx_arg1*2)+9);
			sendbuf[0]=ptr[0];
			sendbuf[1]=ptr[1];
			sendbuf[2]=ptr[2];
			sendbuf[3]=ptr[3];
			sendbuf[4]=((modbus_tcp_rx_arg1*2)+3)/256;
			sendbuf[5]=((modbus_tcp_rx_arg1*2)+3)%256;;
			sendbuf[6]=modbus_tcp_unit;
			sendbuf[7]=modbus_tcp_func;
			sendbuf[8]=(U8)(modbus_tcp_rx_arg1*2);
			mem_copy((char*)&sendbuf[9],modbus_tcp_out_ptr,(modbus_tcp_rx_arg1*2));
			//sendbuf[9]=3;
			//sendbuf[10]=4;
          	tcp_send (socket_tcp, sendbuf, ((modbus_tcp_rx_arg1*2)+9));
			//
			
			//modbus_tcp_tx_buff[4]=0;
			//modbus_tcp_tx_buff[5]=5;
			//modbus_tcp_tx_buff[6]=1;//modbus_tcp_unit;
			//modbus_tcp_tx_buff[7]=3;//modbus_tcp_func;
			//modbus_tcp_tx_buff[8]=2;
			//mem_copy((char*)&modbus_tcp_tx_buff[9],modbus_tcp_out_ptr,2);
			//modbus_tcp_tx_buff[9]=2;
			//modbus_tcp_tx_buff[10]=3;
			//tcp_send (socket_tcp, modbus_tcp_tx_buff, 11);


			}

		if(modbus_tcp_func==4)		//чтение произвольного кол-ва регистров	входов
			{
			U8 *sendbuf;

			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_tcp_func,modbus_tcp_rx_arg0,modbus_tcp_rx_arg1,MODBUS_TCP_PROT);

			sendbuf = tcp_get_buf((modbus_tcp_rx_arg1*2)+9);
			sendbuf[0]=ptr[0];
			sendbuf[1]=ptr[1];
			sendbuf[2]=ptr[2];
			sendbuf[3]=ptr[3];
			sendbuf[4]=((modbus_tcp_rx_arg1*2)+3)/256;
			sendbuf[5]=((modbus_tcp_rx_arg1*2)+3)%256;;
			sendbuf[6]=modbus_tcp_unit;
			sendbuf[7]=modbus_tcp_func;
			sendbuf[8]=(U8)(modbus_tcp_rx_arg1*2);
			mem_copy((char*)&sendbuf[9],modbus_tcp_out_ptr,(modbus_tcp_rx_arg1*2));
			//sendbuf[9]=3;
			//sendbuf[10]=4;
          	tcp_send (socket_tcp, sendbuf, ((modbus_tcp_rx_arg1*2)+9));
			//
			
			//modbus_tcp_tx_buff[4]=0;
			//modbus_tcp_tx_buff[5]=5;
			//modbus_tcp_tx_buff[6]=1;//modbus_tcp_unit;
			//modbus_tcp_tx_buff[7]=3;//modbus_tcp_func;
			//modbus_tcp_tx_buff[8]=2;
			//mem_copy((char*)&modbus_tcp_tx_buff[9],modbus_tcp_out_ptr,2);
			//modbus_tcp_tx_buff[9]=2;
			//modbus_tcp_tx_buff[10]=3;
			//tcp_send (socket_tcp, modbus_tcp_tx_buff, 11);
			}

		else if(modbus_tcp_func==6) 	//запись регистров хранения
			{
			U8 *sendbuf;

			
			if(modbus_tcp_rx_arg0==11)		//Установка времени 
				{
				LPC_RTC->YEAR=(uint16_t)modbus_tcp_rx_arg1;
				}
			if(modbus_tcp_rx_arg0==12)		//Установка времени 
				{
				LPC_RTC->MONTH=(uint16_t)modbus_tcp_rx_arg1;
				}
			if(modbus_tcp_rx_arg0==13)		//Установка времени 
				{
				LPC_RTC->DOM=(uint16_t)modbus_tcp_rx_arg1;
				}
			if(modbus_tcp_rx_arg0==14)		//Установка времени 
				{
				plazma_modbus_tcp[0]++;
				LPC_RTC->HOUR=(uint16_t)modbus_tcp_rx_arg1;
				}
			if(modbus_tcp_rx_arg0==15)		//Установка времени 
				{
				LPC_RTC->MIN=(uint16_t)modbus_tcp_rx_arg1;
				}
				if(modbus_tcp_rx_arg0==16)		//Установка времени 
					{
					LPC_RTC->SEC=(uint16_t)modbus_tcp_rx_arg1;
					}
				if(modbus_tcp_rx_arg0==20)		//ток стабилизации для режима стабилизации тока
					{
					if((modbus_tcp_rx_arg1>0)&&(modbus_tcp_rx_arg1<=18))
					lc640_write_int(EE_NUMIST,modbus_tcp_rx_arg1);  
					}
				if(modbus_tcp_rx_arg0==21)		//ток стабилизации для режима стабилизации тока
					{
					if((modbus_tcp_rx_arg1==0)||(modbus_tcp_rx_arg1==1))
					lc640_write_int(EE_PAR,modbus_tcp_rx_arg1);  
					}
				if(modbus_tcp_rx_arg0==22)		//ток стабилизации для режима стабилизации тока
					{
					if((modbus_tcp_rx_arg1==0)||(modbus_tcp_rx_arg1==1))
					lc640_write_int(EE_ZV_ON,modbus_tcp_rx_arg1);  
					}
				if(modbus_tcp_rx_arg0==23)		//ток стабилизации для режима стабилизации тока
					{
					if((modbus_tcp_rx_arg1==0)||(modbus_tcp_rx_arg1==1))
					lc640_write_int(EE_TERMOKOMP,modbus_tcp_rx_arg1);  
					}
				if(modbus_tcp_rx_arg0==24)		//ток стабилизации для режима стабилизации тока
					{
					if((modbus_tcp_rx_arg1>=0)||(modbus_tcp_rx_arg1<=20))
					lc640_write_int(EE_UBM_AV,modbus_tcp_rx_arg1);  
					}
	
	
				if(modbus_tcp_rx_arg0==30)		//напряжение стабилизации для режима стабилизации напряжения
					{
					if(modbus_tcp_rx_arg1<0)TBAT=0;
					else if((modbus_tcp_rx_arg1>0)&&(modbus_tcp_rx_arg1<=5))modbus_tcp_rx_arg1=0;
					else if(modbus_tcp_rx_arg1>=60)TBAT=60;
					else TBAT=modbus_tcp_rx_arg1;
					lc640_write_int(EE_TBAT,TBAT);
		     		}
				if(modbus_tcp_rx_arg0==31)		//
					{
					lc640_write_int(EE_UMAX,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==32)		//
					{
					lc640_write_int(EE_DU,UB20-modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==33)		//
					{
					lc640_write_int(EE_UB0,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==34)		//
					{
					lc640_write_int(EE_UB20,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==35)		//
					{
					lc640_write_int(EE_USIGN,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==36)		//
					{
					lc640_write_int(EE_UMN,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==37)		//
					{
					lc640_write_int(EE_U0B,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==38)		//
					{
					lc640_write_int(EE_IKB,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==39)		//
					{
					lc640_write_int(EE_IZMAX,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==40)		//
					{
					lc640_write_int(EE_IMAX,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==41)		//
					{
					lc640_write_int(EE_IMIN,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==42)		//
					{
					lc640_write_int(EE_UVZ,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==43)		//
					{
					lc640_write_int(EE_TZAS,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==44)		//
					{
					lc640_write_int(EE_TMAX,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==45)		//
					{
					lc640_write_int(EE_TSIGN,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==46)		//
					{
					lc640_write_int(EE_TBATMAX,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==47)		//
					{
					lc640_write_int(EE_TBATSIGN,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==48)		//
					{
					lc640_write_int(EE_SPEED_CHRG_CURR,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==49)		//
					{
					lc640_write_int(EE_SPEED_CHRG_VOLT,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==50)		//
					{
					lc640_write_int(EE_SPEED_CHRG_TIME,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==51)		//
					{
					lc640_write_int(EE_U_OUT_KONTR_MAX,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==52)		//
					{
					lc640_write_int(EE_U_OUT_KONTR_MIN,modbus_tcp_rx_arg1);
		     		}
				if(modbus_tcp_rx_arg0==53)		//
					{
					lc640_write_int(EE_U_OUT_KONTR_DELAY,modbus_tcp_rx_arg1);
		     		}
	
				if(modbus_tcp_rx_arg0==19)		//вкл/выкл источника напр.
					{
		/*			if(modbus_tcp_rx_arg1==1)
						{
						if(work_stat!=wsPS)
							{
							work_stat=wsPS;
							time_proc=0;
							time_proc_remain=T_PROC_PS;
							restart_on_PS();
							lc640_write_int(EE_MAIN_MENU_MODE,mmmIN);
							}
						}
					if(modbus_tcp_rx_arg1==0)
						{
						if(work_stat==wsPS)
							{
							work_stat=wsOFF;
							restart_off();
							}
						} */
					}
				if(modbus_tcp_rx_arg0==20)		//вкл/выкл источника тока
					{
	/*				if(modbus_tcp_rx_arg1==1)
						{
						if(work_stat!=wsGS)
							{
							work_stat=wsGS;
							time_proc=0;
							time_proc_remain=T_PROC_GS;
							lc640_write_int(EE_MAIN_MENU_MODE,mmmIT);
							}
						}
					if(modbus_tcp_rx_arg1==0)
						{
						if(work_stat==wsGS)
							{
							work_stat=wsOFF;
							restart_off();
							}
						}*/
					}
				modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_tcp_func,modbus_tcp_rx_arg0,1,MODBUS_TCP_PROT);
	
				sendbuf = tcp_get_buf(11);
				sendbuf[0]=ptr[0];
				sendbuf[1]=ptr[1];
				sendbuf[2]=ptr[2];
				sendbuf[3]=ptr[3];
				sendbuf[4]=0;
				sendbuf[5]=6;
				sendbuf[6]=modbus_tcp_unit;
				sendbuf[7]=modbus_tcp_func;
				sendbuf[8]=modbus_tcp_rx_arg0/256;
				sendbuf[9]=modbus_tcp_rx_arg0%256;
				mem_copy((char*)&sendbuf[10],modbus_tcp_out_ptr,2);
				//sendbuf[9]=3;
				//sendbuf[10]=4;
	          	tcp_send (socket_tcp, sendbuf, 12);
				//
				
				//modbus_tcp_tx_buff[4]=0;
				//modbus_tcp_tx_buff[5]=5;
				//modbus_tcp_tx_buff[6]=1;//modbus_tcp_unit;
				//modbus_tcp_tx_buff[7]=3;//modbus_tcp_func;
				//modbus_tcp_tx_buff[8]=2;
				//mem_copy((char*)&modbus_tcp_tx_buff[9],modbus_tcp_out_ptr,2);
				//modbus_tcp_tx_buff[9]=2;
				//modbus_tcp_tx_buff[10]=3;
				//tcp_send (socket_tcp, modbus_tcp_tx_buff, 11);






				}


			
			} 
      	break;

    	case TCP_EVT_CONREQ:
      		/* Remote peer requested connect, accept it */
			//ica_plazma[5]++;
      		return (1);

    	case TCP_EVT_CONNECT:
      		/* The TCP socket is connected */
			tcp_connect_stat=1;
			//ica_plazma[6]++;
      		return (1);

    	case TCP_EVT_CLOSE: 
      		/* The TCP socket is connected */
			tcp_connect_stat=0;
			//ica_plazma[7]++;
      		return (1);
  		}
  	return (0);
}