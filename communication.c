/*******************************************************************************
文件名:Communication.c
版本号:V1.0
版权说明:
作者:吕明君   
编写日期:2011.08.31
简要描述:
修改人:
修改日期:
修改记录:
*******************************************************************************/
/*******************************************************************************
//文件包含
*******************************************************************************/
#include "communication.h"
#include "usart.h"
#ifdef PHASED_ARRAYS_MODE
#include "XKZ_business.h"
#endif
#ifdef _ANTENNA_FW_
#include "modem.h"
#endif
/*******************************************************************************
//声明
*******************************************************************************/


/*******************************************************************************
//变量定义
*******************************************************************************/
rev_UART_stru   g_rev_UART1 = {0, 0, FALSE, 0, {0},UART_1};            /*串口接收*/
rev_UART_stru   g_rev_UART3 = {0, 0, FALSE, 0, {0},UART_3};
rev_UART_stru   g_rev_UART4 = {0, 0, FALSE, 0, {0},UART_4};

rev_UART_stru   g_rev_NET1 = {0, 0, FALSE, 0, {0},TCP_PORT1};           /* 网口接收 */
rev_UART_stru   g_rev_NET2 = {0, 0, FALSE, 0, {0},TCP_PORT2};       


/*******************************************************************************
//函数定义
*******************************************************************************/

/******************************************
*函数名称：u8 antenna_mcu_send
*函数功能：天线盒通过串口1发送数据到控制盒
*入口参数：atten_case_ID = CASE_1; CASE_1对应着串口1
           p_send_data->数据地址；数据长度->len
           RSTCL         帧序列号；    应用层控制
           frame_type    帧格式        取值    FT1 格式1 ，FT2 格式2
*出口参数：无          
*返 回 值： TRUE:  正常;  FALSE: 发送数据过长
*全局变量：无
*调用函数：无
******************************************/

u8 antenna_mcu_send(uc8 *p_send_data, u16 len, u8 RSCTL, u8 frame_type, u16 atten_case_ID)
{

    u16 i = 0;
    u16 send_len = 0;
    u8  BCC  = 0;
    u8  uart_send_buf[UART_SEND_BUF_LEN];        

    if(len >= DATA_LEN_MAX)                   //考虑转义，若发送数据过长，返回FALSE
    {   
        return FALSE;
    }
    if( FT1 == frame_type )                     //按照帧格式一来发
    {
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = RSCTL;
        BCC = RSCTL;
        while( i < len )
        {
            BCC ^= p_send_data[i];
            switch( p_send_data[i] )            //转义
            {
                case 0xFF:
                    uart_send_buf[send_len++] = 0xFE;
                    uart_send_buf[send_len++] = 0x01;
                break;

                case 0xFE:
                    uart_send_buf[send_len++] = 0xFE;
                    uart_send_buf[send_len++] = 0x00;
                break;

                default:
                    uart_send_buf[send_len++] = p_send_data[i];
                break;

            }
            
            i++;
            
        }

        switch(BCC)                                 //BCC是否转义
        {
            case 0xFF:
                uart_send_buf[send_len++] = 0xFE;
                uart_send_buf[send_len++] = 0x01;
            
            break;

            case 0xFE:
                uart_send_buf[send_len++] = 0xFE;
                uart_send_buf[send_len++] = 0x00;
            
            break;

            default:
                uart_send_buf[send_len++] = BCC;
                
            break;
        }
        
        uart_send_buf[send_len++] = 0xFF;   
        
    }
    else                                            //帧格式二
    {
        uart_send_buf[send_len++] = 0x55;
        uart_send_buf[send_len++] = 0xAA;
        uart_send_buf[send_len++] = RSCTL;
        uart_send_buf[send_len] = (len >> 8) & 0xFF;
        BCC = RSCTL ^ uart_send_buf[send_len++];
        uart_send_buf[send_len] = len & 0xFF;
        BCC  ^= uart_send_buf[send_len++];

        while(i < len)
        {
            BCC ^= p_send_data[i];
            uart_send_buf[send_len++] = p_send_data[i];
            i++;
        }  
        uart_send_buf[send_len++] = BCC;        
    }
      
    switch(atten_case_ID)                            //组帧完毕，选择发送模式   
    {
      case UART_1:
          sys_statis.communicate.uart1_send_frame++;
#ifdef _ANTENNA_FW_
          if(FALSE == g_gprs_enter_exchange_flag)
          {
              send_len = hex_string_normal(uart_send_buf,uart_send_buf,send_len);
              /*用SMS发送命令*/
              if(TRUE ==  AT_CMGS(comming_call,uart_send_buf,send_len))
              {
                  sys_statis.communicate.uart1_send_success++;
                  return TRUE;
              }
              else
              {
                  sys_statis.communicate.uart1_send_fail++;
                  return FALSE;
              }
          }
          else
#endif
          {
              if(TRUE == usart1_write_wait(uart_send_buf, send_len))
              {
                  sys_statis.communicate.uart1_send_success++;
                  return TRUE;
              }
              else
              {
                  sys_statis.communicate.uart1_send_fail++;
                  return FALSE;
              }
          }

      case UART_3:
          sys_statis.communicate.uart3_send_frame++;
          if(TRUE == usart3_write_wait(uart_send_buf, send_len))
          {
              sys_statis.communicate.uart3_send_success++;
              return TRUE;
          }
          else
          {
              sys_statis.communicate.uart3_send_fail++;
              return FALSE;
          }

      case UART_4:
          sys_statis.communicate.uart4_send_frame++;
		  #ifdef _ANTENNA_FW_
		  #ifdef PHASED_ARRAYS_MODE
		  MsSendCtrlEnable();
		  #endif
		  #endif
          if(TRUE == uart4_write_wait(uart_send_buf, send_len))
          {
          	  #ifdef _ANTENNA_FW_
			  #ifdef PHASED_ARRAYS_MODE
			  MsSendCtrlDisable();
			  #endif
			  #endif
              sys_statis.communicate.uart4_send_success++;
              return TRUE;
          }
          else
          {      
          	  #ifdef _ANTENNA_FW_
			  #ifdef PHASED_ARRAYS_MODE
			  MsSendCtrlDisable();
			  #endif
			  #endif
              sys_statis.communicate.uart4_send_fail++;                
              return FALSE;
          }

      case TCP_PORT1:
          sys_statis.tcp_ip.tcp_port1_send_frame++;  
          if(TRUE == tcp_ip_write(uart_send_buf, send_len, TCP_PORT1))
          {
              sys_statis.tcp_ip.tcp_port1_send_success++;  
              return TRUE;
          }
          else
          {
              sys_statis.tcp_ip.tcp_port1_send_fail++;  
              return FALSE;
          }

      case TCP_PORT2:
          sys_statis.tcp_ip.tcp_port2_send_frame++;  
          if(TRUE == tcp_ip_write(uart_send_buf, send_len, TCP_PORT2))
          {
              sys_statis.tcp_ip.tcp_port2_send_success++;  
              return TRUE;
          }
          else
          {
              sys_statis.tcp_ip.tcp_port2_send_fail++;  
              return FALSE;
          }

        default:
            
            sys_statis.communicate.send_frame_err_channal++;  
            return FALSE;

    }


}


#ifdef _ANTENNA_FW_
u8  modem_send_buf[UART_SEND_BUF_LEN];
#endif
u8 ctrl_mcu_send(uc8 *p_send_data, u16 len, u8 RSCTL, u8 frame_type, u16 atten_case_ID)
{

    u16 i = 0;
    u16 send_len = 0;
    u8  BCC  = 0;
    u8  uart_send_buf[UART_SEND_BUF_LEN];        

    if(len >= DATA_LEN_MAX)                   //考虑转义，若发送数据过长，返回FALSE
    {   
        return FALSE;
    }
    if( FT1 == frame_type )                     //按照帧格式一来发
    {
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = RSCTL;
        BCC = RSCTL;
        while( i < len )
        {
            BCC ^= p_send_data[i];
            switch( p_send_data[i] )            //转义
            {
                case 0xFF:
                {
                    uart_send_buf[send_len++] = 0xFE;
                    uart_send_buf[send_len++] = 0x01;
                    break;
                }
                case 0xFE:
                {
                    uart_send_buf[send_len++] = 0xFE;
                    uart_send_buf[send_len++] = 0x00;
                    break;
                }
                default:
                    uart_send_buf[send_len++] = p_send_data[i];
                    break;
            }            
            i++;           
        }

        switch(BCC)                                 //BCC是否转义
        {
            case 0xFF:
            {
                uart_send_buf[send_len++] = 0xFE;
                uart_send_buf[send_len++] = 0x01;
                break;
            }
            case 0xFE:
            {
                uart_send_buf[send_len++] = 0xFE;
                uart_send_buf[send_len++] = 0x00;
                break;
            }
            default:
                uart_send_buf[send_len++] = BCC;
                break;
        }
        uart_send_buf[send_len++] = 0xFF;           
    }
    else                                            //帧格式二
    {
        uart_send_buf[send_len++] = 0x55;
        uart_send_buf[send_len++] = 0xAA;
        uart_send_buf[send_len++] = RSCTL;
        uart_send_buf[send_len] = (len >> 8) & 0xFF;
        BCC = RSCTL ^ uart_send_buf[send_len++];
        uart_send_buf[send_len] = len & 0xFF;
        BCC  ^= uart_send_buf[send_len++];

        while(i < len)
        {
            BCC ^= p_send_data[i];
            uart_send_buf[send_len++] = p_send_data[i];
            i++;
        }  
        uart_send_buf[send_len++] = BCC;        
    }
      
    switch(atten_case_ID)                            //组帧完毕，选择发送模式   
    {
        case UART_1:
            sys_statis.communicate.uart1_send_frame++;
#ifdef _ANTENNA_FW_
            if(FALSE == g_gprs_enter_exchange_flag)
            {
				send_len = hex_string_normal(uart_send_buf,modem_send_buf,send_len);
				/*用SMS发送命令*/
				if(TRUE ==  AT_CMGS(comming_call,modem_send_buf,send_len))
				{
					sys_statis.communicate.uart1_send_success++;
					return TRUE;
				}
				else
				{
					sys_statis.communicate.uart1_send_fail++;
					return FALSE;
				}
			}
            else
#endif
            {
                if(TRUE == usart1_write_wait(uart_send_buf, send_len))
                {
                    sys_statis.communicate.uart1_send_success++;
                    return TRUE;
                }
                else
                {
                    sys_statis.communicate.uart1_send_fail++;
                    return FALSE;
                }
            }
        
        case UART_3:
            sys_statis.communicate.uart3_send_frame++;
            if(TRUE == usart3_write_wait(uart_send_buf, send_len))
            {
                sys_statis.communicate.uart3_send_success++;
                return TRUE;
            }
            else
            {
                sys_statis.communicate.uart3_send_fail++;
                return FALSE;
            }

        case UART_4:
            sys_statis.communicate.uart4_send_frame++;
			#ifdef _ANTENNA_FW_
			#ifdef	PHASED_ARRAYS_MODE
		  	MsSendCtrlEnable();
			#endif
		  	#endif
            if(TRUE == uart4_write_wait(uart_send_buf, send_len))
            {
            	#ifdef _ANTENNA_FW_
				#ifdef PHASED_ARRAYS_MODE
		  		MsSendCtrlDisable();
				#endif
		  		#endif
                sys_statis.communicate.uart4_send_success++;
                return TRUE;
            }
            else
            {   
            	#ifdef _ANTENNA_FW_
				#ifdef PHASED_ARRAYS_MODE
		 		MsSendCtrlDisable();
				#endif
		  		#endif
                sys_statis.communicate.uart4_send_fail++;                
                return FALSE;
            }

        case TCP_PORT1:
            sys_statis.tcp_ip.tcp_port1_send_frame++;  
            if(TRUE == tcp_ip_write(uart_send_buf, send_len, TCP_PORT1))
            {
                sys_statis.tcp_ip.tcp_port1_send_success++;  
                return TRUE;
            }
            else
            {
                sys_statis.tcp_ip.tcp_port1_send_fail++;  
                return FALSE;
            }

        case TCP_PORT2:
            sys_statis.tcp_ip.tcp_port2_send_frame++;  
            if(TRUE == tcp_ip_write(uart_send_buf, send_len, TCP_PORT2))
            {
                sys_statis.tcp_ip.tcp_port2_send_success++;  
                return TRUE;
            }
            else
            {
                sys_statis.tcp_ip.tcp_port2_send_fail++;  
                return FALSE;
            }

        default:
            sys_statis.communicate.send_frame_err_channal++;  
        return FALSE;

    }

}


/*****************************************************************
*
*    ------------------------- END -----------------------
*
******************************************************************/






u8 tcp_get_data(uc8 *p_paked_data, u16 len, u16 NET_PORT)
	{
		u8	get_head_flag = FALSE;
		u8	pre_byte = 0;
		u8	now_byte = 0;
		u8	BCC = 0;
		u8	data_section_status = REV_DATE;
		u16 write_offset = 0;
		u16 i;
		u8	first_data_after_head = FALSE;
		rev_UART_stru *p_NET_rve_stru;
	
		
	
		if(TCP_PORT1 == NET_PORT)
		{
			p_NET_rve_stru =  &g_rev_NET1;
		}
		else if(TCP_PORT2 == NET_PORT)
		{
			p_NET_rve_stru =  &g_rev_NET2;
		}
		//else if(TCP_PORT3 == NET_PORT)
		//{
		//	p_NET_rve_stru =  &g_rev_NET3;
		//}
		else
		{
			sys_statis.tcp_ip.rev_frame_err_channal++;
			return FALSE;
		}
	
	
		
		for(i = 0; i < len; i++)
		{
	
			if((write_offset >= UART_REV_BUF_LEN) || (write_offset >= len)) 			   /*接收数据超出范围则复位丢掉该帧 */
			{
				break;
			}
	
			pre_byte = now_byte;
			now_byte = *p_paked_data;		
			p_paked_data ++;
	
			//兼容帧头为两个0XFF的情况，对于帧头后紧跟一个0XFF，丢弃
			if(TRUE == first_data_after_head)
			{
				first_data_after_head = FALSE;
				if(0xFF == now_byte)
				{
					continue;
				}
			}		
	
			/* 没有收到帧头  */
			if(FALSE == get_head_flag)
			{
	
				if(0xFF == now_byte)
				{
					p_NET_rve_stru->frame_type = FT1;
					get_head_flag = TRUE;
					data_section_status = REV_RSCTL;
					first_data_after_head = TRUE;
				}
	
				else if(0x55 == pre_byte)
				{
					if(0xAA == now_byte)
					{
						p_NET_rve_stru->frame_type = FT2;
						get_head_flag = TRUE;
						data_section_status = REV_RSCTL;
					}
				}
				continue;
			}
	
			/* 收到了帧头	 frame_type == FT2	 */
			else if(FT2 == p_NET_rve_stru->frame_type)
			{
				switch(data_section_status) 				/*各个字段分别存储	 */
				{
					case REV_DATE:
						break;
	
					case REV_RSCTL:
						BCC ^= now_byte;
						p_NET_rve_stru->RSCTL = now_byte;
						data_section_status = REV_LEN_HIGH;
						continue;
	
					case REV_LEN_HIGH:
						BCC ^= now_byte;
						p_NET_rve_stru->len = now_byte;
						data_section_status = REV_LEN_LOW;
						continue;
	
					case REV_LEN_LOW:
						BCC ^= now_byte;
						p_NET_rve_stru->len = ((p_NET_rve_stru->len << 8)&0xff00) + now_byte;
						data_section_status = REV_DATE;
						continue;
	
					default:
						break;
				}
	
				if(write_offset < p_NET_rve_stru->len ) 				/*写入DATA	*/
				{
					p_NET_rve_stru->data[write_offset] = now_byte;
					BCC ^= p_NET_rve_stru->data[write_offset++];
					continue;
				}
	
				if((now_byte == BCC) && (write_offset > 0)) 		/*校验通过后则置起标志位  */
				{	 
					p_NET_rve_stru->data_flag = TRUE;
					if(TCP_PORT1 == NET_PORT)
					{
						p_NET_rve_stru =  &g_rev_NET1;
						sys_statis.tcp_ip.tcp_port1_rev_frame++;
					}
					else if(TCP_PORT2 == NET_PORT)
					{
						p_NET_rve_stru =  &g_rev_NET2;
						sys_statis.tcp_ip.tcp_port2_rev_frame++;
					}
					return TRUE;
				}
				break;
			}
			/*	frame_type == FT1	*/
			else if(0xFF == now_byte)
			{
				if((0 == BCC) && (write_offset >1)) /*校验通过，置标志位，并计算长度   */
				{	
					p_NET_rve_stru->data_flag = TRUE;
					p_NET_rve_stru->len = write_offset - 1;
					//p_NET_rve_stru->current_link_no = p_tcp_pcb_server3_index; /*记录当前消息的link号*/
				   
				   if(i+1 < len)
					{
						sys_statis.park.combine_tcp_data++;
					}
	
					return TRUE;
				}
	
				break;
			}
			else if(0xFE == now_byte)
			{
				continue;
			}
			else if(0xFE == pre_byte)				/*反转义  */
			{
				if(0x01 == now_byte)
				{
					p_NET_rve_stru->data[write_offset] = 0xFF;
				}
				else if(0x00 == now_byte)
				{
					p_NET_rve_stru->data[write_offset] = 0xFE;
				}
				else
				{
					break;
				}
			}
			else
			{
				p_NET_rve_stru->data[write_offset] = now_byte;
			}
	
			if(REV_DATE == data_section_status) 					/* 收数据DATA		 */
			{
				BCC ^= p_NET_rve_stru->data[write_offset];
				write_offset ++;
				continue;
			}
																	/*RSCTL  */
			BCC ^= p_NET_rve_stru->data[write_offset];
			p_NET_rve_stru->RSCTL = p_NET_rve_stru->data[write_offset];
	
			data_section_status = REV_DATE;
	
		}	 
		return FALSE;
	}


