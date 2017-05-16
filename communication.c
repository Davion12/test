/*******************************************************************************
�ļ���:Communication.c
�汾��:V1.0
��Ȩ˵��:
����:������   
��д����:2011.08.31
��Ҫ����:
�޸���:
�޸�����:
�޸ļ�¼:
*******************************************************************************/
/*******************************************************************************
//�ļ�����
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
//����
*******************************************************************************/


/*******************************************************************************
//��������
*******************************************************************************/
rev_UART_stru   g_rev_UART1 = {0, 0, FALSE, 0, {0},UART_1};            /*���ڽ���*/
rev_UART_stru   g_rev_UART3 = {0, 0, FALSE, 0, {0},UART_3};
rev_UART_stru   g_rev_UART4 = {0, 0, FALSE, 0, {0},UART_4};

rev_UART_stru   g_rev_NET1 = {0, 0, FALSE, 0, {0},TCP_PORT1};           /* ���ڽ��� */
rev_UART_stru   g_rev_NET2 = {0, 0, FALSE, 0, {0},TCP_PORT2};       


/*******************************************************************************
//��������
*******************************************************************************/

/******************************************
*�������ƣ�u8 antenna_mcu_send
*�������ܣ����ߺ�ͨ������1�������ݵ����ƺ�
*��ڲ�����atten_case_ID = CASE_1; CASE_1��Ӧ�Ŵ���1
           p_send_data->���ݵ�ַ�����ݳ���->len
           RSTCL         ֡���кţ�    Ӧ�ò����
           frame_type    ֡��ʽ        ȡֵ    FT1 ��ʽ1 ��FT2 ��ʽ2
*���ڲ�������          
*�� �� ֵ�� TRUE:  ����;  FALSE: �������ݹ���
*ȫ�ֱ�������
*���ú�������
******************************************/

u8 antenna_mcu_send(uc8 *p_send_data, u16 len, u8 RSCTL, u8 frame_type, u16 atten_case_ID)
{

    u16 i = 0;
    u16 send_len = 0;
    u8  BCC  = 0;
    u8  uart_send_buf[UART_SEND_BUF_LEN];        

    if(len >= DATA_LEN_MAX)                   //����ת�壬���������ݹ���������FALSE
    {   
        return FALSE;
    }
    if( FT1 == frame_type )                     //����֡��ʽһ����
    {
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = RSCTL;
        BCC = RSCTL;
        while( i < len )
        {
            BCC ^= p_send_data[i];
            switch( p_send_data[i] )            //ת��
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

        switch(BCC)                                 //BCC�Ƿ�ת��
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
    else                                            //֡��ʽ��
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
      
    switch(atten_case_ID)                            //��֡��ϣ�ѡ����ģʽ   
    {
      case UART_1:
          sys_statis.communicate.uart1_send_frame++;
#ifdef _ANTENNA_FW_
          if(FALSE == g_gprs_enter_exchange_flag)
          {
              send_len = hex_string_normal(uart_send_buf,uart_send_buf,send_len);
              /*��SMS��������*/
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

    if(len >= DATA_LEN_MAX)                   //����ת�壬���������ݹ���������FALSE
    {   
        return FALSE;
    }
    if( FT1 == frame_type )                     //����֡��ʽһ����
    {
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = 0xFF;
        uart_send_buf[send_len++] = RSCTL;
        BCC = RSCTL;
        while( i < len )
        {
            BCC ^= p_send_data[i];
            switch( p_send_data[i] )            //ת��
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

        switch(BCC)                                 //BCC�Ƿ�ת��
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
    else                                            //֡��ʽ��
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
      
    switch(atten_case_ID)                            //��֡��ϣ�ѡ����ģʽ   
    {
        case UART_1:
            sys_statis.communicate.uart1_send_frame++;
#ifdef _ANTENNA_FW_
            if(FALSE == g_gprs_enter_exchange_flag)
            {
				send_len = hex_string_normal(uart_send_buf,modem_send_buf,send_len);
				/*��SMS��������*/
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
	
			if((write_offset >= UART_REV_BUF_LEN) || (write_offset >= len)) 			   /*�������ݳ�����Χ��λ������֡ */
			{
				break;
			}
	
			pre_byte = now_byte;
			now_byte = *p_paked_data;		
			p_paked_data ++;
	
			//����֡ͷΪ����0XFF�����������֡ͷ�����һ��0XFF������
			if(TRUE == first_data_after_head)
			{
				first_data_after_head = FALSE;
				if(0xFF == now_byte)
				{
					continue;
				}
			}		
	
			/* û���յ�֡ͷ  */
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
	
			/* �յ���֡ͷ	 frame_type == FT2	 */
			else if(FT2 == p_NET_rve_stru->frame_type)
			{
				switch(data_section_status) 				/*�����ֶηֱ�洢	 */
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
	
				if(write_offset < p_NET_rve_stru->len ) 				/*д��DATA	*/
				{
					p_NET_rve_stru->data[write_offset] = now_byte;
					BCC ^= p_NET_rve_stru->data[write_offset++];
					continue;
				}
	
				if((now_byte == BCC) && (write_offset > 0)) 		/*У��ͨ�����������־λ  */
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
				if((0 == BCC) && (write_offset >1)) /*У��ͨ�����ñ�־λ�������㳤��   */
				{	
					p_NET_rve_stru->data_flag = TRUE;
					p_NET_rve_stru->len = write_offset - 1;
					//p_NET_rve_stru->current_link_no = p_tcp_pcb_server3_index; /*��¼��ǰ��Ϣ��link��*/
				   
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
			else if(0xFE == pre_byte)				/*��ת��  */
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
	
			if(REV_DATE == data_section_status) 					/* ������DATA		 */
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


