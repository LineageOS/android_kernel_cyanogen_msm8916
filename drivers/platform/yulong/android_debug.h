/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2008-2010  YULONG Company             ��������������     */
/*                 ���������ͨ�ſƼ������ڣ����޹�˾  ��Ȩ���� 2008-2010 */
/*                                                                        */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the  ������������ */
/* subject matter of this material.  All manufacturing, reproduction, use,*/
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement.  The recipient of this software implicitly accepts  */ 
/* the terms of the license.                                              */
/* ������ĵ�������������˾���ʲ�,�κ���ʿ�Ķ���ʹ�ñ����ϱ�����        */
/* ��Ӧ��������Ȩ,�е��������κͽ�����Ӧ�ķ���Լ��.                       */
/*                                                                        */
/*                                                                        */
/**************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2010, Yulong Tech. Co., Ltd.
**  FileName:          MPCOMM.h  
**  Author:            �ƽݷ�
**  Version :          1.00
**  Date:              2009-06-17
**  Description:       ͷ�ļ����궨��ioctrol��ֵ
**                     ��������������һЩ�ṹ��
**  History:         
**  <author>      <time>      <version >      <desc>
**   �ƽݷ�      2009-06-17    1.00           ����
**   �ƽݷ�      2009-10-12    1.01           Ϊ�˼�����ǰ��ͨ�ų����޸�ioctrolcmd�궨���ֵ
**                                           
**************************************************************************/

#ifndef __ANDROID_DEBUG_H__
#define __ANDROID_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif


#define IOCTL_UART_MAGIC                        'D'
#define IOCTL_UART_SWITCH_AP                   	_IOWR(IOCTL_UART_MAGIC, 1, unsigned long )
#define IOCTL_UART_SWITCH_WIFI_LOG            	_IOWR(IOCTL_UART_MAGIC, 2, unsigned long )
#define IOCTL_UART_SWITCH_MODEM               	_IOWR(IOCTL_UART_MAGIC, 3, unsigned long )
#define IOCTL_UART_SWITCH_WIFI_RF_TEST       	_IOWR(IOCTL_UART_MAGIC, 4, unsigned long )
#define IOCTL_USB_SWITCH_MODEM              	_IOWR(IOCTL_UART_MAGIC, 5, unsigned long )
#define IOCTL_USB_SWITCH_AP                   	_IOWR(IOCTL_UART_MAGIC, 6, unsigned long )
#define IOCTL_GET_CPU_VERSION                  	_IOWR(IOCTL_UART_MAGIC, 7, unsigned long )

#ifdef __cplusplus
}
#endif

struct ANDROID_DEBUG_DEV
{
	struct cdev android_debug_cdev;/*cdev�ṹ��*/
	int major;
	int name;
};

#endif // __MPCOMM_H__
