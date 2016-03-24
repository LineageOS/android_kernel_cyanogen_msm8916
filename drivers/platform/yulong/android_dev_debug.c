/****************************************************************************/
/*                                                                          */
/*             Copyright (c) 2004-2009  YULONG Company             �������� */
/*      ���������ͨ�ſƼ������ڣ����޹�˾  ��Ȩ���� 2004-2009              */
/*                                                                          */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the  ��             */
/* subject matter of this material.  All manufacturing, reproduction, use,  */
/* and sales rights pertaining to this subject matter are governed by the   */
/* license agreement.  The recipient of this software implicitly accepts    */
/* the terms of the license.                                                */
/* ������ĵ�������������˾���ʲ�,�κ���ʿ�Ķ���ʹ�ñ����ϱ�����          */
/* ��Ӧ��������Ȩ,�е��������κͽ�����Ӧ�ķ���Լ��.                         */
/*                                                                          */
/****************************************************************************/

/**************************************************************************
**  FileName:          MPCOMM.c  
**  Author:            �ƽݷ�
**  Version :          1.00
**  Date:              2009-06-17
**  Description:       ���ϲ�Ӧ���ṩһЩ����Modem�Ľӿڣ�����򿪡��رա���λ�����ѡ���ѯModem�Ƚӿڣ�
**                     ��Щ�ӿ��ϲ�Ӧ��ͨ��ioctrol�·���Ϣ����
**  History:         
**  <author>      <time>      <version >      <desc>
**   �ƽݷ�      2009-06-17    1.00           ����
**   �ƽݷ�      2009-10-12    1.01           �ڸ�λ����MPC_HWReset���������ж�����
**                                           
**************************************************************************/

/* 
 * MP Communication module
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/memory.h>
#include <linux/poll.h>
#include <mach/gpio.h>//Ϊ��ʹ������gpio�ĺ���
#include <linux/kernel.h>
#include <linux/slab.h>
#include "android_debug.h"


#define USB_SEL		100
#define SEL_UART	132

int android_debug_major;//Ԥ����android_device���豸��
struct ANDROID_DEBUG_DEV *g_pandroid_debug_dev = NULL;
static struct class *sg_pandroid_debug_class = NULL;

static int android_debug_release(struct inode *inode, struct file *filp);
static int android_debug_open(struct inode *inode, struct file *filp);
static long android_debug_iocontrol(struct file *filp, unsigned  int cmd, unsigned long arg);
static int android_debug_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);
static int android_debug_write(struct file *filp, const char __user *buf,  size_t count, loff_t *ppos);
static loff_t android_debug_seek(struct file *filp, loff_t offset, int orig);

static struct file_operations android_debug_fops = 
{
	.owner      = THIS_MODULE,
	.open        = android_debug_open,
	.release     = android_debug_release,
	.write       = android_debug_write,	
	.read        = android_debug_read,
	.llseek      = android_debug_seek,
	.unlocked_ioctl       = android_debug_iocontrol,
};


/**************************************************************************
* Function    : MPC_Resume
* Description : ��Դ������������ʱ�����
* Calls       :
* Called By   :
*          1����ϵͳҪ����ʱ���ɵ�Դ������ã�
* Input :  struct platform_device *dev
* Output : ��    
* Return : ��  
* others : ��
**************************************************************************/
static int android_debug_resume(struct platform_device *dev)
{
	return 0;
}


/**************************************************************************
* Function    : MPC_Suspend
* Description : ��Դ������������ʱ����õ�
* Calls       :
* Called By   :
*          1����ϵͳҪ����ʱ���ɵ�Դ������ã�
* Input :  
*          1��struct platform_device *dev
*          2��pm_message_t state
* Output : ��    
* Return : ��  
* others : ��
**************************************************************************/
static int android_debug_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}




/**************************************************************************
* Function    : MPC_Probe
* Description : �ַ��豸̽�⺯�������ڻ�ȡӲ����Դ��Ϣ���ж������
* Calls       :
* Called By   :
*          1��ϵͳ���������豸ʱ���ã�
* Input :  
*          1��*pdev  ƽ̨�豸ָ�룬ͨ���˺�����ö�Ӧ��Ӳ����Ϣ
* Output : ��    
* Return : 
*          1���ɹ�����0
*          2��ʧ�ܷ��ط�0 
* others : 
**************************************************************************/
static int android_debug_probe(struct platform_device *pdev)
{
	printk("++android_debug_probe()!\r\n");
	if(NULL == pdev)
	{
		printk(KERN_ERR"pdev =NULL\r\n");
		return -ENOMEM;	
	}

	printk("--android_debug_probe()!\r\n");

	return 0;
}


/**************************************************************************
* Function    : MPC_Remove
* Description : ɾ��Ӳ����Դ��Ϣ���ж������
* Calls       :
* Called By   :
*          1����Ҫɾ���豸ʱ����kernel���ã�
* Input :  
*          1��*dev ƽ̨�豸ָ�� 
* Output : ��    
* Return : ����0
* others : 
**************************************************************************/
static int android_debug_remove(struct platform_device *dev)
{
	if(NULL == dev)
	{
		printk(KERN_INFO"dev =NULL\r\n");
		return -ENOMEM;	
	}
	
	if(NULL != g_pandroid_debug_dev)
	{
		kfree(g_pandroid_debug_dev);
		g_pandroid_debug_dev = NULL;
	}
	cdev_del(&g_pandroid_debug_dev->android_debug_cdev);//ɾ��cdev
       unregister_chrdev_region(MKDEV(android_debug_major, 0), 1);//ע���豸����
	return 0;
}


static struct platform_driver ANDROID_DEBUG_DRIVER = 
{
	.probe		= android_debug_probe,
	.remove		= android_debug_remove,
	.suspend	= android_debug_suspend,
	.resume		= android_debug_resume,
	.driver		= 
	{
		.name	= "ANDROID_DEBUG",
		.owner	= THIS_MODULE,
	},	
};



/**************************************************************************
* Function    : modem_setup_cdev
* Description : ��ʼ���豸�ṹ�壬��������ӵ�ϵͳ���豸����ʼ���豸����ָ��
* Calls       :
* Called By   :
*          1����ϵͳ��ʼ��ʱ��MPC_Init����
* Input :  
*          1��*dev  �豸�ṹ��ָ�� 
*          2��index  ���豸����ʼֵ 
* Output : ��    
* Return : ��  
* others : 
**************************************************************************/
void android_debug_setup_cdev(struct ANDROID_DEBUG_DEV *dev, int dev_index)
{
	int err = 0;
    int dev_number = 0;
    int maxdevicenum = 1;

	printk("++android_debug_setup_cdev()!\r\n");
	if(dev == NULL)
	{
		printk(KERN_ERR"dev =NULL\r\n");
		return;	
	}

	dev_number = MKDEV(android_debug_major, dev_index);
	
	cdev_init(&dev->android_debug_cdev, &android_debug_fops);
	
	dev->android_debug_cdev.owner = THIS_MODULE;
	dev->android_debug_cdev.ops   = &android_debug_fops;
	
	err = cdev_add(&dev->android_debug_cdev, dev_number, maxdevicenum);	
	if(err)
	{
		printk(KERN_ERR"android_debug_cdev: Error %d adding android_debug_cdev\r\n",err);	
	}

	printk("--android_debug_setup_cdev()!\r\n");
}

void switch_tail_to_wifi_log(void)
{


}
void switch_tail_to_wifi_rf(void)
{


}

void switch_tail_to_ap_usb(void)
{
	gpio_direction_output(USB_SEL, 0);
	gpio_direction_output(SEL_UART, 0);
}

void switch_tail_to_ap_uart(void)
{
	gpio_direction_output(USB_SEL, 1);
	gpio_direction_output(SEL_UART, 0);
}

void switch_tail_to_modem(void)
{
	gpio_direction_output(USB_SEL, 1);
	gpio_direction_output(SEL_UART, 1);
}

static void uart_switch(int uartSwitch)
{
	switch(uartSwitch)
	{
		case IOCTL_UART_SWITCH_AP:
			printk("uart switch ap\r\n");
			switch_tail_to_ap_uart();

			break;

		case IOCTL_UART_SWITCH_WIFI_LOG:
			printk("uart switch wifi log\r\n");
			switch_tail_to_wifi_log();

			break;

		case IOCTL_UART_SWITCH_MODEM:
			printk("uart switch modem\r\n");
			switch_tail_to_modem();
	
			break;
			
		case IOCTL_UART_SWITCH_WIFI_RF_TEST:
			printk("uart switch wifi rf test\r\n");
			switch_tail_to_wifi_rf();
			printk("uart switch successful\n");
			break;

		default:
			printk("other uart switch\r\n");
		break;
	}
}

static void usb_switch(int usbSwitch)
{
	switch(usbSwitch)
	{
		case IOCTL_USB_SWITCH_MODEM:


			break;

		case IOCTL_USB_SWITCH_AP:
			switch_tail_to_ap_usb();
			break;
			
		default:
			printk("Don't support usb switch\r\n");
		break;
	}

	return;
}
/**************************************************************************
* Function    : MPC_Init
* Description : ��ʼ���豸
* Calls       :
* Called By   :
*          1����ϵͳ���������豸ʱ����kernel���ã�
* Input :  �� 
* Output : ��    
* Return : 
*          1���ɹ�����0��
*          2��ʧ�ܷ�������ֵ  
* others : 
**************************************************************************/
static int __init android_debug_init(void)
{
	int dwRet = 0;
	dev_t dev_number = 0;
	int base_number = 0;
	int max_device_number = 1;

	printk("++android_debug_init()!\r\n");

    dwRet = alloc_chrdev_region(&dev_number, base_number, max_device_number, "android-debug-device");	
    if(!dwRet)
    {
		android_debug_major = MAJOR(dev_number);
		printk(KERN_DEBUG"android_debug_init: android_debug major number is %d!\r\n",android_debug_major);
    }
    else
    {
		printk(KERN_ERR"android_debug_init: request android_debug major number fail!\r\n");
		goto out;
    }

	g_pandroid_debug_dev = kmalloc(sizeof(struct ANDROID_DEBUG_DEV), GFP_KERNEL);	
	if(!g_pandroid_debug_dev)/*alloc failure*/	
	{		
		printk(KERN_ERR"android_debug_init: kmalloc g_pandroid_debug_dev fail!\r\n");
		dwRet = -ENOMEM;		
		goto erro_unchr;
	}	
	memset(g_pandroid_debug_dev, 0, sizeof(struct ANDROID_DEBUG_DEV));

	android_debug_setup_cdev(g_pandroid_debug_dev, base_number);

	//�����豸�ڵ�
	sg_pandroid_debug_class = class_create(THIS_MODULE, "android-debug-class");
	if(IS_ERR(sg_pandroid_debug_class))
	{
		printk(KERN_ERR"android_debug_init: android_debug_device register node fail!\r\n");
	}
	device_create(sg_pandroid_debug_class, NULL, MKDEV(android_debug_major, base_number), NULL, "yl_android_device");

	dwRet = platform_driver_register(&ANDROID_DEBUG_DRIVER);
	if(dwRet != 0)
	{
		printk(KERN_ERR"android_debug_init: platform_driver_register of android_debug is failure!\r\n");
		goto erro_undrv;
	}

	printk("--android_debug_init()!\r\n");
	return dwRet;//�ɹ�����0
	
erro_undrv:	
	if(NULL != g_pandroid_debug_dev)
	{
		kfree(g_pandroid_debug_dev);
		g_pandroid_debug_dev = NULL;	
	}	
	
erro_unchr:
	unregister_chrdev_region(dev_number, max_device_number);
	
out:
	printk(KERN_ERR"--android_debug_init Fail!\r\n");	
	return dwRet;	
} 


/**************************************************************************
* Function    : MPC_Deinit
* Description : ж���豸���ͷ��ڴ桢�����
* Calls       :
* Called By   :
*          1��ж���豸ʱ��Ӧ�õ��ã�
* Input :  �� 
* Output : ��    
* Return : ��  
* others : 
**************************************************************************/
// Device deinit - devices are expected to close down.
// The device manager does not check the return code.
static void __exit android_debug_deinit(void)
{
	return;	
}


/**************************************************************************
* Function    : MPC_Open
* Description : ���豸
* Calls       :
* Called By   :
*          1����Ҫ���豸���в���֮ǰ��ͨ��open�������ã�
* Input :  
*          1��*inode �豸�ڵ�ָ��
*          2��*filp  �豸�ļ�ָ��
* Output : ��    
* Return : �ɹ�����0 
* others : 
**************************************************************************/
static int android_debug_open(struct inode *inode, struct file *filp)
{
    printk("android_debug_open: +-\r\n");
    return 0;
} 


/**************************************************************************
* Function    : MPC_Release
* Description : �ر��豸
* Calls       :
* Called By   :
*          1����Ҫ�ر��豸ʱ����Ӧ��ͨ��release�������ã�
* Input :  
*          1��*inode �豸�ڵ�ָ��
*          2��*filp  �豸�ļ�ָ��
* Output : ��    
* Return : ����0
* others : 
**************************************************************************/
static int android_debug_release(struct inode *inode, struct file *filp)
{
	return 0;
} 


/**************************************************************************
* Function    : MPC_IOControl
* Description : ͨ��cmd�����·���Ϣ��ʵ�ֶ�ģ��ĸ��ֿ���
* Calls       :
* Called By   :
*          1���ϲ�Ӧ��ͨ��ioctl�������ã�
* Input :  
*          1��*inodep �豸�ڵ�ָ��
*          2��*filp �ļ�ָ��
*          3��cmd  �����룬ͨ���ò����·���Ϣ
*          4��arg  �ϲ�Ӧ�ú͵ײ������������ݵĲ���
* Output : 
*          1��arg  �ϲ�Ӧ�ú͵ײ������������ݵĲ���   
* Return : 
*          1���ɹ����أ�0
*          2��ʧ�ܷ��أ�-EINVAL��cmd�������ʱ����  
* others : 
*    1���ƽݷ�     2009-09-22    ���ڴ������Ĳ���arg��һ���ṹ�壬�ṹ����
*                                �ִ���ָ�룬����Ҫ�Խṹ���ָ�����ʱ����
*                                Ҫ��һ��copy_to_user/copy_from_user     
**************************************************************************/
//extern void set_alarm_test(int value);
static long android_debug_iocontrol(struct file *filp, unsigned  int cmd, unsigned long arg)
{
	//void __user *tmp = (void __user *)arg;
	printk("android_debug_iocontrol: ++\r\n");	
    
	switch(cmd)
	{
		case IOCTL_UART_SWITCH_AP:
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_AP\r\n");
			uart_switch(IOCTL_UART_SWITCH_AP);
			break;
			
		case IOCTL_UART_SWITCH_WIFI_LOG://WIFI������Ϣ
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_WIFI_LOG\r\n");
			uart_switch(IOCTL_UART_SWITCH_WIFI_LOG);
			break;			

		case IOCTL_UART_SWITCH_MODEM:
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_MODEM\r\n");
			uart_switch(IOCTL_UART_SWITCH_MODEM);		
			break;

		case IOCTL_UART_SWITCH_WIFI_RF_TEST:
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_WIFI_RF_TEST\r\n");
			uart_switch(IOCTL_UART_SWITCH_WIFI_RF_TEST);
			break;

		case IOCTL_USB_SWITCH_MODEM:
			printk("android_debug_iocontrol(): IOCTL_USB_SWITCH_MODEM\n");
			usb_switch(IOCTL_USB_SWITCH_MODEM);
			break;

		case IOCTL_USB_SWITCH_AP:
			printk("android_debug_iocontrol(): IOCTL_USB_SWITCH_AP\n");
			usb_switch(IOCTL_USB_SWITCH_AP);
			break;

		default:
			printk(KERN_WARNING"No supporting cmd\r\n");
			return -EINVAL;
	}

	printk("android_debug_iocontrol: --\r\n");
   	return 0;
}  


/**************************************************************************
* Function    : MPC_Read
* Description : ģ����������ú���Ŀǰ��δ�õ�
* Calls       :
* Called By   :
*             1���ϲ�Ӧ��ͨ��read�������ã�
* Input :  
*             1��*filp �ļ�ָ��
*             2��*ppos 
* Output : 
*             1��*buf  Ҫ�������ݻ�����ָ��
*             2��count��Ҫ���������ֽ�    
* Return : ����ֵΪ0   
* others : ��
**************************************************************************/	
static int android_debug_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}


/**************************************************************************
* Function    : MPC_Write
* Description : ģ��д�������ú���Ŀǰ��δ�õ�
* Calls       :
* Called By   :
*             1���ϲ�Ӧ��ͨ��write��������
* Input :  
*             1��*filp �ļ�ָ��
*             2��*buf  Ҫд�����ݻ�����ָ��
*             3��count��Ҫд�������ֽ�
*             4��*ppos
* Output : ��    
* Return : ����ֵΪ0  
* others : ��
**************************************************************************/
static int android_debug_write(struct file *filp, const char __user *buf,  size_t count, loff_t *ppos)
{
	char arg[32] = {0};
	int ret=0;
	ret=copy_from_user(arg, buf, sizeof(arg)-1);
	printk("%s:arg=%s\n", __func__, arg);
	if(!strncmp(arg, "usb", strlen("usb")))
	{
		printk(KERN_ERR "%s:tail switch to ap usb\n", __func__);
    		switch_tail_to_ap_usb();
	}
	else if(!strncmp(arg, "uart", strlen("uart")))
	{
		printk(KERN_ERR "%s:tail switch to ap uart\n", __func__);
		switch_tail_to_ap_uart();
	}
	return 0;
}


/**************************************************************************
* Function    : MPC_Seek
* Description : ָ�붨λ�������ú���Ŀǰ��δ�õ�
* Calls       :
* Called By   :
*          1���ϲ�Ӧ��ͨ��seek���ã�
* Input :  
*          1��*filp �ļ�ָ��
*          2��offset ָ��ƫ��
*          3��orig
* Output : ��    
* Return : ����ֵ0  
* others : ��
**************************************************************************/
static loff_t android_debug_seek(struct file *filp, loff_t offset, int orig)
{
	return 0;
}


module_init(android_debug_init);
module_exit(android_debug_deinit);
MODULE_AUTHOR("Huang Jiefeng");
MODULE_LICENSE("GPL");
