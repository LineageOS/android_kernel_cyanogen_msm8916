/********************************************************************************/
/*																				*/
/* Copyright (c) 2000-2010  YULONG Company             ��������������       	*/
/*         ���������ͨ�ſƼ������ڣ����޹�˾  ��Ȩ���� 2000-2010               */
/*																				*/
/* PROPRIETARY RIGHTS of YULONG Company are involved in the           			*/
/* subject matter of this material.  All manufacturing, reproduction, use,      */
/* and sales rights pertaining to this subject matter are governed by the     	*/
/* license agreement.  The recipient of this software implicitly accepts        */
/* the terms of the license.                                                    */
/* ������ĵ�������������˾���ʲ�,�κ���ʿ�Ķ���ʹ�ñ����ϱ�����     			*/
/* ��Ӧ��������Ȩ,�е��������κͽ�����Ӧ�ķ���Լ��.                             */
/*																				*/
/********************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2010, Yulong Tech. Co., Ltd.
**  FileName:          touchscreen.c
**  Author:            �봺��
**  Version :          2.00
**  Date:                2011/11/19
**  Description:       �����������м��
**
**  History:
**  <author>      <time>      <version >      <desc>
**   �봺�� 2012/01/16     2.20           �޸Ĵ������ڵ����ԣ���ͬ���޸�init.rc����chown system
**   �봺�� 2012/01/11     2.10           ���Ӵ���������mutex lock����
**   �봺�� 2012/01/04     2.00           �޸�ֻ��ѡ���Ĵ������в���
**   �봺�� 2011/11/19     1.00           ����
**
**************************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/input/touchscreen_yl.h>
#include <linux/power_supply.h>


/*begin to transplant TW charge status detect code from 8675_C00,
 you must include linux/power_supply.h,the TW add charger status detect function, 0 is charger offline,1 is charger online
by liushilong@yulong.com on 2014-11-6 11:10*/
void (*touch_charger_notify)(void);
static int charger_status=0;

/****************************************************
 *		0 charger offline
 *		1 charger online
 *****************************************************/
unsigned int touch_get_charger_status(void)
{
	if(charger_status)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void touch_register_charger_notify(void (*fn)(void))
{
	touch_charger_notify = fn;
}
void touch_unregister_charger_notify(void)
{
	touch_charger_notify = NULL;
}
void touch_charger_status_changed(struct power_supply *psy)
{
	union power_supply_propval value;
	int ret = 0;

	pr_debug("%s: enter touch_charger_status_changed(), psy->type = %d.\n",
			__func__, psy->type);

	if(psy->type != POWER_SUPPLY_TYPE_USB &&
			psy->type != POWER_SUPPLY_TYPE_MAINS &&
			psy->type != POWER_SUPPLY_TYPE_USB_DCP &&
			psy->type != POWER_SUPPLY_TYPE_USB_CDP &&
			psy->type != POWER_SUPPLY_TYPE_USB_ACA)
		return;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	if(ret < 0)
	{
		pr_err("%s: get_property failed.\n", __func__);
		return;
	}

	if(value.intval)
	{
		charger_status |= 1<<psy->type;
	}
	else
	{
		charger_status &= ~(1<<psy->type);
	}


	pr_debug("%s:  charger_status = %d.\n", __func__, charger_status);

	if (touch_charger_notify)
	{
		touch_charger_notify();
	}

}
/*end to transplant TW charge status detect code from 8675_C00,
 you must include linux/power_supply.h,the TW add charger status detect function, 0 is charger offline,1 is charger online
by liushilong@yulong.com on 2014-11-6 11:10*/


touchscreen_ops_tpye *touchscreen_ops[2];

#define TOUCH_IN_ACTIVE(num) (touchscreen_ops[num] && touchscreen_ops[num]->active && touchscreen_ops[num]->active())
static DEFINE_MUTEX(touchscreen_mutex);

/**********************************************************************
* �������ƣ�touchscreen_set_ops

* �������������ô������ڵ��������

* ���������touchscreen_ops_tpye

* ���������NONE

* ����ֵ      ��0---�ɹ���-EBUSY---ʧ��

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
int touchscreen_set_ops(touchscreen_ops_tpye *ops)
{
	if(ops==NULL || ops->touch_id>1 )
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:ops error!\n");
		return -EBUSY;
	}
	mutex_lock(&touchscreen_mutex);
	if(touchscreen_ops[ops->touch_id]!=NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:ops has been used!\n");
		mutex_unlock(&touchscreen_mutex);
		return -EBUSY;
	}
       touchscreen_ops[ops->touch_id] = ops;
	mutex_unlock(&touchscreen_mutex);
	printk("BJ_BSP_Driver:CP_Touchscreen:ops add success!\n");
	return 0;
}

/**********************************************************************
* �������ƣ�touchscreen_type_show

* ��������������ǰ����������

* ���������buf

* ���������buf: 1---��������2---������

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_type_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;

	if (buf==NULL)
	{
		dev_dbg(dev, "BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0))
	{
		ret = touchscreen_ops[0]->touch_type;
	}
	else if (TOUCH_IN_ACTIVE(1))
	{
		ret = touchscreen_ops[1]->touch_type;
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%d\n",ret);

}

/**********************************************************************
* �������ƣ�touchscreen_active_show

* ��������������ǰ������״̬

* ���������buf

* ���������buf: 1--��ǰʹ��״̬��0--����״̬

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_active_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;
	int ret1=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (touchscreen_ops[0] && touchscreen_ops[0]->active)
	{
		ret = touchscreen_ops[0]->active();
	}

	if (touchscreen_ops[1] && touchscreen_ops[1]->active)
	{
		ret1 = touchscreen_ops[1]->active();
	}
	mutex_unlock(&touchscreen_mutex);

	printk("BJ_BSP_Driver:CP_Touchscreen:%d,%d in %s\n",ret,ret1,__FUNCTION__);
	return sprintf(buf, "%d,%d\n",ret,ret1);
}

/**********************************************************************
* �������ƣ�touchscreen_firmware_update_show

* ������������ѯ�������̼��Ƿ���Ҫ����

* ���������buf

* ���������buf: 1--��Ҫ�����̼���0--�̼��Ѿ�������

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t touchscreen_firmware_update_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->firmware_need_update)
	       	ret = touchscreen_ops[0]->firmware_need_update();
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->firmware_need_update)
	       	ret = touchscreen_ops[1]->firmware_need_update();
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%d\n",ret);
}


/**********************************************************************
* �������ƣ�touchscreen_firmware_update_store

* �������������¹̼�

* ���������buf:ϵͳд"update"

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_firmware_update_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	if(strncmp(buf, "update",count-1))
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:string is %s,count=%zu not update!\n",buf,count);
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
	     if(touchscreen_ops[0]->firmware_need_update && touchscreen_ops[0]->firmware_need_update() && touchscreen_ops[0]->firmware_do_update)
		{
                   ret = touchscreen_ops[0]->firmware_do_update();
            	}
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
  	     if(touchscreen_ops[1]->firmware_need_update && touchscreen_ops[1]->firmware_need_update() && touchscreen_ops[1]->firmware_do_update)
		{
                   ret = touchscreen_ops[1]->firmware_do_update();
            	}
	}
	mutex_unlock(&touchscreen_mutex);
	return ret;
}


/**********************************************************************
* �������ƣ�touchscreen_calibrate_store

* ����������������У׼

* ���������buf:ϵͳд"calibrate"

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t touchscreen_calibrate_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->need_calibrate)
	       	ret = touchscreen_ops[0]->need_calibrate();
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->need_calibrate)
	       	ret = touchscreen_ops[1]->need_calibrate();
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%d\n",ret);
}

/**********************************************************************
* �������ƣ�touchscreen_calibrate_store

* ����������������У׼

* ���������buf:ϵͳд"calibrate"

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_calibrate_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	if(strncmp(buf, "calibrate",count-1))
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:string is %s,not calibrate!\n",buf);
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->calibrate)
	       	ret = touchscreen_ops[0]->calibrate();
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->calibrate)
	       	ret = touchscreen_ops[1]->calibrate();
	}
	mutex_unlock(&touchscreen_mutex);
	return ret;

}


/**********************************************************************
* �������ƣ�touchscreen_firmware_version_show

* ����������������дģʽ

* ���������buf

* ���������buf ���������ҡ�ic�ͺš��̼��汾

* ����ֵ      ��size ���س���

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_firmware_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	char version[32]={0};

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->get_firmware_version)
	       	touchscreen_ops[0]->get_firmware_version(version);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->get_firmware_version)
	       	touchscreen_ops[1]->get_firmware_version(version);
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%s\n",version);
}


/**********************************************************************
* �������ƣ�touchscreen_reset_store

* ����������������дģʽ

* ���������buf:ͨ��д"reset"

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_reset_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0;


	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}


	if(strncmp(buf, "reset",count-1))
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:string is %s,not reset!\n",buf);
		return -EINVAL;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->reset_touchscreen)
	       	ret = touchscreen_ops[0]->reset_touchscreen();
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->reset_touchscreen)
	       	ret = touchscreen_ops[1]->reset_touchscreen();
	}
	mutex_unlock(&touchscreen_mutex);

	return ret;

}

/**********************************************************************
* �������ƣ�touchscreen_mode_show

* ��������������ǰ����������ģʽ

* ���������buf

* ���������buf: 1--��д��0--����

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char* mode;

	if( NULL == buf )
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->get_mode)
		ret = touchscreen_ops[0]->get_mode();
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->get_mode)
		ret = touchscreen_ops[1]->get_mode();
	}
	mutex_unlock(&touchscreen_mutex);


	switch( ret ){
                case MODE_INVALID:
			mode = "invalid";
                        break;

                case MODE_NORMAL:
			mode = "normal";
                        break;

                case MODE_HANDWRITE:
			mode = "handwrite";
                        break;

                case MODE_GLOVE:
			mode = "glove";
                        break;

                case MODE_WINDOW:
			mode = "window";
                        break;

                case MODE_MAX:
			mode = "invalid";
                        break;

                default:
			mode = "invalid";
                        break;
        }


	return sprintf(buf, "%s\n", mode);
}

/**********************************************************************
* �������ƣ�touchscreen_work_mode_store

* �������������ù���ģʽ

* ���������buf:���뷨"handwrite" "normal"

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_mode_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0;
	int mode=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	if(strncmp(buf, "normal",count-1)==0)
		mode=MODE_NORMAL;
	else if(strncmp(buf, "handwrite",count-1)==0)
		mode=MODE_HANDWRITE;
	else if(strncmp(buf, "glove",count-1)==0)
		mode=MODE_GLOVE;
	else if(strncmp(buf, "window",count-1)==0)
		mode=MODE_WINDOW;
	else
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:Don't support %s mode!\n",buf);
		return -EINVAL;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->set_mode)
	       	ret = touchscreen_ops[0]->set_mode(mode);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->set_mode)
	       	ret = touchscreen_ops[1]->set_mode(mode);
	}
	mutex_unlock(&touchscreen_mutex);

	return count;
}

/**********************************************************************
* �������ƣ�touchscreen_oreitation_show

* ������������ȡ��������ǰ��λ

* ���������buf

* ���������buf:������Ӧ��"X"    X:1,2,3,4--> 0,90,180,270

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_oreitation_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->get_oreitation)
			ret = touchscreen_ops[0]->get_oreitation();
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->get_oreitation)
			ret = touchscreen_ops[1]->get_oreitation();
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%d\n",ret);
}


/**********************************************************************
* �������ƣ�touchscreen_oreitate_store

* �������������ô�������ǰ��λ

* ���������buf:������Ӧ��"oreitation:X"    X:1,2,3,4--> 0,90,180,270

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_oreitation_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	ssize_t ret=0;
	int oreitation=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}


	if(strncmp(buf, "oreitation",count-2))
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:string is %s,not oreitation\n",buf);
		return -EINVAL;
	}

	oreitation=buf[count-2]-'0';
	printk("BJ_BSP_Driver:CP_Touchscreen:oreitation=%d",oreitation);
	if(oreitation<0 || oreitation>3)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:oreitation[%d] is invalid\n",oreitation);
		return -EINVAL;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->set_oreitation)
	       	ret = touchscreen_ops[0]->set_oreitation(oreitation);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->set_oreitation)
	       	ret = touchscreen_ops[1]->set_oreitation(oreitation);
	}
	mutex_unlock(&touchscreen_mutex);

	return ret;

}


/**********************************************************************
* �������ƣ�touchscreen_read_regs_show

* ������������������ic�Ĵ���

* ���������NONE

* ���������buf[256]: ef:ab [�Ĵ���]��ֵ

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_regs_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;
	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0] ->read_regs)
			ret=touchscreen_ops[0] ->read_regs(buf);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1] ->read_regs)
			ret=touchscreen_ops[1] ->read_regs(buf);
	}
	mutex_unlock(&touchscreen_mutex);

	return ret;
}


/**********************************************************************
* �������ƣ�touchscreen_write_regs_store

* ����������д�������Ĵ���ֵ

* ���������buf[256]: ef:ab [�Ĵ���]��ֵ

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_regs_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0;
	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0] ->write_regs)
			ret=touchscreen_ops[0] ->write_regs(buf);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1] ->write_regs)
			ret=touchscreen_ops[1] ->write_regs(buf);
	}
	mutex_unlock(&touchscreen_mutex);
	return ret;
}


/**********************************************************************
* �������ƣ�touchscreen_debug_store

* �������������õ���ģʽ

* ���������buf:"on" "off"

* ���������buf

* ����ֵ      ��size

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2011/11/19	   �봺��                  �� ��
**********************************************************************/
static ssize_t  touchscreen_debug_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0;
	int on=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	if(strncmp(buf, "on",count-1)==0)
		on=1;

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0] ->debug)
			ret= touchscreen_ops[0] ->debug(on);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1] ->debug)
			ret= touchscreen_ops[1] ->debug(on);
	}
	mutex_unlock(&touchscreen_mutex);
	return count;
}
/**********************************************************************
* �������ƣ�touchscreen_vendor_show

* ������������ѯ����IC����

* ���������buf

* ���������buf ������IC ����

* ����ֵ      ��size ���س���

* ����˵����

* �޸�����         �޸���	              �޸�����
* --------------------------------------------------------------------
* 2013/7/10	   �ƻ���                 �� ��
**********************************************************************/
static ssize_t  touchscreen_vendor_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	char vendor[32]={0};

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->get_vendor)
	       	touchscreen_ops[0]->get_vendor(vendor);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->get_vendor)
	       	touchscreen_ops[1]->get_vendor(vendor);
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%s\n",vendor);
}

static ssize_t  touchscreen_gesture_wakeup_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    char gesture[64]={0};

    if(buf==NULL)
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
        return -ENOMEM;
	}

    mutex_lock(&touchscreen_mutex);
    if(TOUCH_IN_ACTIVE(0))
    {
        if(touchscreen_ops[0]->get_wakeup_gesture)
            touchscreen_ops[0]->get_wakeup_gesture(gesture);
	}
	else if(TOUCH_IN_ACTIVE(1))
    {
		if(touchscreen_ops[1]->get_wakeup_gesture)
			touchscreen_ops[1]->get_wakeup_gesture(gesture);
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%s\n",gesture);
}

static ssize_t  touchscreen_gesture_ctrl_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	char gesture[64]={0};

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->get_gesture_ctrl)
			touchscreen_ops[0]->get_gesture_ctrl(gesture);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->get_gesture_ctrl)
			touchscreen_ops[1]->get_gesture_ctrl(gesture);
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%s\n", gesture);
}

static ssize_t  touchscreen_gesture_ctrl_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->gesture_ctrl)
			ret = touchscreen_ops[0]->gesture_ctrl(buf);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->gesture_ctrl)
			ret = touchscreen_ops[1]->gesture_ctrl(buf);
      }
     mutex_unlock(&touchscreen_mutex);

    return count;
}

static ssize_t touchscreen_charger_state_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;
    char charger_state[16] = {0};
	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->get_charger_state)
			ret = touchscreen_ops[0]->get_charger_state(charger_state);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->get_charger_state)
			ret = touchscreen_ops[1]->get_charger_state(charger_state);
	}
	mutex_unlock(&touchscreen_mutex);

	return sprintf(buf, "%s\n", charger_state);
}

static ssize_t  touchscreen_charger_state_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;

	if(buf==NULL)
	{
		printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if(TOUCH_IN_ACTIVE(0))
	{
		if(touchscreen_ops[0]->set_charger_state)
			ret = touchscreen_ops[0]->set_charger_state(buf);
	}
	else if(TOUCH_IN_ACTIVE(1))
	{
		if(touchscreen_ops[1]->set_charger_state)
	       	ret = touchscreen_ops[1]->set_charger_state(buf);
	}
	mutex_unlock(&touchscreen_mutex);

	return count;
}

static DEVICE_ATTR(type, 0444, touchscreen_type_show, NULL);
static DEVICE_ATTR(active, 0444, touchscreen_active_show, NULL);
static DEVICE_ATTR(firmware_update, 0664, touchscreen_firmware_update_show, touchscreen_firmware_update_store);
static DEVICE_ATTR(calibrate, 0664, touchscreen_calibrate_show, touchscreen_calibrate_store);
static DEVICE_ATTR(firmware_version, 0444, touchscreen_firmware_version_show,NULL);
static DEVICE_ATTR(reset, 0224, NULL, touchscreen_reset_store);
static DEVICE_ATTR(mode, 0664, touchscreen_mode_show, touchscreen_mode_store);
static DEVICE_ATTR(oreitation, 0664, touchscreen_oreitation_show, touchscreen_oreitation_store);
static DEVICE_ATTR(regs, 0664, touchscreen_regs_show, touchscreen_regs_store);
static DEVICE_ATTR(debug, 0224, NULL, touchscreen_debug_store);
//static DEVICE_ATTR(get_rawdata, 0664, ft5x0x_rawdata_show, ft5x0x_rawdata_store);//add by Yun
static DEVICE_ATTR(vendor, 0444, touchscreen_vendor_show, NULL);
static DEVICE_ATTR(gesture_wakeup, 0444, touchscreen_gesture_wakeup_show, NULL);
static DEVICE_ATTR(gesture_ctrl, 0664, touchscreen_gesture_ctrl_show, touchscreen_gesture_ctrl_store);
static DEVICE_ATTR(charger_state, 0664, touchscreen_charger_state_show, touchscreen_charger_state_store);

static const struct attribute *touchscreen_attrs[] = {
	&dev_attr_type.attr,
	&dev_attr_active.attr,
	&dev_attr_firmware_update.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_reset.attr,
	&dev_attr_mode.attr,
	&dev_attr_oreitation.attr,
	&dev_attr_regs.attr,
	&dev_attr_debug.attr,
//	&dev_attr_get_rawdata.attr,//add by Yun
	&dev_attr_vendor.attr,
	&dev_attr_gesture_wakeup.attr,
	&dev_attr_gesture_ctrl.attr,
	&dev_attr_charger_state.attr,
	NULL,
};

static const struct attribute_group touchscreen_attr_group = {
	.attrs = (struct attribute **) touchscreen_attrs,
};

static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len)
{
	return 1;
}

static ssize_t unexport_store(struct class *class, struct class_attribute *attr,const char *buf, size_t len)
{
	return 1;
}

static struct class_attribute uart_class_attrs[] = {
	__ATTR(export, 0200, NULL, export_store),
	__ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

static struct class touchscreen_class = {
	.name =		"touchscreen",
	.owner =	THIS_MODULE,

	.class_attrs =	uart_class_attrs,
};

static struct device *touchscreen_dev;
struct device *touchscreen_get_dev(void)
{
    return touchscreen_dev;
}
EXPORT_SYMBOL(touchscreen_get_dev);

static int touchscreen_export(void)
{
	int	status = 0;
	struct device	*dev = NULL;

	dev = device_create(&touchscreen_class, NULL, MKDEV(0, 0), NULL, "touchscreen_dev");
	if (dev)
	{
		status = sysfs_create_group(&dev->kobj, &touchscreen_attr_group);
		touchscreen_dev = dev;
	}
	else
	{
		printk(KERN_ERR"BJ_BSP_Driver:CP_Touchscreen:uart switch sysfs_create_group fail\r\n");
		status = -ENODEV;
	}

	return status;
}

static int __init touchscreen_sysfs_init(void)
{
	int	status = 0;
	touchscreen_ops[0]=NULL;
	touchscreen_ops[1]=NULL;
	status = class_register(&touchscreen_class);
	if (status < 0)
	{
		printk(KERN_ERR"BJ_BSP_Driver:CP_Touchscreen:uart switch class_register fail\r\n");
		return status;
	}

	status = touchscreen_export();

	return status;
}

arch_initcall(touchscreen_sysfs_init);
