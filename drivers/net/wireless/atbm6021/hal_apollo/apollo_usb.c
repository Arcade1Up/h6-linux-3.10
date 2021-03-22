/*
 * Mac80211 USB driver for altobeam APOLLO device
 * *
 * Copyright (c) 2016, altobeam
 * Author:
 *
 * Based on apollo code Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 #define DEBUG 1
#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <net/atbm_mac80211.h>
#include <linux/kthread.h>
#include <linux/usb.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include "hwio.h"
#include "apollo.h"
#include "sbus.h"
#include "apollo_plat.h"
#include "debug.h"
#include "bh.h"
#include "svn_version.h"

#define DBG_EVENT_LOG
#include "dbg_event.h"
MODULE_DESCRIPTION("mac80211 altobeam apollo wifi USB driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("atbm_wlan");
#define WSM_TX_SKB 1

#define ATBM_USB_EP0_MAX_SIZE 64
#define ATBM_USB_EP1_MAX_RX_SIZE 512
#define ATBM_USB_EP2_MAX_TX_SIZE 512

#define ATBM_USB_VENQT_WRITE  0x40
#define ATBM_USB_VENQT_READ 0xc0

#define usb_printk(...)
/*usb vendor define type, EP0, bRequest*/
enum {
	VENDOR_HW_READ=0,
	VENDOR_HW_WRITE=1,
	VENDOR_HW_RESVER=2,
	VENDOR_SW_CPU_JUMP=3,/*cpu jump to real lmac code,after fw download*/
	VENDOR_SW_READ=4,
	VENDOR_SW_WRITE=5,	
	VENDOR_DBG_SWITCH=6,
	VENDOR_EP0_CMD=7,
};
 int atbm_usb_pm(struct sbus_priv *self, bool  auto_suspend);
 
 int atbm_usb_pm_async(struct sbus_priv *self, bool  auto_suspend);
 void atbm_usb_urb_put(struct sbus_priv *self,unsigned long *bitmap,int id,int tx);
 int atbm_usb_urb_get(struct sbus_priv *self,unsigned long *bitmap,int max_urb,int tx);
 static void atbm_usb_lock(struct sbus_priv *self);
 static void atbm_usb_unlock(struct sbus_priv *self);
 
 extern int atbm_bh_schedule_rx(struct atbm_common *hw_priv);
 extern int atbm_bh_schedule_tx(struct atbm_common *hw_priv);
 
 
#ifdef CONFIG_NFRAC_40M
#define DPLL_CLOCK 40
#elif defined (CONFIG_NFRAC_26M)
#define DPLL_CLOCK 26
#endif
struct build_info{
	int ver;
	int dpll;
	char driver_info[64];
};
typedef int (*sbus_complete_handler)(struct urb *urb);
static int atbm_usb_receive_data(struct sbus_priv *self,unsigned int addr,void *dst, int count,sbus_callback_handler hander);
static int atbm_usb_xmit_data(struct sbus_priv *self,unsigned int addr,const void *pdata, int len,sbus_callback_handler hander);

#if 0
const char DRIVER_INFO[]={"[===USB-APOLLO=="__DATE__" "__TIME__"""=====]"};
#else
const char DRIVER_INFO[]={"[===USB-APOLLO=="};

#endif
static int driver_build_info(void)
{
	struct build_info build;
	build.ver=DRIVER_VER;
	build.dpll=DPLL_CLOCK;
	memcpy(build.driver_info,(void*)DRIVER_INFO,sizeof(DRIVER_INFO));
	printk("SVN_VER=%d,DPLL_CLOCK=%d,BUILD_TIME=%s\n",build.ver,build.dpll,build.driver_info);
	return 0;
}
int G_tx_urb=0;
int G_rx_urb;
static int wifi_module_exit =0;
static int wifi_tx_urb_pending =0;
#define TEST_URB_NUM 5
#define TX_URB_NUM 32
#define RX_URB_NUM 16

struct sbus_urb {
	struct sbus_priv* obj;
	struct urb *test_urb;
	struct sk_buff *test_skb;
	sbus_callback_handler	callback_handler;
	int urb_id;
	int test_pending;
	int test_seq;
	int test_hwChanId;
#ifdef ATBM_NEW_USB_AGGR_TX
	dma_addr_t dma_transfer_addr;	/* (in) dma addr for transfer_buffer */
	
	int pallocated_buf_len;
	u8 *pallocated_buf;
#endif
};
struct dvobj_priv{
	struct usb_device *pusbdev;
	struct usb_interface *pusbintf;
	//struct sk_buff *rx_skb;
	struct sbus_urb rx_urb[RX_URB_NUM];
	struct sbus_urb tx_urb[TX_URB_NUM];
	unsigned long	 rx_urb_map[BITS_TO_LONGS(RX_URB_NUM)];
	unsigned long	 tx_urb_map[BITS_TO_LONGS(TX_URB_NUM)];
	int tx_test_seq_need; //just fot test
	int tx_test_hwChanId_need;//just fot test
	struct urb *cmd_urb;
	struct usb_anchor submitted;
	struct sbus_priv *self;
	struct net_device *netdev;
	u8	usb_speed; // 1.1, 2.0 or 3.0
	u8	nr_endpoint;
	int ep_in;
	int ep_in_size;
	int ep_out;
	int ep_out_size;
	int	ep_num[6]; //endpoint number
	struct sk_buff *suspend_skb;
	unsigned long suspend_skb_len;
#ifdef ATBM_NEW_USB_AGGR_TX
	struct sbus_urb *tx_save_urb;
	int tx_save_urb_data_len;
#endif
};
struct sbus_priv {
	struct dvobj_priv *drvobj;
	struct atbm_common	*core;
	struct atbm_platform_data *pdata;
	//struct sk_buff *rx_skb;
	//struct sk_buff *tx_skb;
	void 			*tx_data;
	int   			tx_vif_selected;
	unsigned int   	tx_hwChanId;
	unsigned int   	rx_seqnum;
	atomic_t 			rx_lock;
	atomic_t 			tx_lock;
	//sbus_callback_handler	tx_callback_handler;
	//sbus_callback_handler	rx_callback_handler;
	spinlock_t		lock;
	struct mutex 	sbus_mutex;
	int 			auto_suspend;
	int 			suspend;
	u8 *usb_data;
	u8 *usb_req_data;
	struct sbus_wtd         * wtd;
};
struct sbus_wtd {
	int 	wtd_init;
	struct task_struct		*wtd_thread;
	wait_queue_head_t		wtd_evt_wq;
	atomic_t				wtd_term;
	atomic_t				wtd_run;
	atomic_t				wtd_probe;
};

static const struct usb_device_id atbm_usb_ids[] = {
	/* Asus */
	{USB_DEVICE(0x0906, 0x5678)},
	{USB_DEVICE(0x007a, 0x8888)},
	{ /* end: all zeroes */}
};

MODULE_DEVICE_TABLE(usb, atbm_usb_ids);

static int  atbm_usb_init(void);
static void  atbm_usb_exit(void);
static struct sbus_wtd         g_wtd={
	.wtd_init  = 0,
	.wtd_thread = NULL,
};

struct dvobj_priv *g_dvobj= NULL;

#ifdef ATBM_NEW_USB_AGGR_TX

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
#define atbm_usb_buffer_alloc(dev, size, dma) usb_alloc_coherent((dev), (size), (in_interrupt() ? GFP_ATOMIC : GFP_KERNEL), (dma))
#define atbm_usb_buffer_free(dev, size, addr, dma) usb_free_coherent((dev), (size), (addr), (dma))
#else
#define atbm_usb_buffer_alloc(dev, size, dma) usb_buffer_alloc((dev), (size), (in_interrupt() ? GFP_ATOMIC : GFP_KERNEL), (dma))
#define atbm_usb_buffer_free(dev, size, addr, dma) usb_buffer_free((dev), (size), (addr), (dma))
#endif
#endif
/* sbus_ops implemetation */
static int atbm_usbctrl_vendorreq_sync(struct sbus_priv *self, u8 request,u8 b_write,
					u16 value, u16 index, void *pdata,u16 len)
{
	unsigned int pipe;
	int status;
	u8 reqtype;
	int vendorreq_times = 0;
	struct usb_device *udev = self->drvobj->pusbdev;
	static int count;
	u8 * reqdata=self->usb_req_data;


	//printk("atbm_usbctrl_vendorreq_sync++ reqtype=%d\n",reqtype);
	if (!reqdata){
		printk("regdata is Null\n");
	}
	if(len > ATBM_USB_EP0_MAX_SIZE){
		atbm_dbg(ATBM_APOLLO_DBG_MSG,"usbctrl_vendorreq request 0x%x, b_write %d! len:%d >%d too long \n",
		       request, b_write, len,ATBM_USB_EP0_MAX_SIZE);
		return -1;
	}
	if(b_write){
		pipe = usb_sndctrlpipe(udev, 0); /* write_out */
		reqtype =  ATBM_USB_VENQT_WRITE;//host to device
		// reqdata must used dma data
		memcpy(reqdata,pdata,len);
	}
	else {
		pipe = usb_rcvctrlpipe(udev, 0); /* read_in */
		reqtype =  ATBM_USB_VENQT_READ;//device to host
	}
	do {
		status = usb_control_msg(udev, pipe, request, reqtype, value,
						 index, reqdata, len, 500); /*500 ms. timeout*/
		if (status < 0) {
			printk(KERN_ERR "%s:err(%d)addr[%x] len[%d],b_write %d request %d\n",__func__,status,value|(index<<16),len,b_write, request);
		} else if(status != len) {
			printk(KERN_ERR "%s:len err(%d)\n",__func__,status);
		}
		else{
			break;
		}
	} while (++vendorreq_times < 3);

	if((b_write==0) && (status>0)){
		memcpy(pdata,reqdata,len);
	}
	if (status < 0 && count++ < 4)
		atbm_dbg(ATBM_APOLLO_DBG_MSG,"reg 0x%x, usbctrl_vendorreq TimeOut! status:0x%x value=0x%x\n",
		       value, status, *(u32 *)pdata);
	return status;
}
#ifdef HW_DOWN_FW
static int atbm_usb_hw_read_port(struct sbus_priv *self, u32 addr, void *pdata,int len)
{
	int ret = 0;
	u8 request = VENDOR_HW_READ; //HW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	//printk("ERR,read addr %x,len %x\n",addr,len);

	//hardware just support len=4
	WARN_ON((len != 4) && (request== VENDOR_HW_READ));
	ret = atbm_usbctrl_vendorreq_sync(self,request,0,wvalue, index, pdata,len);
	if (ret < 0)
	{
		printk("ERR read addr %x,len %x\n",addr,len);
	}
	return ret;
}

static int atbm_usb_hw_write_port(struct sbus_priv *self, u32 addr, const void *pdata,int len)
{

	u8 request = VENDOR_HW_WRITE; //HW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;
	int ret =0;

	atbm_usb_pm(self,0);

	//printk(KERN_ERR "%s:addr(%x)\n",__func__,addr);
	//hardware just support len=4
	//WARN_ON((len != 4) && (request== VENDOR_HW_WRITE));
	ret =  atbm_usbctrl_vendorreq_sync(self,request,1,wvalue, index, (void *)pdata,len);
	if (ret < 0)
	{
		printk("ERR write addr %x,len %x\n",addr,len);
	}
	atbm_usb_pm(self,1);
	return ret;
}
int atbm_usb_sw_read_port(struct sbus_priv *self, u32 addr, void *pdata,int len)
{
	u8 request = VENDOR_SW_READ; //SW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	WARN_ON(len > ATBM_USB_EP0_MAX_SIZE);
	return atbm_usbctrl_vendorreq_sync(self,request,0,wvalue, index, pdata,len);
}

#else
static int atbm_usb_sw_read_port(struct sbus_priv *self, u32 addr, void *pdata,int len)
{
	u8 request = VENDOR_SW_READ; //SW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	WARN_ON(len > ATBM_USB_EP0_MAX_SIZE);
	return atbm_usbctrl_vendorreq_sync(self,request,0,wvalue, index, pdata,len);
}
//#ifndef HW_DOWN_FW
static int atbm_usb_sw_write_port(struct sbus_priv *self, u32 addr,const void *pdata,int len)
{
	u8 request = VENDOR_SW_WRITE; //SW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	WARN_ON(len > ATBM_USB_EP0_MAX_SIZE);
	return atbm_usbctrl_vendorreq_sync(self,request,1,wvalue, index, (void *)pdata,len);
}
#endif
int atbm_lmac_start(struct sbus_priv *self)
{
	u8 request = VENDOR_SW_CPU_JUMP;
	static int tmpdata =0;
	return atbm_usbctrl_vendorreq_sync(self,request,1,0, 0, &tmpdata,0);
}

int atbm_usb_ep0_cmd(struct sbus_priv *self)
{
	u8 request = VENDOR_EP0_CMD; //SW
	
	static int tmpdata =0;
	return atbm_usbctrl_vendorreq_sync(self,request,1,0, 0, &tmpdata,0);
}
/*
wvalue=1 : open uart debug;
 wvalue=0 : close uart debug;
 */
int atbm_usb_debug_config(struct sbus_priv *self,u16 wvalue)
{
	u8 request = 0x6;
	u16 index = 0;

	usb_printk(KERN_DEBUG "atbm_usb_debug_config\n");

	return atbm_usbctrl_vendorreq_sync(self,request,1,wvalue, index, &wvalue,0);
}

static void atbm_usb_xmit_data_complete(struct urb *urb)
{
	struct sbus_urb *tx_urb=(struct sbus_urb*)urb->context;
	struct sbus_priv *self = tx_urb->obj;
	struct atbm_common	*hw_priv	= self->core;

	switch(urb->status){
		case 0:
			break;
		case -ENOENT:
		case -ECONNRESET:
		case -ENODEV:
		case -ESHUTDOWN:
			printk("WARNING>%s %d status %d\n",__func__,__LINE__,urb->status);
			goto __free;
		default:
			printk("WARNING> %s %d status %d\n",__func__,__LINE__,urb->status);
			goto resubmit;
	}
resubmit:
	if(hw_priv == NULL)
	{
		printk(KERN_DEBUG "<WARNING> q. hw_priv =0 drop\n");
		return;
	}
	if(!self->core->init_done){
		printk(KERN_DEBUG "[BH] irq. init_done =0 drop\n");
		return ;
	}
	if (/* WARN_ON */(self->core->bh_error)){
		printk(KERN_DEBUG "[BH] irq. bh_error =0 drop\n");
		return ;
	}


#ifdef ATBM_NEW_USB_AGGR_TX	
	//if((tx_urb->test_seq!= self->drvobj->tx_test_seq_need)
	//	||(tx_urb->test_hwChanId != self->drvobj->tx_test_hwChanId_need))
	//{
	//	printk(" test_seq %d tx_test_seq_need %d\n",tx_urb->test_seq, self->drvobj->tx_test_seq_need);
	//	printk(" test_hwChanId %d tx_test_hwChanId_need %d\n",tx_urb->test_hwChanId, self->drvobj->tx_test_hwChanId_need);
	//	WARN_ON(1);
	//}
	//self->drvobj->tx_test_seq_need = (tx_urb->test_seq+1) & WSM_TX_SEQ_MAX;
	//self->drvobj->tx_test_hwChanId_need = (tx_urb->test_hwChanId+1) ;
	//usb_printk( "%s handler %p\n",__func__,tx_urb->callback_handler);

	//if(tx_urb->callback_handler){
	//	atbm_usb_urb_put(self,self->drvobj->tx_urb_map,tx_urb->urb_id);
	//	printk("tx: callback_handler atbm_usb_urb_put %d\n",tx_urb->urb_id);
	//	tx_urb->callback_handler(hw_priv,NULL);
	//	return ;
	//}
	//printk("txcmp %d\n",tx_urb->test_hwChanId);
#else
	if((tx_urb->test_seq!= self->drvobj->tx_test_seq_need)
		||(tx_urb->test_hwChanId != self->drvobj->tx_test_hwChanId_need))
	{
		printk(" test_seq %d tx_test_seq_need %d\n",tx_urb->test_seq, self->drvobj->tx_test_seq_need);
		printk(" test_hwChanId %d tx_test_hwChanId_need %d\n",tx_urb->test_hwChanId, self->drvobj->tx_test_hwChanId_need);
		WARN_ON(1);
	}
	self->drvobj->tx_test_seq_need = (tx_urb->test_seq+1) & WSM_TX_SEQ_MAX;
	self->drvobj->tx_test_hwChanId_need = (tx_urb->test_hwChanId+1) ;
	//usb_printk( "%s handler %p\n",__func__,tx_urb->callback_handler);

	//if(tx_urb->callback_handler){
	//	atbm_usb_urb_put(self,self->drvobj->tx_urb_map,tx_urb->urb_id);
	//	printk("tx: callback_handler atbm_usb_urb_put %d\n",tx_urb->urb_id);
	//	tx_urb->callback_handler(hw_priv,NULL);
	//	return ;
	//}
#endif
	atbm_usb_urb_put(self,self->drvobj->tx_urb_map,tx_urb->urb_id,1);

	tx_urb->test_pending = 0;

	atbm_bh_schedule_tx(hw_priv);
	
	return ;

__free:
	printk("<Warning> usb drop 1 frame txend:_urb_put %d\n",tx_urb->urb_id);
	wsm_release_tx_buffer(hw_priv, 1);
	
	atbm_usb_urb_put(self,self->drvobj->tx_urb_map,tx_urb->urb_id,1);
	tx_urb->test_pending = 0;

	atbm_bh_schedule_tx(hw_priv);

	return ;
}

#ifdef ATBM_NEW_USB_AGGR_TX	
static int atbm_usb_xmit_data(struct sbus_priv *self,
				   unsigned int addr,
				   const void *pdata, int len,sbus_callback_handler hander)
{
	unsigned int pipe;
	int status=0;
	int tx_burst=0;
	int vif_selected;
	struct wsm_hdr_tx *wsm;
	struct atbm_common *hw_priv=self->core;
	u8 *txdata =NULL;
	u8 *data =NULL;
	int tx_len=0;
	int tx_len_offset=0;
	int txdata_offset = 0;
	int ret = 0;
	int urb_id =-1;
	int is_cmd =0;
	struct sbus_urb *tx_urb = NULL;
	unsigned long flags=0;

	if(self->drvobj->tx_save_urb == NULL){
		
		urb_id = atbm_usb_urb_get(self,self->drvobj->tx_urb_map,TX_URB_NUM,1);
		if(urb_id<0){
			usb_printk(KERN_DEBUG "atbm_usb_xmit_data:urb_id<0\n");
			status=-4;
			goto error;
		}
		if (hw_priv->device_can_sleep) {
				hw_priv->device_can_sleep = false;
		}
		tx_urb = &self->drvobj->tx_urb[urb_id];
		if(tx_urb->test_skb ==0){			
				status=-1;
				printk(KERN_ERR,"atbm_usb_tx_data++ dev_alloc_skb %p ERROR\n",tx_urb->test_skb);
				goto error;
		}
				
		txdata =(u8 *) tx_urb->test_skb->data;

		tx_len_offset = 0;
		do{
			wsm_alloc_tx_buffer(hw_priv);
			ret = wsm_get_tx(hw_priv, &data, &tx_len, &tx_burst,
						&vif_selected);
			if (ret <= 0) {
				 wsm_release_tx_buffer(hw_priv, 1);
				 if(txdata_offset == 0){
				  	status=-3;
				    atbm_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
				  	goto error;
				 }
				 else {
					txdata_offset -= tx_len_offset ;
					break;
				 }
			} else {
				wsm = (struct wsm_hdr_tx *)data;
				tx_len=wsm->usb_len;
				BUG_ON(tx_len < sizeof(*wsm));
				BUG_ON(__le32_to_cpu(wsm->usb_len) != tx_len);
				atomic_add(1, &hw_priv->bh_tx);
				self->tx_vif_selected =vif_selected;
				//WSM_FIRMWARE_CHECK_ID have no confirm
				if(wsm->id == WSM_FIRMWARE_CHECK_ID){
					 wsm_release_tx_buffer(hw_priv, 1);
				}

				wsm->flag = __cpu_to_le16(0xe569)<<16;
				wsm->flag |= BIT(6);
				wsm->flag |= (self->tx_hwChanId & 0x1f);
				
				wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
				wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq));
				//txdata =wsm;
				memcpy(txdata+txdata_offset,(u8 *)wsm,tx_len);

				spin_lock_bh(&hw_priv->tx_com_lock);
				if(wsm_txed(hw_priv, data)==0){
					hw_priv->wsm_txframe_num++;
				}
				else {
					is_cmd = 1;
				}
				
				if (vif_selected != -1) {
					hw_priv->hw_bufs_used_vif[vif_selected]++;
				}
				self->tx_hwChanId++;
				hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;			
				spin_unlock_bh(&hw_priv->tx_com_lock);
				tx_len_offset =  ALIGN(tx_len,USB_BLOCK_SIZE) - tx_len; //2048-1538=510
				txdata_offset += ALIGN(tx_len,USB_BLOCK_SIZE);//2048
				if((txdata_offset + 2048 >TX_LONG_BUFFER_SIZE)||
					is_cmd)
 				{
					txdata_offset -= tx_len_offset;
					break;
				}
			}
		}while(1);
			
		tx_urb->callback_handler = hander;
		tx_urb->test_pending = 1;
		
		tx_urb->test_seq= hw_priv->wsm_tx_seq;
		tx_urb->test_hwChanId= self->tx_hwChanId;
		
	}
	else {
		tx_urb = self->drvobj->tx_save_urb;
		txdata_offset = self->drvobj->tx_save_urb_data_len;
	}
	
	{
		
		txdata_offset = ALIGN(txdata_offset,4);
		printk( KERN_DEBUG "txoffset = %d _seqnum %d  %x\n",txdata_offset,tx_urb->test_hwChanId,txdata[0]);
		
		spin_lock_irqsave(&self->lock, flags);
		
		pipe = usb_sndbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_out);
		///self->drvobj->tx_urb->transfer_flags |= URB_ZERO_PACKET;
		usb_fill_bulk_urb(tx_urb->test_urb,
			self->drvobj->pusbdev, pipe,txdata,txdata_offset,
			atbm_usb_xmit_data_complete,tx_urb);
		
		//usb_anchor_urb(self->drvobj->tx_urb,
		status = usb_submit_urb(tx_urb->test_urb, GFP_ATOMIC);
		spin_unlock_irqrestore(&self->lock, flags); 
		if (status) {
			printk("<WARNING>%s %d status %d\n",__func__,__LINE__,status);
			status = -5;
			
			spin_lock_bh(&hw_priv->tx_com_lock);
			//hw_priv->save_buf = data;
			//hw_priv->save_buf_len = tx_len;
			//hw_priv->save_buf_vif_selected = vif_selected;
			
			//if (vif_selected != -1) {
			//	hw_priv->hw_bufs_used_vif[vif_selected]--;
			//}
			self->drvobj->tx_save_urb=tx_urb;
			self->drvobj->tx_save_urb_data_len=txdata_offset;
			
			spin_unlock_bh(&hw_priv->tx_com_lock);	
			//usb_kill_urb(tx_urb->test_urb);
			//atomic_xchg(&self->tx_lock, 0);
			//wsm_release_tx_buffer(hw_priv, 1);
			//atbm_usb_urb_put_tx(self,self->drvobj->tx_urb_map,urb_id);
			//printk("tx:atbm_usb_urb_put %d\n",urb_id);
			//msleep(1000);
			goto error;
		}
		spin_lock_bh(&hw_priv->tx_com_lock);
		//self->tx_data = NULL;
		usb_printk(KERN_DEBUG "tx_seq %d\n",self->tx_hwChanId);
		//hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;
		spin_unlock_bh(&hw_priv->tx_com_lock);
	
	}
	if(status==0){
		return tx_burst;
	}
error:
	return status;
}
#else
static int atbm_usb_xmit_data(struct sbus_priv *self,
				   unsigned int addr,
				   const void *pdata, int len,sbus_callback_handler hander)
{
	unsigned int pipe;
	int status=0;
	int tx_burst=0;
	int vif_selected;
	struct wsm_hdr_tx *wsm;
	struct atbm_common *hw_priv=self->core;
	void *txdata =NULL;
	u8 *data =NULL;
	int tx_len=0;
	int ret = 0;
	int urb_id =-1;
	struct sbus_urb *tx_urb = NULL;

	if(wifi_module_exit){
		printk("atbm_usb_xmit_data drop urb req because rmmod driver\n");
		status=-9;
		goto error;
	}

	urb_id = atbm_usb_urb_get(self,self->drvobj->tx_urb_map,TX_URB_NUM,1);
	if(urb_id<0){
		usb_printk(KERN_DEBUG "atbm_usb_xmit_data:urb_id<0\n");
		status=-4;
		goto error;
	}

	/*if (atomic_read(&self->tx_lock)==0)*/{
		if (hw_priv->device_can_sleep) {
				hw_priv->device_can_sleep = false;
		}

		wsm_alloc_tx_buffer(hw_priv);
		ret = wsm_get_tx(hw_priv, &data, &tx_len, &tx_burst,
					&vif_selected);
		if (ret <= 0) {
//			  printk(KERN_ERR "%s __LINE__ %d wsm_get_tx null\n",__func__,__LINE__);
			  wsm_release_tx_buffer(hw_priv, 1);
			  atbm_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
			 // printk("tx:atbm_usb_urb_put NULL %d\n",urb_id);
			  status=-3;
			  goto error;
		} else {
			tx_urb = &self->drvobj->tx_urb[urb_id];
			wsm = (struct wsm_hdr_tx *)data;
			tx_len=wsm->usb_len;
			BUG_ON(tx_len < sizeof(*wsm));

			BUG_ON(__le32_to_cpu(wsm->usb_len) != tx_len);

			atomic_add(1, &hw_priv->bh_tx);
			//self->tx_data = (void *)data;
			self->tx_vif_selected =vif_selected;
			//WSM_FIRMWARE_CHECK_ID have no confirm
			if(wsm->id == WSM_FIRMWARE_CHECK_ID){
				 wsm_release_tx_buffer(hw_priv, 1);
			}

			wsm->flag = __cpu_to_le16(0xe569)<<16;
			wsm->flag |= BIT(6);
			wsm->flag |= (self->tx_hwChanId & 0x1f);
			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq));
			txdata =wsm;
			//usb_printk( "tx hw_bufs_used_vif[%d]=%d\n",vif_selected,hw_priv->hw_bufs_used_vif[vif_selected]);
			//tx_len =ALIGN(tx_len,USB_BLOCK_SIZE);
			usb_printk( "%s flag=%x, tx_len %d seq %d len %d\n",__func__,wsm->flag,tx_len,hw_priv->wsm_tx_seq,wsm->len);

			//atomic_xchg(&self->tx_lock, 1);
			tx_urb->callback_handler = hander;
			tx_urb->test_pending = 1;
			tx_urb->test_seq= hw_priv->wsm_tx_seq;
			tx_urb->test_hwChanId= self->tx_hwChanId;
			//printk(KERN_DEBUG "txstart:urb_get %d seq %d\n",urb_id,hw_priv->wsm_tx_seq);
			spin_lock_bh(&hw_priv->tx_com_lock);
			if(wsm_txed(hw_priv, data)==0){
				hw_priv->wsm_txframe_num++;
			}
			
			if (vif_selected != -1) {
				hw_priv->hw_bufs_used_vif[vif_selected]++;
			}
			spin_unlock_bh(&hw_priv->tx_com_lock);

			pipe = usb_sndbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_out);
			///self->drvobj->tx_urb->transfer_flags |= URB_ZERO_PACKET;
			usb_fill_bulk_urb(tx_urb->test_urb,
				self->drvobj->pusbdev, pipe,txdata,tx_len,
				atbm_usb_xmit_data_complete,tx_urb);
			//usb_anchor_urb(self->drvobj->tx_urb,
			status = usb_submit_urb(tx_urb->test_urb, GFP_ATOMIC);
			if (status) {
				status = -5;
				
				spin_lock_bh(&hw_priv->tx_com_lock);
				hw_priv->save_buf = data;
				hw_priv->save_buf_len = tx_len;
				hw_priv->save_buf_vif_selected = vif_selected;
				
				if (vif_selected != -1) {
					hw_priv->hw_bufs_used_vif[vif_selected]--;
				}
				spin_unlock_bh(&hw_priv->tx_com_lock);				
				//atomic_xchg(&self->tx_lock, 0);
				printk("<WARNING>%s %d tx_len %d\n",__func__,__LINE__,tx_len);
				wsm_release_tx_buffer(hw_priv, 1);
				atbm_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
				printk("tx:atbm_usb_urb_put %d\n",urb_id);
				//msleep(1000);
				goto error;
			}
			spin_lock_bh(&hw_priv->tx_com_lock);
			//self->tx_data = NULL;
			self->tx_hwChanId++;
			usb_printk(KERN_DEBUG "tx_seq %d\n",self->tx_hwChanId);
			hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;
			spin_unlock_bh(&hw_priv->tx_com_lock);
		}
	}
	if(status==0){
		return tx_burst;
	}
error:
	return status;
}
#endif

void atbm_usb_receive_data_cancel(struct sbus_priv *self)
{
	printk("&&&fuc=%s\n",__func__);
	//usb_kill_urb(self->drvobj->rx_urb);
	atbm_usb_pm(self,1);
}

#ifdef ATBM_NEW_USB_AGGR_TX	
void atbm_usb_urb_free(struct sbus_priv *self,struct sbus_urb * pUrb,int max_num)
{
	int i=0;
	for(i=0;i<max_num;i++){
		usb_kill_urb(pUrb[i].test_urb);
		usb_free_urb(pUrb[i].test_urb);
		if(pUrb[i].test_skb)
			dev_kfree_skb(pUrb[i].test_skb);
		if(pUrb[i].pallocated_buf)
			atbm_usb_buffer_free(self->drvobj->pusbdev,pUrb[i].pallocated_buf_len,pUrb[i].pallocated_buf,pUrb[i].dma_transfer_addr);
		pUrb[i].test_skb =NULL;
		pUrb[i].test_urb =NULL;
		pUrb[i].pallocated_buf = NULL;
		pUrb[i].pallocated_buf_len = 0;
	}
}
#else
void atbm_usb_urb_free(struct sbus_priv *self,struct sbus_urb * pUrb,int max_num)
{
	int i=0;
	for(i=0;i<max_num;i++){
		usb_kill_urb(pUrb[i].test_urb);
		usb_free_urb(pUrb[i].test_urb);
		dev_kfree_skb(pUrb[i].test_skb);
		pUrb[i].test_skb =NULL;
		pUrb[i].test_urb =NULL;
	}
}
#endif
#ifdef ATBM_NEW_USB_AGGR_TX	
int atbm_usb_urb_malloc(struct sbus_priv *self,struct sbus_urb * pUrb,int max_num,int len,int b_skb)
{
	int i=0;
	for(i=0;i<max_num;i++){
		pUrb[i].test_urb=usb_alloc_urb(0,GFP_KERNEL);
		if (!pUrb[i].test_urb){
			atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate test_urb.");
			goto __free_urb;
		}
		if(b_skb){
			pUrb[i].test_skb =dev_alloc_skb(len);
			if (!pUrb[i].test_skb){
				atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate test_skb.");
				goto __free_skb;
			}
			pUrb[i].dma_transfer_addr =0;
			pUrb[i].pallocated_buf  =NULL;
			pUrb[i].pallocated_buf_len = 0;
			printk("pUrb[%d] skb\n",i);
		}
		else {
			pUrb[i].pallocated_buf = atbm_usb_buffer_alloc(self->drvobj->pusbdev, (size_t)len, &pUrb[i].dma_transfer_addr);
			if (!pUrb[i].pallocated_buf)
			{
				atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate pallocated_buf.");
				goto __free_skb;
			}
			pUrb[i].test_skb = NULL;
			pUrb[i].pallocated_buf_len = len;
			printk("pUrb[%d] usb_buffer\n",i);
		}
		pUrb[i].test_pending =0;
		pUrb[i].urb_id = i;
		pUrb[i].obj =self;	 
	}
	return 0;
__free_skb:
	for( ;i>=0;--i){
		dev_kfree_skb(pUrb[i].test_skb);
	}
	i = max_num;
__free_urb:
	for( ;i>=0;--i){
		usb_free_urb(pUrb[i].test_urb);
	}

	return -ENOMEM;
}
#else
int atbm_usb_urb_malloc(struct sbus_priv *self,struct sbus_urb * pUrb,int max_num,int len,int b_skb)
{
	int i=0;
	for(i=0;i<max_num;i++){
		pUrb[i].test_urb=usb_alloc_urb(0,GFP_KERNEL);
		if (!pUrb[i].test_urb){
			atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate test_urb.");
			goto __free_urb;
		}
		pUrb[i].test_skb =dev_alloc_skb(len);
		if (!pUrb[i].test_skb){
			atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate test_skb.");
			goto __free_skb;
		}
		pUrb[i].test_pending =0;
		pUrb[i].urb_id = i;
		pUrb[i].obj =self;
	}
	return 0;
__free_skb:
	for( ;i>=0;--i){
		dev_kfree_skb(pUrb[i].test_skb);
	}
	i = max_num;
__free_urb:
	for( ;i>=0;--i){
		usb_free_urb(pUrb[i].test_urb);
	}

	return -ENOMEM;
}
#endif
int atbm_usb_urb_get(struct sbus_priv *self,unsigned long *bitmap,int max_urb,int tx)
{
	int id = 0;
	unsigned long flags=0;

	spin_lock_irqsave(&self->lock, flags);
	id= find_first_zero_bit(bitmap,max_urb);
	if((id>=max_urb)||(id<0)){
		spin_unlock_irqrestore(&self->lock, flags);
		return -1;
	}
	__set_bit(id,bitmap);
	if(tx)
		wifi_tx_urb_pending++;
	spin_unlock_irqrestore(&self->lock, flags);

	return id;
}
void atbm_usb_urb_put(struct sbus_priv *self,unsigned long *bitmap,int id,int tx)
{
	unsigned long flags=0;

	spin_lock_irqsave(&self->lock, flags);
	if(tx)
		wifi_tx_urb_pending--;
	WARN_ON((*bitmap & BIT(id))==0);
	__clear_bit(id,bitmap);
	spin_unlock_irqrestore(&self->lock, flags);
}

static void atbm_usb_receive_data_complete(struct urb *urb)
{
	struct sbus_urb *rx_urb=(struct sbus_urb*)urb->context;
	struct sbus_priv *self = rx_urb->obj;
	struct sk_buff *skb=rx_urb->test_skb;
	struct atbm_common *hw_priv=self->core;
	int RecvLength=urb->actual_length;
	struct wsm_hdr *wsm;
	unsigned long flags;
	usb_printk(KERN_DEBUG "rxend  Len %d\n",RecvLength);

	if(!hw_priv)
		goto __free;
	if (!skb){
		WARN_ON(1);
		goto __free;
	}	

	switch(urb->status){
		case 0:
			break;
		case -ENOENT:
		case -ECONNRESET:
		case -ENODEV:
		case -ESHUTDOWN:
			printk(KERN_DEBUG "atbm_usb_rx_complete1 error status=%d len %d\n",urb->status,RecvLength);
			goto __free;
		case -EPROTO:
			printk(KERN_DEBUG "atbm_usb_rx_complete3 error status=%d len %d\n",urb->status,RecvLength);
			if(RecvLength !=0){
				break;
			}
		case -EOVERFLOW:
			if(RecvLength)
				break;
			printk(KERN_DEBUG "<ERROR>:atbm_usb_rx_complete status=%d len %d\n",urb->status,RecvLength);
			goto __free;
		default:
			printk(KERN_DEBUG "atbm_usb_rx_complete2 error status=%d len %d\n",urb->status,RecvLength);
			goto resubmit;
	}


	if((self->drvobj->suspend_skb != NULL)&&(self->drvobj->suspend_skb_len != 0))
	{
		if(skb_tailroom(self->drvobj->suspend_skb)<self->drvobj->suspend_skb_len+RecvLength)
		{
			struct sk_buff * long_suspend_skb = NULL;
			BUG_ON(self->drvobj->suspend_skb_len+RecvLength>RX_BUFFER_SIZE);
			printk(KERN_ERR "suspend skb len is not enough(%d),(%ld)\n",
				skb_tailroom(self->drvobj->suspend_skb),self->drvobj->suspend_skb_len+RecvLength);

			long_suspend_skb = dev_alloc_skb(RX_BUFFER_SIZE+64);
			BUG_ON(!long_suspend_skb);
			skb_reserve(long_suspend_skb, 64);
			memcpy((u8 *)long_suspend_skb->data,self->drvobj->suspend_skb->data,self->drvobj->suspend_skb_len);
			dev_kfree_skb(self->drvobj->suspend_skb);
			self->drvobj->suspend_skb = long_suspend_skb;
		}
		memcpy((u8 *)self->drvobj->suspend_skb->data+self->drvobj->suspend_skb_len,skb->data,RecvLength);
		RecvLength += self->drvobj->suspend_skb_len;
		memcpy(skb->data,(u8 *)self->drvobj->suspend_skb->data,RecvLength);
		self->drvobj->suspend_skb_len = 0;					
		printk(KERN_ERR "suspend rebuild\n");
	}
	
	wsm = (struct wsm_hdr *)skb->data;
	
	if (wsm->len != RecvLength){	
		//add because usb not reset when rmmod driver, just drop error frame  
		if(hw_priv->wsm_caps.firmwareReady==0){
			//BUG_ON(1);
			goto resubmit;
		}
	
		if(((wsm->len  % 512)==0) && ((wsm->len+1) == RecvLength)){
			//this correct , lmac output len = (wsm len +1 ) ,inorder to let hmac usb callback
		}
		else {
			printk("rx rebulid usbsuspend  id %d wsm->len %d,RecvLength %d\n",wsm->id,wsm->len,RecvLength);

			if(self->drvobj->suspend_skb_len== 0){ 
				if(wsm->len > RX_BUFFER_SIZE){
					printk(" %s %d id %d wsm->len %d,RecvLength %d\n",__func__,__LINE__,wsm->id,wsm->len,RecvLength);
					goto resubmit;
				}
				printk(KERN_DEBUG "rx rebulid usbsuspend0	\n");
				/*
				* alloc 4K buff for suspend_skb is save
				*/
				BUG_ON(self->drvobj->suspend_skb == NULL);
				memcpy((u8 *)self->drvobj->suspend_skb->data,skb->data,RecvLength);
				self->drvobj->suspend_skb_len = RecvLength;
				goto resubmit;
			}
		}
	}
	
	WARN_ON(self->drvobj->suspend_skb_len != 0);
	
	if (WARN_ON(4 > RecvLength)){
		printk("%s %d id %d wsm->len %d,RecvLength %d\n",__func__,__LINE__,wsm->id,wsm->len,RecvLength);
		frame_hexdump("atbm_usb_receive_data_complete",(u8 *)wsm,32);
		goto resubmit;
		//goto __free;
	}

	BUG_ON(RecvLength > RX_BUFFER_SIZE);

	if(rx_urb->callback_handler){
		rx_urb->callback_handler(hw_priv,wsm);
		return;
	}
	self->rx_seqnum++;

	spin_lock_irqsave(&self->lock, flags);
	rx_urb->test_skb = NULL;
	skb->len = RecvLength;
	skb_queue_tail(&hw_priv->rx_frame_queue, skb);
	spin_unlock_irqrestore(&self->lock, flags);

	if(skb_queue_len(&hw_priv->rx_frame_queue) <=1){
		atbm_bh_schedule_rx(hw_priv);
	}

resubmit:
	if(!hw_priv->init_done){
		printk(KERN_DEBUG "[BH] irq. init_done =0 drop\n");
		goto __free;
	}
	if (/* WARN_ON */(hw_priv->bh_error))
		goto __free;

	atbm_usb_urb_put(self,self->drvobj->rx_urb_map,rx_urb->urb_id,0);
	return;
	
__free:
	if(self->drvobj->suspend_skb_len != 0){
		printk("rx rebulid usbsuspend3	rx drop\n");
	}
	atbm_usb_urb_put(self,self->drvobj->rx_urb_map,rx_urb->urb_id,0);
	printk(KERN_DEBUG "[WARNING] atbm_usb_receive_data drop\n");
	return;

}




static int atbm_usb_receive_data(struct sbus_priv *self,unsigned int addr,void *dst, int count,sbus_callback_handler hander)
{
	unsigned int pipe;
	int status=0;
	struct sk_buff *skb;
	struct sbus_urb *rx_urb;
	int urb_id;

	if(self->suspend == 1){
		return 2;
	}
	if(wifi_module_exit){
		printk("atbm_usb_receive_data drop urb req because rmmod driver\n");
		return 3;
	}

	urb_id = atbm_usb_urb_get(self,self->drvobj->rx_urb_map,RX_URB_NUM,0);
	if(urb_id<0){
		status=-4;
		return 1;
	}
	G_rx_urb=urb_id;
	rx_urb = &self->drvobj->rx_urb[urb_id];
	//if not rxdata complete
	//initial new rxdata
	if(rx_urb->test_skb == NULL){
		if(dst ){

			rx_urb->test_skb=dst;
			skb_trim(rx_urb->test_skb,0);

		}
		else {
			skb=dev_alloc_skb(count+64);
			if (!skb){
				status=-1;
				printk("atbm_usb_receive_data++ dev_alloc_skb %p ERROR\n",skb);
				goto __err_skb;
			}
			skb_reserve(skb, 64);
			rx_urb->test_skb=skb;
		}
	}
	else {
		if(dst ){
		
		dev_kfree_skb(dst);
		
		}

	}
	
	skb = rx_urb->test_skb;
	rx_urb->callback_handler = hander;
	rx_urb->test_pending = 1;
	//atomic_xchg(&self->rx_lock, 1);

	pipe = usb_rcvbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_in);
	usb_fill_bulk_urb(rx_urb->test_urb, self->drvobj->pusbdev, pipe,skb->data,count,atbm_usb_receive_data_complete,rx_urb);
	status = usb_submit_urb(rx_urb->test_urb, GFP_ATOMIC);
	usb_printk(KERN_DEBUG "usb_rx urb_id %d\n",urb_id);

	if (status) {
		status = -2;
		//atomic_xchg(&self->rx_lock, 0);
		printk("receive_data usb_submit_urb ++ ERR %d\n",status);
		goto __err_skb;
	}

__err_skb:
	if(status < 0){
		atbm_usb_urb_put(self,self->drvobj->rx_urb_map,urb_id,0);
	}
	return status;
}


static int atbm_usb_memcpy_fromio(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count)
{
	int i=0;
	if(atomic_add_return(0, &self->rx_lock)){
		return -1;
	}
	for(i=0;i<RX_URB_NUM;i++){
	 	atbm_usb_receive_data(self,addr,dst,count,NULL);
	}
	return 0;
}

static int atbm_usb_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
	return atbm_usb_xmit_data(self, addr,(void *)src,count,NULL);
}

static int atbm_usb_memcpy_fromio_async(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count,sbus_callback_handler func)
{
	 int ret = 0;

	//  self->rx_callback_handler = func;

	// if(atomic_add_return(0, &self->rx_lock)){
	//	 return -1;
	 //}

	 atbm_usb_receive_data(self,addr,dst,count,func);

	 return ret;
}

static int atbm_usb_memcpy_toio_async(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count ,sbus_callback_handler func)
{
	 int ret =  0;
	 atbm_usb_xmit_data(self, addr,(void *)src, count,func);

	 return ret;
}

static void atbm_usb_lock(struct sbus_priv *self)
{
	//mutex_lock(&self->sbus_mutex);
	atbm_usb_pm_async(self,0);

}

static void atbm_usb_unlock(struct sbus_priv *self)
{
	//mutex_unlock(&self->sbus_mutex);
	atbm_usb_pm_async(self,1);
}

static int atbm_usb_off(const struct atbm_platform_data *pdata)
{
	int ret = 0;

	//if (pdata->insert_ctrl)
	//	ret = pdata->insert_ctrl(pdata, false);
	return ret;
}

static int atbm_usb_on(const struct atbm_platform_data *pdata)
{
	int ret = 0;

   // if (pdata->insert_ctrl)
	//	ret = pdata->insert_ctrl(pdata, true);

	return ret;
}


static int atbm_usb_reset(struct sbus_priv *self)
{
//	u32 regdata = 1;
	printk(" %s\n",__func__);
	usb_reset_device(interface_to_usbdev(self->drvobj->pusbintf));
	//#ifdef HW_DOWN_FW
	//atbm_usb_hw_write_port(self,0x16100074,&regdata,4);
	//#else
	//atbm_usb_sw_write_port(self,0x16100074,&regdata,4);
	//#endif
	return 0;
}

static u32 atbm_usb_align_size(struct sbus_priv *self, u32 size)
{
	size_t aligned = size;
	return aligned;
}

int atbm_usb_set_block_size(struct sbus_priv *self, u32 size)
{
	return 0;
}
#ifdef CONFIG_PM
int atbm_usb_pm(struct sbus_priv *self, bool  auto_suspend)
{
	int ret = 0;

	//printk("atbm_usb_pm %d -> %d\n",self->auto_suspend ,auto_suspend);
	//if(self->auto_suspend == auto_suspend){
		//printk("***********func=%s,usage_count=%d\n",__func__,self->drvobj->pusbdev->dev.power.usage_count);
	//	return;
	//}
	self->auto_suspend  = auto_suspend;
#if USB_AUTO_WAKEUP
	if(auto_suspend){
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
			usb_autopm_put_interface(self->drvobj->pusbintf);
#elif (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,20))
			usb_autopm_enable(self->drvobj->pusbintf);
#else
			usb_autosuspend_device(self->drvobj->pusbdev, 1);
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
	}
	else { //if(auto_suspend)
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
		usb_autopm_get_interface(self->drvobj->pusbintf);
#elif (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,20))
			usb_autopm_disable(self->drvobj->pusbintf);
#else //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
			usb_autoresume_device(self->drvobj->pusbdev, 1);
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
	}
#endif  //#if USB_AUTO_WAKEUP
	//printk("***********func=%s,usage_count=%d\n",__func__,self->drvobj->pusbdev->dev.power.usage_count);
	return ret;
}
int atbm_usb_pm_async(struct sbus_priv *self, bool  auto_suspend)
{
	int ret = 0;

	self->auto_suspend  = auto_suspend;
	
#if USB_AUTO_WAKEUP
	if(auto_suspend){
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
		usb_autopm_put_interface_async(self->drvobj->pusbintf);
#else
#endif
	}
	else {
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
		usb_autopm_get_interface_async(self->drvobj->pusbintf);
#else
#endif
	}
#endif  //#if USB_AUTO_WAKEUP

	return ret;
}
#else
int atbm_usb_pm(struct sbus_priv *self, bool  auto_suspend)
{
	return 0;
}
int atbm_usb_pm_async(struct sbus_priv *self, bool  auto_suspend)
{

	return 0;
}
#endif
static int atbm_usb_irq_subscribe(struct sbus_priv *self, sbus_irq_handler handler,void *priv)
{
	int ret = 0;
	return ret;
}
static int atbm_usb_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;
	return ret;
}

void atbm_wtd_wakeup( struct sbus_priv *self)
{
#ifdef CONFIG_ATBMWIFI_WDT
	if(atomic_read(&self->wtd->wtd_term))
		return;
	atomic_set(&g_wtd.wtd_run, 1);
	printk( "[atbm_wtd] wakeup.\n");
	wake_up(&self->wtd->wtd_evt_wq);
#endif //CONFIG_ATBMWIFI_WDT
}
#ifdef RESET_CHANGE
extern int atbm_reset_driver(struct atbm_common *hw_priv);
#endif
extern struct atbm_common *g_hw_priv;
static int atbm_wtd_process(void *arg)
{
#ifdef CONFIG_ATBMWIFI_WDT
	int status=0;
	int term=0;
	int wtd_run=0;
	int waittime = 20;
	int wtd_probe=0;
#ifdef RESET_CHANGE
	int err;
#endif
	while(1){
		status = wait_event_interruptible(g_wtd.wtd_evt_wq, ({
				term = atomic_read(&g_wtd.wtd_term);
				wtd_run = atomic_read(&g_wtd.wtd_run);
				(term || wtd_run);}));
		if (status < 0 || term ){
			printk("[atbm_wtd]:1 thread break %d %d\n",status,term);
			goto __stop;
		}
		atomic_set(&g_wtd.wtd_run, 0);
		while(waittime-- >0){
			msleep(1);
			term = atomic_read(&g_wtd.wtd_term);
			if(term) {
				printk("[atbm_wtd]:2 thread break %d %d\n",status,term);
				goto __stop;
			}
			wtd_probe = atomic_read(&g_wtd.wtd_probe);
			if(wtd_probe != 0){
				printk("[atbm_wtd]:wtd_probe(%d) have done\n",wtd_probe);
				break;
			}
		}
		waittime = 10;
		//check if sdio init ok?
		wtd_probe = atomic_read(&g_wtd.wtd_probe);
		//if sdio init have probem, need call wtd again
		if(wtd_probe != 1){
			atomic_set(&g_wtd.wtd_run, 1);
			printk("[atbm_wtd]:wtd_run again\n");
		}
	}
__stop:
	while(term){
		
		printk("[atbm_wtd]:kthread_should_stop\n");
		if(kthread_should_stop()){
			break;
		}
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
	}
#endif //CONFIG_ATBMWIFI_WDT
	return 0;
}
static void atbm_wtd_init(void)
{
#ifdef CONFIG_ATBMWIFI_WDT
	int err = 0;
	struct sched_param param = { .sched_priority = 1 };
	if(g_wtd.wtd_init)
		return;
	printk( "[wtd] register.\n");
	init_waitqueue_head(&g_wtd.wtd_evt_wq);
	atomic_set(&g_wtd.wtd_term, 0);
	g_wtd.wtd_thread = kthread_create(&atbm_wtd_process, &g_wtd, "atbm_wtd");
	if (IS_ERR(g_wtd.wtd_thread)) {
		err = PTR_ERR(g_wtd.wtd_thread);
		g_wtd.wtd_thread = NULL;
	} else {
	sched_setscheduler(g_wtd.wtd_thread,
			SCHED_FIFO, &param);
#ifdef HAS_PUT_TASK_STRUCT
		get_task_struct(g_wtd.wtd_thread);
#endif
		wake_up_process(g_wtd.wtd_thread);
	}
	g_wtd.wtd_init = 1;
#endif //CONFIG_ATBMWIFI_WDT
}
static void atbm_wtd_exit(void)
{
#ifdef CONFIG_ATBMWIFI_WDT
	struct task_struct *thread = g_wtd.wtd_thread;
	if (WARN_ON(!thread))
		return;
	if(atomic_read(&g_wtd.wtd_term)==0)
		return;
	g_wtd.wtd_thread = NULL;
	printk( "[wtd] unregister.\n");
	atomic_add(1, &g_wtd.wtd_term);
	wake_up(&g_wtd.wtd_evt_wq);
	kthread_stop(thread);
#ifdef HAS_PUT_TASK_STRUCT
	put_task_struct(thread);
#endif
	g_wtd.wtd_init = 0;
#endif //CONFIG_ATBMWIFI_WDT
}

static struct sbus_ops atbm_usb_sbus_ops = {
	.sbus_memcpy_fromio	= atbm_usb_memcpy_fromio,
	.sbus_memcpy_toio	= atbm_usb_memcpy_toio,
#ifdef HW_DOWN_FW
	.sbus_read_sync		= atbm_usb_hw_read_port,
	.sbus_write_sync	= atbm_usb_hw_write_port,
#else //SW
	.sbus_read_sync		= atbm_usb_sw_read_port,
	.sbus_write_sync	= atbm_usb_sw_write_port,
#endif
	.sbus_read_async	= atbm_usb_memcpy_fromio_async,
	.sbus_write_async	= atbm_usb_memcpy_toio_async,
	.lock				= atbm_usb_lock,
	.unlock				= atbm_usb_unlock,
	.reset				= atbm_usb_reset,
	.align_size			= atbm_usb_align_size,
	.power_mgmt			= atbm_usb_pm,
	.set_block_size		= atbm_usb_set_block_size,
	.wtd_wakeup			= atbm_wtd_wakeup,
	#ifdef ATBM_USB_RESET
	.usb_reset			= atbm_usb_reset,
	#endif
	.bootloader_debug_config = atbm_usb_debug_config,
	.lmac_start			=atbm_lmac_start,
#ifdef USB_CMD_UES_EP0	
	.ep0_cmd			=atbm_usb_ep0_cmd,
#endif
	.irq_unsubscribe	= atbm_usb_irq_unsubscribe,
	.irq_subscribe	= atbm_usb_irq_subscribe

};

static struct dvobj_priv *usb_dvobj_init(struct usb_interface *usb_intf)
{
	int	i;
	struct dvobj_priv *pdvobjpriv=NULL;
	struct usb_device_descriptor 	*pdev_desc;
	struct usb_host_config			*phost_conf;
	struct usb_config_descriptor		*pconf_desc;
	struct usb_host_interface		*phost_iface;
	struct usb_interface_descriptor	*piface_desc;
	struct usb_host_endpoint		*phost_endp;
	struct usb_endpoint_descriptor	*pendp_desc;
	struct usb_device				*pusbd;
	
	pdvobjpriv = kzalloc(sizeof(*pdvobjpriv), GFP_KERNEL);
	if (!pdvobjpriv){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate USB dvobj.");
		goto exit;
	}
	pdvobjpriv->pusbintf = usb_intf ;
	pusbd = pdvobjpriv->pusbdev = interface_to_usbdev(usb_intf);
	usb_set_intfdata(usb_intf, pdvobjpriv);

	//pdvobjpriv->RtNumInPipes = 0;
	//pdvobjpriv->RtNumOutPipes = 0;
	pdev_desc = &pusbd->descriptor;
#if 1
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"\natbm_usb_device_descriptor:\n");
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bLength=%x\n", pdev_desc->bLength);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDescriptorType=%x\n", pdev_desc->bDescriptorType);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bcdUSB=%x\n", pdev_desc->bcdUSB);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDeviceClass=%x\n", pdev_desc->bDeviceClass);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDeviceSubClass=%x\n", pdev_desc->bDeviceSubClass);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDeviceProtocol=%x\n", pdev_desc->bDeviceProtocol);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bMaxPacketSize0=%x\n", pdev_desc->bMaxPacketSize0);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"idVendor=%x\n", pdev_desc->idVendor);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"idProduct=%x\n", pdev_desc->idProduct);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bcdDevice=%x\n", pdev_desc->bcdDevice);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"iManufacturer=%x\n", pdev_desc->iManufacturer);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"iProduct=%x\n", pdev_desc->iProduct);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"iSerialNumber=%x\n", pdev_desc->iSerialNumber);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bNumConfigurations=%x\n", pdev_desc->bNumConfigurations);
#endif

	phost_conf = pusbd->actconfig;
	pconf_desc = &phost_conf->desc;
#if 1
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"\natbm_usb_configuration_descriptor:\n");
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bLength=%x\n", pconf_desc->bLength);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDescriptorType=%x\n", pconf_desc->bDescriptorType);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"wTotalLength=%x\n", pconf_desc->wTotalLength);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bNumInterfaces=%x\n", pconf_desc->bNumInterfaces);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bConfigurationValue=%x\n", pconf_desc->bConfigurationValue);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"iConfiguration=%x\n", pconf_desc->iConfiguration);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bmAttributes=%x\n", pconf_desc->bmAttributes);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bMaxPower=%x\n", pconf_desc->bMaxPower);
#endif

	phost_iface = &usb_intf->altsetting[0];
	piface_desc = &phost_iface->desc;
#if 1
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"\natbm_usb_interface_descriptor:\n");
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bLength=%x\n", piface_desc->bLength);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDescriptorType=%x\n", piface_desc->bDescriptorType);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bInterfaceNumber=%x\n", piface_desc->bInterfaceNumber);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bAlternateSetting=%x\n", piface_desc->bAlternateSetting);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bNumEndpoints=%x\n", piface_desc->bNumEndpoints);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bInterfaceClass=%x\n", piface_desc->bInterfaceClass);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bInterfaceSubClass=%x\n", piface_desc->bInterfaceSubClass);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"bInterfaceProtocol=%x\n", piface_desc->bInterfaceProtocol);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"iInterface=%x\n", piface_desc->iInterface);
#endif

	//pdvobjpriv->NumInterfaces = pconf_desc->bNumInterfaces;
	//pdvobjpriv->InterfaceNumber = piface_desc->bInterfaceNumber;
	pdvobjpriv->nr_endpoint = piface_desc->bNumEndpoints;


	for (i = 0; i < pdvobjpriv->nr_endpoint; i++)
	{
		phost_endp = phost_iface->endpoint + i;
		if (phost_endp)
		{
			pendp_desc = &phost_endp->desc;

			atbm_dbg(ATBM_APOLLO_DBG_MSG,"\nusb_endpoint_descriptor(%d):\n", i);
			atbm_dbg(ATBM_APOLLO_DBG_MSG,"bLength=%x\n",pendp_desc->bLength);
			atbm_dbg(ATBM_APOLLO_DBG_MSG,"bDescriptorType=%x\n",pendp_desc->bDescriptorType);
			atbm_dbg(ATBM_APOLLO_DBG_MSG,"bEndpointAddress=%x\n",pendp_desc->bEndpointAddress);
			atbm_dbg(ATBM_APOLLO_DBG_MSG,"wMaxPacketSize=%d\n",le16_to_cpu(pendp_desc->wMaxPacketSize));
			atbm_dbg(ATBM_APOLLO_DBG_MSG,"bInterval=%x\n",pendp_desc->bInterval);

			if (usb_endpoint_is_bulk_in(pendp_desc))
			{
				atbm_dbg(ATBM_APOLLO_DBG_MSG,"usb_endpoint_is_bulk_in = %x\n",usb_endpoint_num(pendp_desc));
				pdvobjpriv->ep_in_size=le16_to_cpu(pendp_desc->wMaxPacketSize);
				pdvobjpriv->ep_in=usb_endpoint_num(pendp_desc);
			}
			else if (usb_endpoint_is_bulk_out(pendp_desc))
			{
				atbm_dbg(ATBM_APOLLO_DBG_MSG,"usb_endpoint_is_bulk_out = %x\n",usb_endpoint_num(pendp_desc));
				pdvobjpriv->ep_out_size=le16_to_cpu(pendp_desc->wMaxPacketSize);
				pdvobjpriv->ep_out=usb_endpoint_num(pendp_desc);
			}
			pdvobjpriv->ep_num[i] = usb_endpoint_num(pendp_desc);
		}
	}
	usb_get_dev(pusbd);
	atbm_dbg(ATBM_APOLLO_DBG_MSG,"nr_endpoint=%d, in_num=%d, out_num=%d\n\n", pdvobjpriv->nr_endpoint, pdvobjpriv->ep_in, pdvobjpriv->ep_out);
exit:
	return pdvobjpriv;
}
static int g_rebootSystem = 0;
static int atbm_reboot_notifier(struct notifier_block *nb,
				unsigned long action, void *unused)
{
	printk(KERN_ERR "atbm_reboot_notifier\n");
	g_rebootSystem =1;
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	atbm_usb_exit();
	atbm_ieee80211_exit();
	return NOTIFY_DONE;
}

/* Probe Function to be called by USB stack when device is discovered */
static struct notifier_block atbm_reboot_nb = {
	.notifier_call = atbm_reboot_notifier,
	.priority=1,
};


static int atbm_usb_probe(struct usb_interface *intf,
				   const struct usb_device_id *id)
{
	struct sbus_priv *self;
	struct dvobj_priv *dvobj;
	int status;

	atbm_dbg(ATBM_APOLLO_DBG_INIT, "Probe called\n");
	atomic_set(&g_wtd.wtd_probe, 0);

	self = kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate USB sbus_priv.");
		return -ENOMEM;
	}
	mutex_init(&self->sbus_mutex);

	spin_lock_init(&self->lock);
	self->pdata = atbm_get_platform_data();
	/* 1--- Initialize dvobj_priv */
	dvobj = usb_dvobj_init(intf);
	if (!dvobj){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate USB dvobj.");
		g_dvobj=NULL;
		return -ENOMEM;
	}
	dvobj->self =self;
	self->drvobj=dvobj;
	/*2---alloc rx_urb*/
	dvobj->suspend_skb = dev_alloc_skb(RX_BUFFER_SIZE+64);
	BUG_ON(dvobj->suspend_skb == NULL);
	skb_reserve(self->drvobj->suspend_skb, 64);
	dvobj->suspend_skb_len = 0;
#ifdef ATBM_NEW_USB_AGGR_TX	
	dvobj->tx_save_urb=NULL;
	status = atbm_usb_urb_malloc(self,dvobj->rx_urb,RX_URB_NUM,RX_BUFFER_SIZE,1);
	if (status != 0){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate rx_urb.");
		return status;
	}
	memset(dvobj->rx_urb_map,0,sizeof(dvobj->rx_urb_map));
	/*3---alloc tx_urb*/
	status = atbm_usb_urb_malloc(self,dvobj->tx_urb,TX_URB_NUM,TX_LONG_BUFFER_SIZE+USB_BLOCK_SIZE+USB_BLOCK_SIZE,1);
#else
	status = atbm_usb_urb_malloc(self,dvobj->rx_urb,RX_URB_NUM,RX_BUFFER_SIZE, 1);
	if (status != 0){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate rx_urb.");
		return status;
	}
	memset(dvobj->rx_urb_map,0,sizeof(dvobj->rx_urb_map));
	/*3---alloc tx_urb*/
	status = atbm_usb_urb_malloc(self,dvobj->tx_urb,TX_URB_NUM,TX_BUFFER_SIZE, 1);
#endif
	if (!dvobj->tx_urb){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate tx_urb.");
		return -ENOMEM;
	}	
	memset(dvobj->tx_urb_map,0,sizeof(dvobj->tx_urb_map));
	/*4---alloc cmd_urb*/
	dvobj->cmd_urb=usb_alloc_urb(0,GFP_KERNEL);
	if (!dvobj->cmd_urb){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate cmd_urb.");
		return -ENOMEM;
	}
	/*5---alloc rx data buffer*/
	self->usb_data = kzalloc(ATBM_USB_EP0_MAX_SIZE+16,GFP_KERNEL);
	if (!self->usb_data)
		return -ENOMEM;
	//self->rx_skb = NULL;
//#define ALIGN(a,b)  (((a)+((b)-1))&(~((b)-1)))
	self->usb_req_data = (u8 *)ALIGN((unsigned long)self->usb_data,16);

	atomic_xchg(&self->tx_lock, 0);
	atomic_xchg(&self->rx_lock, 0);
	self->tx_hwChanId = 0;
	self->rx_seqnum = 0;
	self->drvobj->tx_test_seq_need =0;
	self->drvobj->tx_test_hwChanId_need =0;
/*
	atbm_usb_hw_read_port(self,0XB000540,&self->tx_hwChanId,4);
	self->tx_hwChanId++;
	self->tx_hwChanId &=  0x1F;
	self->drvobj->tx_test_hwChanId_need = self->tx_hwChanId;
	*/
	printk("self->tx_hwChanId %d\n",self->tx_hwChanId);
	
	//self->tx_callback_handler = NULL;
	//self->rx_callback_handler = NULL;
	wifi_tx_urb_pending = 0;

	self->wtd = &g_wtd;
	//
	//usb auto-suspend init
	self->suspend=0;	
	self->auto_suspend=0;	

#ifdef CONFIG_PM
#if USB_AUTO_WAKEUP

#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
	usb_enable_autosuspend(self->drvobj->pusbdev);
#else
	self->drvobj->pusbdev->autosuspend_disabled = 0;//autosuspend disabled by the user
#endif
#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	{
		dvobj->pusbdev->do_remote_wakeup=1;
		dvobj->pusbintf->needs_remote_wakeup = 1;
		device_init_wakeup(&dvobj->pusbintf->dev,1);
		pm_runtime_set_autosuspend_delay(&dvobj->pusbdev->dev,15000);
	}
#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#endif //CONFIG_PM

#else //USB_AUTO_WAKEUP
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
	usb_disable_autosuspend(self->drvobj->pusbdev);
#else //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
	self->drvobj->pusbdev->autosuspend_disabled = 1;//autosuspend disabled by the user
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	dvobj->pusbdev->do_remote_wakeup=0;
	dvobj->pusbintf->needs_remote_wakeup = 0;
	device_init_wakeup(&dvobj->pusbintf->dev,1);
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))

#endif //#if USB_AUTO_WAKEUP
#endif //CONFIG_PM


	//initial wifi mac
	status = atbm_core_probe(&atbm_usb_sbus_ops,
			      self, &intf->dev, &self->core);

	if (status) {
		printk("<ERROR> %s %d reset usb\n",__func__,__LINE__);
		usb_free_urb(dvobj->cmd_urb);
		atbm_usb_urb_free(self,self->drvobj->rx_urb,RX_URB_NUM);
		atbm_usb_urb_free(self,self->drvobj->tx_urb,TX_URB_NUM);
		if(self->drvobj->suspend_skb)
		{
			dev_kfree_skb(self->drvobj->suspend_skb);
			self->drvobj->suspend_skb_len = 0;
			self->drvobj->suspend_skb = NULL;
		}
		if(dvobj->pusbdev)
			usb_put_dev(dvobj->pusbdev);
		usb_set_intfdata(intf, NULL);
		kfree(dvobj);
		kfree(self);
		atomic_set(&g_wtd.wtd_probe, -1);
		//reset usb
		usb_reset_device(interface_to_usbdev(intf));
		g_dvobj = NULL;
	}
	else {
		atomic_set(&g_wtd.wtd_probe, 1);
		g_dvobj = dvobj;
		printk("[atbm_wtd]:set wtd_probe = 1\n");
	}
	return status;
}

/* Disconnect Function to be called by USB stack when
 * device is disconnected */
static void atbm_usb_disconnect(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	struct sbus_priv *self  = NULL;
	struct usb_device *pdev=NULL;
	if (dvobj) {
		self  = dvobj->self;
		pdev = dvobj->pusbdev;
		if (self->core) {
			printk(" %s %d core %p\n",__func__,__LINE__,self->core);
			atbm_core_release(self->core);
			self->core = NULL;
		}
#if (EXIT_MODULE_RESET_USB==0)
		if(g_rebootSystem)
#endif /*EXIT_MODULE_RESET_USB*/
		if (interface_to_usbdev(intf)->state != USB_STATE_NOTATTACHED) 
		{
			usb_reset_device(interface_to_usbdev(intf));
		}	
		self->suspend = 1;
		atbm_usb_urb_free(self,self->drvobj->rx_urb,RX_URB_NUM);
		atbm_usb_urb_free(self,self->drvobj->tx_urb,TX_URB_NUM);
		if(self->drvobj->suspend_skb)
		{
			dev_kfree_skb(self->drvobj->suspend_skb);
			self->drvobj->suspend_skb = NULL;
			self->drvobj->suspend_skb_len = 0;
		}
		usb_kill_urb(self->drvobj->cmd_urb);
		usb_free_urb(self->drvobj->cmd_urb);
//		usb_put_dev(self->drvobj->pusbdev);
		usb_set_intfdata(intf, NULL);
		mutex_destroy(&self->sbus_mutex);
		kfree(self->usb_data);
		kfree(self);
		kfree(dvobj);
		g_dvobj = NULL;
		g_hw_priv = NULL;
		if(pdev)
		{
			BUG_ON((pdev!=interface_to_usbdev(intf)));
			printk("we have get dev,so put it in the end\n");
			usb_put_dev(pdev);
		}
		printk("atbm_usb_disconnect---->oK\n");
	}
}

static int atbm_usb_suspend(struct usb_interface *intf,pm_message_t message)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	printk(KERN_DEBUG "***********func=%s,line=%d\n",__func__,__LINE__);
	dvobj->self->suspend=1;
	//atbm_usb_suspend_start(dvobj->self);
	//msleep(20);
	//atbm_usb_urb_kill(dvobj->self,dvobj->rx_urb,RX_URB_NUM);
	return 0;

}

static int atbm_usb_resume(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	printk(KERN_DEBUG "===----===start resume state\n");
	dvobj->self->suspend=0;
	atbm_usb_memcpy_fromio(dvobj->self,0,NULL,RX_BUFFER_SIZE);
	return 0;
	
}
#if 0
static const struct dev_pm_ops atbm_pm_ops = {
	.suspend = atbm_usb_suspend,
	.resume = atbm_usb_resume,
};
#endif

static struct usb_driver apollod_driver = {
	.name		= "atbm_wlan",
	.id_table	= atbm_usb_ids,
	.probe		= atbm_usb_probe,
	.disconnect	= atbm_usb_disconnect,
#ifdef CONFIG_PM
	.suspend	= atbm_usb_suspend,
	.resume		= atbm_usb_resume,
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
	.disable_hub_initiated_lpm = 1,
#endif  //(LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
#if USB_AUTO_WAKEUP
	.supports_autosuspend = 1,
#else //USB_AUTO_WAKEUP
	.supports_autosuspend = 0,
#endif  //USB_AUTO_WAKEUP
};


/* Init Module function -> Called by insmod */
static int  atbm_usb_init(void)
{
	const struct atbm_platform_data *pdata;
	int ret;
	pdata = atbm_get_platform_data();

	ret=driver_build_info();
	if (pdata->clk_ctrl) {
		ret = pdata->clk_ctrl(pdata, true);
		if (ret)
			goto err_clk;
	}
	if (pdata->power_ctrl) {
		ret = pdata->power_ctrl(pdata, true);
		if (ret)
			goto err_power;
	}
	ret = usb_register(&apollod_driver);
	if (ret)
		goto err_reg;

	ret = atbm_usb_on(pdata);
	if (ret)
		goto err_on;

	atbm_wtd_init();
	return 0;

err_on:
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
err_power:
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
err_clk:
	usb_deregister(&apollod_driver);
err_reg:
	return ret;
}

/* Called at Driver Unloading */
static void  atbm_usb_exit(void)
{
	struct atbm_platform_data *pdata;
	printk("atbm_usb_exit+++++++\n");
	pdata = atbm_get_platform_data();
	atbm_wtd_exit();
	usb_deregister(&apollod_driver);
	printk("atbm_usb_exit:usb_deregister\n");
	atbm_usb_off(pdata);
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
	printk("atbm_usb_exit---------\n");
}
static int __init atbm_usb_module_init(void)
{
	printk(KERN_DEBUG "atbm_usb_module_init\n");
	wifi_module_exit =0;
	g_dvobj = NULL;
	atbm_init_firmware();
	atbm_ieee80211_init();
	atbm_usb_init();
	register_reboot_notifier(&atbm_reboot_nb);
	return 0;
}
static void  atbm_usb_module_exit(void)
{
	int loop =0;
	u32 regdata = 0;
	printk("atbm_usb_module_exit g_dvobj %p\n",g_dvobj );
	wifi_module_exit =1;

	if(atomic_read(&g_wtd.wtd_probe)){
		if(g_dvobj){
			printk("module_exit g_hw_priv  %p\n",g_hw_priv );
				
			if(g_hw_priv){
				printk("g_hw_priv->wsm_caps.HiHwCnfBufaddr  %x\n",g_hw_priv->wsm_caps.HiHwCnfBufaddr );
				if((g_hw_priv->wsm_caps.HiHwCnfBufaddr  & 0xFFF00000) == 0x9000000){
					printk("atbm_usb_hw_write_port  0x87690121\n");
				
					regdata = 0x87690121;
					atbm_usb_hw_write_port(g_dvobj->self,  g_hw_priv->wsm_caps.HiHwCnfBufaddr,&regdata,4);
					loop = 0;
					atbm_usb_ep0_cmd(g_dvobj->self);
					while(regdata != 0x87690122) {						
						mdelay(10);						
						atbm_usb_hw_read_port(g_dvobj->self, g_hw_priv->wsm_caps.HiHwCnfBufaddr+4,&regdata,4);
						if(regdata == 0x87690122){
							printk("wait usb lmac rmmod ok !!!!!!!!\n");
							break;
						}
						if(loop++ > 100){
							printk("wait usb lmac rmmod fail !!!!!!!!\n");
							break;
						}
					}
				}
			}
		}
	}
	mdelay(100);
	loop = 0;
	while(wifi_tx_urb_pending !=0){
		mdelay(10);
		if(loop++ > 100){
			break;
		}
	}
	printk("_____atbm_usb_module_exit loop  %d!!!!!!!!\n",loop);
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	unregister_reboot_notifier(&atbm_reboot_nb);
	atbm_usb_exit();
	atbm_ieee80211_exit();
	atbm_release_firmware();
	g_dvobj = NULL;
	printk("atbm_usb_module_exit--\n");
	return ;
}


module_init(atbm_usb_module_init);
module_exit(atbm_usb_module_exit);
