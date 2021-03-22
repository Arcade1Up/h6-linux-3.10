/*
 * Device handling thread interface for mac80211 altobeam APOLLO drivers
 *
 * Copyright (c) 2016, altobeam
 * Author:
 *
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ATBM_APOLLO_BH_H
#define ATBM_APOLLO_BH_H

/* extern */ struct atbm_common;

#define SDIO_BLOCK_SIZE 288


#include "bh_usb.h"

int atbm_register_bh(struct atbm_common *hw_priv);
void atbm_unregister_bh(struct atbm_common *hw_priv);
void atbm_irq_handler(struct atbm_common *hw_priv);
void atbm_bh_wakeup(struct atbm_common *hw_priv);
int atbm_bh_suspend(struct atbm_common *hw_priv);
int atbm_bh_resume(struct atbm_common *hw_priv);
/* Must be called from BH thread. */
void atbm_enable_powersave(struct atbm_vif *priv,
			     bool enable);
int wsm_release_tx_buffer(struct atbm_common *hw_priv, int count);
void wsm_alloc_tx_buffer(struct atbm_common *hw_priv);
int wsm_release_vif_tx_buffer(struct atbm_common *hw_priv, int if_id,
				int count);
void atbm_put_skb(struct atbm_common *hw_priv, struct sk_buff *skb);

int atbm_powerave_sdio_sync(struct atbm_common *hw_priv);
int atbm_device_wakeup(struct atbm_common *hw_priv);
void atbm_get_cca_work(struct work_struct *work);

static inline int atbm_bh_is_term(struct atbm_common *hw_priv){
	if((hw_priv->bh_thread==NULL) || (hw_priv->bh_error==1)){
		return 1;
	}
	else {
		return 0;
	}
}
#define can_not_queue_work(hw_priv) 					\
	(((hw_priv)->workqueue==NULL)||(atbm_bh_is_term(hw_priv)))
#define atbm_hw_priv_queue_work(hw_priv,work)		\
	(can_not_queue_work(hw_priv) ? -1:queue_work((hw_priv)->workqueue,work))
#define atbm_hw_priv_queue_delayed_work(hw_priv,dwork,delay)	\
	(can_not_queue_work(hw_priv) ? -1:queue_delayed_work((hw_priv)->workqueue,dwork,delay))
static inline bool atbm_cancle_queue_work(struct work_struct *work,bool sync)
{
	bool retval = false;
	if(sync || work_pending(work))
	{
		retval = cancel_work_sync(work);
	}

	return retval;
}

static inline bool atbm_cancle_delayed_work(struct delayed_work *dwork,bool sync)
{
	bool retval = false;
	if(sync)
	{
		retval = cancel_delayed_work_sync(dwork);
	}
	else
	{
		retval = cancel_delayed_work(dwork);
	}

	return retval;
}
#endif /* ATBM_APOLLO_BH_H */
