/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/diagchar.h>
#include <linux/kmemleak.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/ratelimit.h>
#include <linux/platform_device.h>
#include <linux/smux.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include "diagchar.h"
#include "diagmem.h"
#include "diagfwd_cntl.h"
#include "diagfwd_smux.h"
#include "diagfwd_hsic.h"
#include "diag_masks.h"
#include "diagfwd_bridge.h"

struct diag_bridge_dev *diag_bridge;

int diagfwd_connect_bridge(int process_cable)
{
	int i;

	pr_debug("diag: in %s\n", __func__);

	for (i = 0; i < MAX_BRIDGES; i++)
		if (diag_bridge[i].enabled)
			connect_bridge(process_cable, i);
	return 0;
}

void connect_bridge(int process_cable, int index)
{
	int err;

	mutex_lock(&diag_bridge[index].bridge_mutex);
	
	if (process_cable) {
		err = usb_diag_alloc_req(diag_bridge[index].ch, N_MDM_WRITE,
			       N_MDM_READ);
		if (err)
			pr_err("diag: unable to alloc USB req on mdm ch err:%d\n",
							 err);

		diag_bridge[index].usb_connected = 1;
		driver->qxdmusb_drop = 0;
	}

	if (index == SMUX && driver->diag_smux_enabled) {
		driver->in_busy_smux = 0;
		diagfwd_connect_smux();
	} else if (index == HSIC && driver->hsic_device_enabled) {
		driver->in_busy_hsic_read_on_device = 0;
		driver->in_busy_hsic_write = 0;
		
		if (!driver->hsic_device_opened) {
			err = diag_bridge_open(&hsic_diag_bridge_ops);
			if (err) {
				pr_err("diag: HSIC channel open error: %d\n",
					  err);
			} else {
				pr_debug("diag: opened HSIC channel\n");
				driver->hsic_device_opened = 1;
			}
		} else {
			pr_debug("diag: HSIC channel already open\n");
		}
		if (driver->hsic_device_opened) {
			driver->hsic_ch = 1;
			
			if (driver->logging_mode == USB_MODE)
				queue_work(diag_bridge[HSIC].wq,
					  &diag_bridge[HSIC].diag_read_work);
			
			queue_work(diag_bridge[HSIC].wq,
				   &driver->diag_read_hsic_work);
		}
	}
	mutex_unlock(&diag_bridge[index].bridge_mutex);
}

int diagfwd_disconnect_bridge(int process_cable)
{
	int i;
	pr_debug("diag: In %s, process_cable: %d\n", __func__, process_cable);

	for (i = 0; i < MAX_BRIDGES; i++) {
		if (diag_bridge[i].enabled) {
			mutex_lock(&diag_bridge[i].bridge_mutex);
			
			if (process_cable) {
				diag_bridge[i].usb_connected = 0;
				usb_diag_free_req(diag_bridge[i].ch);
				driver->qxdmusb_drop = 1;
			}

			if (i == HSIC && driver->hsic_device_enabled &&
				 driver->logging_mode != MEMORY_DEVICE_MODE) {
#if !DIAG_XPST
				driver->in_busy_hsic_read_on_device = 1;
				driver->in_busy_hsic_write = 1;
				
				diag_hsic_close();
#endif
			} else if (i == SMUX && driver->diag_smux_enabled &&
					driver->logging_mode == USB_MODE) {
				driver->in_busy_smux = 1;
				driver->lcid = LCID_INVALID;
				driver->smux_connected = 0;
				
				msm_smux_close(LCID_VALID);
			}
			mutex_unlock(&diag_bridge[i].bridge_mutex);
		}
	}
	return 0;
}

int diagfwd_read_complete_bridge(struct diag_request *diag_read_ptr)
{
	 int index = (int)(diag_read_ptr->context);

	
	diag_bridge[index].read_len = diag_read_ptr->actual;

	if (index == SMUX) {
		if (driver->diag_smux_enabled) {
			diagfwd_read_complete_smux();
			return 0;
		} else {
			pr_warning("diag: incorrect callback for smux\n");
		}
	}

	
	driver->in_busy_hsic_read_on_device = 0;
	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return 0;
	}

#if DIAG_XPST
	if (!driver->in_busy_hsic_write && diag_bridge[HSIC].usb_buf_out &&
		(diag_bridge[HSIC].read_len > 0) && !driver->nohdlc) {
#else
	if (!driver->in_busy_hsic_write && diag_bridge[HSIC].usb_buf_out &&
		(diag_bridge[HSIC].read_len > 0)) {
#endif

		int err;
		driver->in_busy_hsic_write = 1;
		err = diag_bridge_write(diag_bridge[HSIC].usb_buf_out,
					diag_bridge[HSIC].read_len);
		if (err) {
			pr_err_ratelimited("diag: mdm data on HSIC write err: %d\n",
					err);
			if ((-ENODEV) != err)
				driver->in_busy_hsic_write = 0;
		}
	}

	if (!driver->in_busy_hsic_write)
		queue_work(diag_bridge[HSIC].wq,
			 &diag_bridge[HSIC].diag_read_work);

	return 0;
}

static void diagfwd_bridge_notifier(void *priv, unsigned event,
					struct diag_request *d_req)
{
	int index;

	switch (event) {
	case USB_DIAG_CONNECT:
		diagfwd_connect_bridge(1);
		break;
	case USB_DIAG_DISCONNECT:
		queue_work(driver->diag_wq,
			 &driver->diag_disconnect_work);
		break;
	case USB_DIAG_READ_DONE:
		index = (int)(d_req->context);
		queue_work(diag_bridge[index].wq,
		&diag_bridge[index].usb_read_complete_work);
		break;
	case USB_DIAG_WRITE_DONE:
		index = (int)(d_req->context);
		if (index == HSIC &&  driver->hsic_device_enabled)
			diagfwd_write_complete_hsic(d_req);
		else if (index == SMUX && driver->diag_smux_enabled)
			diagfwd_write_complete_smux();
		break;
	default:
		pr_err("diag: in %s: Unknown event from USB diag:%u\n",
			__func__, event);
		break;
	}
}

void diagfwd_bridge_init(int index)
{
	int ret;
	unsigned char name[20];

	if (index == HSIC)
		strlcpy(name, "hsic", sizeof(name));
	else
		strlcpy(name, "smux", sizeof(name));

	strlcpy(diag_bridge[index].name, name, sizeof(diag_bridge[index].name));
	strlcat(name, "_diag_wq", sizeof(diag_bridge[index].name));
	diag_bridge[index].enabled = 1;
	diag_bridge[index].wq = create_singlethread_workqueue(name);
	diag_bridge[index].read_len = 0;
	diag_bridge[index].write_len = 0;
	if (diag_bridge[index].usb_buf_out == NULL)
		diag_bridge[index].usb_buf_out =
				 kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
	if (diag_bridge[index].usb_buf_out == NULL)
		goto err;
	if (diag_bridge[index].usb_read_ptr == NULL)
		diag_bridge[index].usb_read_ptr =
			 kzalloc(sizeof(struct diag_request), GFP_KERNEL);
	if (diag_bridge[index].usb_read_ptr == NULL)
		goto err;
	if (diag_bridge[index].usb_read_ptr->context == NULL)
		diag_bridge[index].usb_read_ptr->context =
					 kzalloc(sizeof(int), GFP_KERNEL);
	if (diag_bridge[index].usb_read_ptr->context == NULL)
		goto err;
	mutex_init(&diag_bridge[index].bridge_mutex);

	if (index == HSIC) {
		INIT_WORK(&(diag_bridge[index].usb_read_complete_work),
				 diag_usb_read_complete_hsic_fn);
#ifdef CONFIG_DIAG_OVER_USB
		INIT_WORK(&(diag_bridge[index].diag_read_work),
		      diag_read_usb_hsic_work_fn);
		diag_bridge[index].ch = usb_diag_open(DIAG_MDM, (void *)index,
						  diagfwd_bridge_notifier);
		if (IS_ERR(diag_bridge[index].ch)) {
			pr_err("diag: Unable to open USB diag MDM channel\n");
			goto err;
		}
#endif
		
		ret = platform_driver_register(&msm_hsic_ch_driver);
		if (ret)
			pr_err("diag: could not register HSIC device, ret: %d\n",
									 ret);
	} else if (index == SMUX) {
		INIT_WORK(&(diag_bridge[index].usb_read_complete_work),
					 diag_usb_read_complete_smux_fn);
#ifdef CONFIG_DIAG_OVER_USB
		INIT_WORK(&(diag_bridge[index].diag_read_work),
					 diag_read_usb_smux_work_fn);
		diag_bridge[index].ch = usb_diag_open(DIAG_QSC, (void *)index,
					     diagfwd_bridge_notifier);
		if (IS_ERR(diag_bridge[index].ch)) {
			pr_err("diag: Unable to open USB diag QSC channel\n");
			goto err;
		}
#endif
		ret = platform_driver_register(&msm_diagfwd_smux_driver);
		if (ret)
			pr_err("diag: could not register SMUX device, ret: %d\n",
									 ret);
	}
	 return;
err:
	pr_err("diag: Could not initialize for bridge forwarding\n");
	kfree(diag_bridge[index].usb_buf_out);
	kfree(driver->hsic_buf_tbl);
	kfree(driver->write_ptr_mdm);
	kfree(diag_bridge[index].usb_read_ptr);
	if (diag_bridge[index].wq)
		destroy_workqueue(diag_bridge[index].wq);
	return;
}

void diagfwd_bridge_exit(void)
{
	int i;
	pr_debug("diag: in %s\n", __func__);

	if (driver->hsic_device_enabled) {
		diag_hsic_close();
		driver->hsic_device_enabled = 0;
		diag_bridge[HSIC].enabled = 0;
	}
	driver->hsic_inited = 0;
	diagmem_exit(driver, POOL_TYPE_ALL);
	if (driver->diag_smux_enabled) {
		driver->lcid = LCID_INVALID;
		kfree(driver->buf_in_smux);
		driver->diag_smux_enabled = 0;
		diag_bridge[SMUX].enabled = 0;
	}
	platform_driver_unregister(&msm_hsic_ch_driver);
	platform_driver_unregister(&msm_diagfwd_smux_driver);
	
	for (i = 0; i < MAX_BRIDGES; i++) {
		if (diag_bridge[i].enabled) {
#ifdef CONFIG_DIAG_OVER_USB
			if (diag_bridge[i].usb_connected)
				usb_diag_free_req(diag_bridge[i].ch);
			usb_diag_close(diag_bridge[i].ch);
#endif
			kfree(diag_bridge[i].usb_buf_out);
			kfree(diag_bridge[i].usb_read_ptr);
			destroy_workqueue(diag_bridge[i].wq);
			diag_bridge[i].enabled = 0;
		}
	}
	kfree(driver->hsic_buf_tbl);
	kfree(driver->write_ptr_mdm);
}
