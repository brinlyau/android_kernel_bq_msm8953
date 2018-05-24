/* Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *     Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>

#include "gf_spi.h"

#include <linux/platform_device.h>

#define GF_SPIDEV_NAME      "goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME         "goodix_fp"
#define	GF_INPUT_NAME	    "fingerprint_key"	/*"goodix_fp" */

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME		    "goodix_fp"
#define SPIDEV_MAJOR		154	/* assigned */
#define N_SPI_MINORS		32	/* ... up to 256 */


struct gf_key_map key_map[] =
{
      {  "POWER",  KEY_POWER  },
      {  "HOME" ,  KEY_HOME   },
      {  "MENU" ,  KEY_MENU   },
      {  "BACK" ,  KEY_BACK   },
      {  "UP"   ,  KEY_F18    },
      {  "DOWN" ,  KEY_F18    },
      {  "LEFT" ,  KEY_F18    },
      {  "RIGHT",  KEY_F18    },
      {  "FORCE",  KEY_F9     },
      {  "CLICK",  KEY_F19    },
      {  "DOUBLE",  KEY_F20   },
      {  "LONG",  KEY_F21     },
};

/**************************debug******************************/
#define GF_DEBUG

#define gf_dbg(fmt, args...) do { \
					pr_info("gf:" fmt, ##args);\
		} while (0)
#define FUNC_ENTRY()  pr_info("gf:%s, entry\n", __func__)
#define FUNC_EXIT()  pr_info("gf:%s, exit\n", __func__)

/*Global variables*/
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf_dev gf;
static irqreturn_t gf_irq(int irq, void *handle);

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		pr_warn("IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		pr_warn("IRQ has been disabled.\n");
	}
}

static int driver_init_partial(struct gf_dev *gf_dev)
{
	int ret = 0;
	int rc;
	if (gf_power_on(gf_dev)) {
		ret = -ENODEV;
		goto error;
	}

	if(!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		ret = -EIO;
		goto error;
	} else {
		rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
		if(rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
			ret = -EIO;
			goto error;
		}
		gpio_direction_output(gf_dev->reset_gpio, 1);
		gpio_free(gf_dev->reset_gpio);
	}

	if(!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		ret = -EIO;
		goto error;
	} else {
		pr_info("gf:irq_gpio:%d\n", gf_dev->irq_gpio);
		rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
		if(rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
			ret = -EIO;
			goto error;
		}
		gpio_direction_input(gf_dev->irq_gpio);
	}

	gf_dev->irq = gf_irq_num(gf_dev);
	ret = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"gf", gf_dev);
	if (!ret) {
		gf_dev->irq_enabled = 1;
		enable_irq_wake(gf_dev->irq);
		gf_disable_irq(gf_dev);
	} else {
		pr_info("request_threaded_irq failed with return %d\n", ret);
		goto error;
	}

	if (gf_dev->users == 1)
		gf_enable_irq(gf_dev);

	/*power the sensor*/
	gf_power_on(gf_dev);
	gf_hw_reset(gf_dev, 1);
	gf_dev->device_available = 1;

	return 0;

error:
	if (gpio_is_valid(gf_dev->irq_gpio)){
		gf_disable_irq(gf_dev);
		free_irq(gf_dev->irq, gf_dev);
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)){
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
	gf_dev->device_available = 0;
	return ret;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key = { 0 };
	int retval = 0;
	int i;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	if ((retval == 0) && (_IOC_DIR(cmd) & _IOC_WRITE))
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	if(gf_dev->device_available == 0) {
		if((cmd == GF_IOC_POWER_ON) || (cmd == GF_IOC_POWER_OFF)){
            gf_dbg("power cmd\n");
        }
        else{
            gf_dbg("Sensor is power off currently. \n");
            return -ENODEV;
        }
    }

	switch (cmd) {
	case GF_IOC_DISABLE_IRQ:
		if(gf_dev->irq_enabled)
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		if(!gf_dev->irq_enabled)
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_SETSPEED:
		gf_dbg("This kernel doesn't support control clk in AP\n");
		break;
	case GF_IOC_RESET:
		gf_hw_reset(gf_dev, 1);
		break;
	case GF_IOC_COOLBOOT:
		gf_power_off(gf_dev);
		mdelay(5);
		gf_power_on(gf_dev);
		break;
		/*add by Lee*/
	case GF_IOC_REQUEST_IRQ:
		pr_info("request_threaded_irq,gf_dev->irq= %d \n",gf_dev->irq);

		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "gf", gf_dev);

		if (retval) {
			pr_warn("Failed to request irq, err = %d \n",retval);
			break;
		}
		enable_irq_wake(gf_dev->irq);
		gf_dev->irq_enabled = 1;
		gf_disable_irq(gf_dev);
	    break;
		/*add by Lee*/
	case GF_IOC_SENDKEY:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			gf_dbg("Failed to copy data from user space.\n");
			retval = -EFAULT;
			break;
		}

		for(i = 0; i< ARRAY_SIZE(key_map); i++) {
			if(key_map[i].val == gf_key.key){
				input_report_key(gf_dev->input, gf_key.key, gf_key.value);
				input_sync(gf_dev->input);
				break;
			}
		}

		if(i == ARRAY_SIZE(key_map)) {
			pr_warn("key %d not support yet \n", gf_key.key);
			retval = -EFAULT;
		}

		break;
	case GF_IOC_CLK_READY:
        gf_dbg("ioctl CMD %u Doesn't support control clock.\n", _IOC_NR(cmd));
		break;
	case GF_IOC_CLK_UNREADY:
        gf_dbg("ioctl CMD %u Doesn't support control clock.\n", _IOC_NR(cmd));
		break;
	case GF_IOC_PM_FBCABCK:
		__put_user(gf_dev->fb_black, (u8 __user *) arg);
		break;
        case GF_IOC_POWER_ON:
        if(gf_dev->device_available == 1)
            gf_dbg("Sensor has already powered-on.\n");
		else
            gf_power_on(gf_dev);
        gf_dev->device_available = 1;
        break;
        case GF_IOC_POWER_OFF:
        if(gf_dev->device_available == 0)
            gf_dbg("Sensor has already powered-off.\n");
		else
            gf_power_off(gf_dev);
        gf_dev->device_available = 0;
        break;
	case GF_IOC_ENABLE_GPIO:
		if(!gf_dev->device_available)
			driver_init_partial(gf_dev);
		break;
	default:
		gf_dbg("unsupport cmd:0x%x\n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
    struct gf_dev *gf_dev = &gf;

    char temp = GF_NET_EVENT_IRQ;
    if(gf_dev->gf_extend_wakelock)
    	wake_lock_timeout(&gf_dev->gf_wake_lock, HZ * 2);
    sendnlmsg(&temp);

	return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			gf_dbg("Device Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			gf_dbg("Succeed to open device. irq = %d\n", gf_dev->irq);
			if (gf_dev->users == 1)
				gf_enable_irq(gf_dev);
            /*power the sensor*/
            gf_power_on(gf_dev);
            gf_hw_reset(gf_dev, 360);
            gf_dev->device_available = 1;
		}
	} else {
		gf_dbg("No device for minor %d\n", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close?? */
	gf_dev->users--;
	if (!gf_dev->users) {
		gf_dbg("disble_irq. irq = %d\n", gf_dev->irq);
		gf_disable_irq(gf_dev);

		if (gpio_is_valid(gf_dev->irq_gpio)){
			free_irq(gf_dev->irq,gf_dev);
			gpio_free(gf_dev->irq_gpio);
			pr_info("Goodix remove irq_gpio success\n");
		}
        /*power off the sensor*/
        gf_dev->device_available = 0;
        gf_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	*/
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
};

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
	char temp = 0;

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	pr_debug("[info] %s go to the goodix_fb_state_chg_callback value = %d\n",
		__func__, (int)val);
	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
				temp = GF_NET_EVENT_FB_BLACK;
				sendnlmsg(&temp);
				/*device unavailable */
			}
			break;
		case FB_BLANK_UNBLANK:
		case FB_BLANK_NORMAL:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 0;
				temp = GF_NET_EVENT_FB_UNBLACK;
				sendnlmsg(&temp);
			}
			break;
		default:
			pr_info("%s defalut\n", __func__);
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
    int i;

    set_bit(EV_KEY, gf_dev->input->evbit); //tell the kernel is key event
    for(i = 0; i< ARRAY_SIZE(key_map); i++) {
        set_bit(key_map[i].val, gf_dev->input->keybit);
    }

    gf_dev->input->name = GF_INPUT_NAME;
    if (input_register_device(gf_dev->input))
        pr_warn("Failed to register GF as input device.\n");
}

static struct class *gf_class;
static int gf_probe(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int ret;
	struct regulator *vreg;
	FUNC_ENTRY();

	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->spi = pdev;
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pwr_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;

	printk("[humingming] gf_probe 1\n");
	if (gf_parse_dts(gf_dev))
		goto error;
	printk("[humingming] gf_probe 2\n");
	if (!gpio_is_valid(gf_dev->pwr_gpio)) {
		vreg = regulator_get(&gf_dev->spi->dev,"vdd_ana");
		if (!vreg) {
			dev_err(&gf_dev->spi->dev, "Unable to get vdd_ana\n");
			goto error;
		}

		if (regulator_count_voltages(vreg) > 0) {
			ret = regulator_set_voltage(vreg, 2800000,2800000);
			if (ret){
				dev_err(&gf_dev->spi->dev,"Unable to set voltage on vdd_ana");
				goto error;
			}
		}
		ret = regulator_enable(vreg);
		if (ret) {
			dev_err(&gf_dev->spi->dev, "error enabling vdd_ana %d\n",ret);
			regulator_put(vreg);
			vreg = NULL;
			goto error;
		}
		dev_info(&gf_dev->spi->dev,"Set voltage on vdd_ana for goodix fingerprint");
	}

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	*/
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				    gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			dev_dbg(&gf_dev->input->dev,"Faile to allocate input device.\n");
			status = -ENOMEM;
		}

		gf_dev->notifier = goodix_noti_block;
		fb_register_client(&gf_dev->notifier);
		gf_reg_key_kernel(gf_dev);
    gf_dev->irq = gf_irq_num(gf_dev);
	gf_dev->gf_extend_wakelock = 0;
	gf_dev->wlock_name = kasprintf(GFP_KERNEL,
				"%s", "goodix_intr");
	wake_lock_init(&gf_dev->gf_wake_lock, WAKE_LOCK_SUSPEND,
				gf_dev->wlock_name);

	}

	return status;

error:
	gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
	if (gf_dev->devt != 0) {
		dev_info(&gf_dev->spi->dev,"Err: status = %d\n", status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
		if (gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);
	}

	FUNC_EXIT();
	return status;
}

static int gf_remove(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		gf_disable_irq(gf_dev);
		free_irq(gf_dev->irq, gf_dev);
	}

	gf_dev->gf_extend_wakelock = 0;
	wake_lock_destroy(&gf_dev->gf_wake_lock);

	if (gf_dev->input != NULL)
		input_unregister_device(gf_dev->input);
		input_free_device(gf_dev->input);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->users == 0)
		kfree(gf_dev);

    fb_unregister_client(&gf_dev->notifier);
	mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

static int gf_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gf_dev *gf_dev = &gf;
	if(!gf_dev->irq_enabled){
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
	gf_dev->gf_extend_wakelock = 1;
	pr_info("gf_suspend_test.\n");
	return 0;
}

static int gf_resume(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	gf_dev->gf_extend_wakelock = 0;
	pr_info("gf_resume_test.\n");
	return 0;
}

static struct of_device_id gx_match_table[] = {
	{.compatible = GF_SPIDEV_NAME,},
	{},
};

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
	.suspend = gf_suspend,
	.resume = gf_resume,
};

static int __init gf_init(void)
{
	int status;
	FUNC_ENTRY();

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	*/

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		pr_warn("Failed to register char device!\n");
		FUNC_EXIT();
		return status;
	}
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf_class);
	}
	status = platform_driver_register(&gf_driver);
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
	}

        netlink_init();
	pr_info(" status = 0x%x\n", status);
	FUNC_EXIT();
	return 0;		//status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	FUNC_ENTRY();
    netlink_exit();
	platform_driver_unregister(&gf_driver);
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
	FUNC_EXIT();
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
