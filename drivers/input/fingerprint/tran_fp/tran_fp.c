// SPDX-License-Identifier: GPL-2.0
/*
 * Transsion / Chipone ICNT7332 Fingerprint Driver (tran_fp -> /dev/fpsensor)
 * For Infinix Note 10 (X6812)
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#ifdef CONFIG_SPI_MT65XX
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif

#define FPSENSOR_IOC_MAGIC          0xf0
#define FPSENSOR_IOC_FAE_INFO       _IOWR(FPSENSOR_IOC_MAGIC, 0, uint64_t)
#define FPSENSOR_IOC_INIT           _IOWR(FPSENSOR_IOC_MAGIC, 0, uint32_t)
#define FPSENSOR_IOC_EXIT           _IOWR(FPSENSOR_IOC_MAGIC, 1, uint32_t)
#define FPSENSOR_IOC_RESET          _IOWR(FPSENSOR_IOC_MAGIC, 2, uint32_t)
#define FPSENSOR_IOC_ENABLE_IRQ     _IOWR(FPSENSOR_IOC_MAGIC, 3, uint32_t)
#define FPSENSOR_IOC_DISABLE_IRQ    _IOWR(FPSENSOR_IOC_MAGIC, 4, uint32_t)
#define FPSENSOR_IOC_GET_INT_VAL    _IOWR(FPSENSOR_IOC_MAGIC, 5, uint32_t)
#define FPSENSOR_IOC_DISABLE_SPI_CLK _IOWR(FPSENSOR_IOC_MAGIC, 6, uint32_t)
#define FPSENSOR_IOC_ENABLE_SPI_CLK _IOWR(FPSENSOR_IOC_MAGIC, 7, uint32_t)
#define FPSENSOR_IOC_ENABLE_POWER   _IOWR(FPSENSOR_IOC_MAGIC, 8, uint32_t)
#define FPSENSOR_IOC_DISABLE_POWER  _IOWR(FPSENSOR_IOC_MAGIC, 9, uint32_t)
#define FPSENSOR_IOC_SEND_KEY       _IOWR(FPSENSOR_IOC_MAGIC, 10, uint64_t)
#define FPSENSOR_IOC_REMOVE         _IOWR(FPSENSOR_IOC_MAGIC, 12, uint32_t)
#define FPSENSOR_IOC_CANCEL_WAIT    _IOWR(FPSENSOR_IOC_MAGIC, 13, uint32_t)
#define FPSENSOR_IOC_GET_FP_STATUS  _IOWR(FPSENSOR_IOC_MAGIC, 19, uint32_t)
#define FPSENSOR_IOC_ENABLE_REPORT_BLANKON _IOWR(FPSENSOR_IOC_MAGIC, 21, uint32_t)
#define FPSENSOR_IOC_UPDATE_DRIVER_SN _IOWR(FPSENSOR_IOC_MAGIC, 22, uint32_t)

struct tran_fp_data {
	struct spi_device *spi;
	struct pinctrl *pinctrl;
	struct pinctrl_state *st_default;
	struct pinctrl_state *st_irq;
	struct pinctrl_state *st_rst_h;
	struct pinctrl_state *st_rst_l;
	int irq_gpio;
	int reset_gpio;
	int irq;
	int irq_enabled;
	int spi_clk_enabled;
	int irq_flag;
	int cancel;
	int fb_status;
	wait_queue_head_t wq_irq_return;
	struct mutex lock;
	struct notifier_block fb_notifier;
};

static struct tran_fp_data *g_fp;

static void tran_fp_reset(struct tran_fp_data *fp)
{
	if (fp->st_rst_l && fp->st_rst_h) {
		pinctrl_select_state(fp->pinctrl, fp->st_rst_l);
		mdelay(2);
		pinctrl_select_state(fp->pinctrl, fp->st_rst_h);
		mdelay(5);
	} else if (gpio_is_valid(fp->reset_gpio)) {
		gpio_direction_output(fp->reset_gpio, 0);
		mdelay(2);
		gpio_direction_output(fp->reset_gpio, 1);
		mdelay(5);
	}
}

static void tran_fp_enable_irq(struct tran_fp_data *fp)
{
	if (fp->irq > 0 && !fp->irq_enabled) {
		enable_irq(fp->irq);
		fp->irq_enabled = 1;
	}
}

static void tran_fp_disable_irq(struct tran_fp_data *fp)
{
	if (fp->irq > 0 && fp->irq_enabled) {
		disable_irq_nosync(fp->irq);
		fp->irq_enabled = 0;
	}
}

static irqreturn_t tran_fp_irq_handler(int irq, void *handle)
{
	struct tran_fp_data *fp = handle;

	fp->irq_flag = 1;
	pm_wakeup_event(&fp->spi->dev, 2000);
	wake_up_interruptible(&fp->wq_irq_return);
	return IRQ_HANDLED;
}

static int tran_fp_fb_notifier(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct tran_fp_data *fp = container_of(nb, struct tran_fp_data, fb_notifier);

	if (!evdata || !evdata->data || event != FB_EVENT_BLANK)
		return 0;

	blank = evdata->data;
	fp->fb_status = (*blank == FB_BLANK_UNBLANK) ? 1 : 0;
	return 0;
}

static int tran_fp_open(struct inode *inode, struct file *filp)
{
	pr_info("tran_fp: open called\n");
	filp->private_data = g_fp;
	return 0;
}

static int tran_fp_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static unsigned int tran_fp_poll(struct file *filp, poll_table *wait)
{
	struct tran_fp_data *fp = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &fp->wq_irq_return, wait);
	if (fp->irq_flag || fp->cancel) {
		mask = POLLIN | POLLRDNORM;
		fp->irq_flag = 0;
		fp->cancel = 0;
	}
	return mask;
}

static long tran_fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct tran_fp_data *fp = filp->private_data;
	int val = 0;

	if (!fp)
		return -EINVAL;

	mutex_lock(&fp->lock);
	pr_info("tran_fp: ioctl cmd=0x%x\n", cmd);
	switch (cmd) {
	case FPSENSOR_IOC_INIT:
		fp->irq_flag = 0;
		fp->cancel = 0;
		break;
	case FPSENSOR_IOC_EXIT:
	case FPSENSOR_IOC_REMOVE:
		tran_fp_disable_irq(fp);
		break;
	case FPSENSOR_IOC_RESET:
		tran_fp_reset(fp);
		break;
	case FPSENSOR_IOC_ENABLE_IRQ:
		tran_fp_enable_irq(fp);
		break;
	case FPSENSOR_IOC_DISABLE_IRQ:
		tran_fp_disable_irq(fp);
		break;
	case FPSENSOR_IOC_GET_INT_VAL:
		val = gpio_is_valid(fp->irq_gpio) ? gpio_get_value(fp->irq_gpio) : 0;
		if (copy_to_user((void __user *)arg, &val, sizeof(val))) {
			mutex_unlock(&fp->lock);
			return -EFAULT;
		}
		break;
	case FPSENSOR_IOC_ENABLE_SPI_CLK:
#ifdef CONFIG_SPI_MT65XX
		if (!fp->spi_clk_enabled) {
			mt_spi_enable_master_clk(fp->spi);
			fp->spi_clk_enabled = 1;
		}
#endif
		break;
	case FPSENSOR_IOC_DISABLE_SPI_CLK:
#ifdef CONFIG_SPI_MT65XX
		if (fp->spi_clk_enabled) {
			mt_spi_disable_master_clk(fp->spi);
			fp->spi_clk_enabled = 0;
		}
#endif
		break;
	case FPSENSOR_IOC_CANCEL_WAIT:
		fp->cancel = 1;
		wake_up_interruptible(&fp->wq_irq_return);
		break;
	case FPSENSOR_IOC_GET_FP_STATUS:
		val = fp->fb_status;
		if (copy_to_user((void __user *)arg, &val, sizeof(val))) {
			mutex_unlock(&fp->lock);
			return -EFAULT;
		}
		break;
	case FPSENSOR_IOC_ENABLE_POWER:
	case FPSENSOR_IOC_DISABLE_POWER:
	case FPSENSOR_IOC_SEND_KEY:
	case FPSENSOR_IOC_ENABLE_REPORT_BLANKON:
	case FPSENSOR_IOC_UPDATE_DRIVER_SN:
	case FPSENSOR_IOC_FAE_INFO:
		break;
	default:
		pr_info_ratelimited("tran_fp: unhandled ioctl 0x%x\n", cmd);
		break;
	}
	mutex_unlock(&fp->lock);
	return 0;
}

static const struct file_operations tran_fp_fops = {
	.owner = THIS_MODULE,
	.open = tran_fp_open,
	.release = tran_fp_release,
	.poll = tran_fp_poll,
	.unlocked_ioctl = tran_fp_ioctl,
	.compat_ioctl = tran_fp_ioctl,
};

static struct kobject *tran_fp_kobj;

static ssize_t vendor_name_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "chipone\n");
}

static struct kobj_attribute vendor_name_attr = __ATTR_RO(vendor_name);

static struct miscdevice tran_fp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fpsensor",
	.fops = &tran_fp_fops,
};

static int tran_fp_probe(struct spi_device *spi)
{
	struct tran_fp_data *fp;
	struct device_node *np = spi->dev.of_node;
	int ret;

	fp = devm_kzalloc(&spi->dev, sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;

	fp->spi = spi;
	fp->fb_status = 1;
	mutex_init(&fp->lock);
	init_waitqueue_head(&fp->wq_irq_return);
	device_init_wakeup(&spi->dev, true);

	fp->pinctrl = devm_pinctrl_get(&spi->dev);
	if (!IS_ERR(fp->pinctrl)) {
		fp->st_default = pinctrl_lookup_state(fp->pinctrl, "tran_fp_default");
		if (IS_ERR(fp->st_default)) fp->st_default = NULL;
		fp->st_irq = pinctrl_lookup_state(fp->pinctrl, "tran_fp_pin_irq");
		if (IS_ERR(fp->st_irq)) fp->st_irq = NULL;
		fp->st_rst_h = pinctrl_lookup_state(fp->pinctrl, "tran_fp_reset_high");
		if (IS_ERR(fp->st_rst_h)) fp->st_rst_h = NULL;
		fp->st_rst_l = pinctrl_lookup_state(fp->pinctrl, "tran_fp_reset_low");
		if (IS_ERR(fp->st_rst_l)) fp->st_rst_l = NULL;
		if (fp->st_default)
			pinctrl_select_state(fp->pinctrl, fp->st_default);
		if (fp->st_irq)
			pinctrl_select_state(fp->pinctrl, fp->st_irq);
	}

	fp->reset_gpio = of_get_named_gpio(np, "reset-gpio-std", 0);
	if (!gpio_is_valid(fp->reset_gpio))
		fp->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	fp->irq_gpio = of_get_named_gpio(np, "irq-gpio-std", 0);
	if (!gpio_is_valid(fp->irq_gpio))
		fp->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);

	if (gpio_is_valid(fp->irq_gpio))
		gpio_request(fp->irq_gpio, "tran_fp_irq");
	if (gpio_is_valid(fp->reset_gpio))
		gpio_request(fp->reset_gpio, "tran_fp_rst");

	fp->irq = irq_of_parse_and_map(np, 0);
	if (fp->irq <= 0 && gpio_is_valid(fp->irq_gpio))
		fp->irq = gpio_to_irq(fp->irq_gpio);

	if (fp->irq > 0) {
		ret = devm_request_threaded_irq(&spi->dev, fp->irq, NULL,
						tran_fp_irq_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"tran_fp", fp);
		if (ret == 0) {
			enable_irq_wake(fp->irq);
			disable_irq_nosync(fp->irq);
			fp->irq_enabled = 0;
		} else {
			pr_err("tran_fp: request_irq(%d) failed: %d\n", fp->irq, ret);
			fp->irq = 0;
		}
	}

	fp->fb_notifier.notifier_call = tran_fp_fb_notifier;
	fb_register_client(&fp->fb_notifier);

	tran_fp_reset(fp);
	g_fp = fp;
	spi_set_drvdata(spi, fp);

	ret = misc_register(&tran_fp_misc);
	if (ret) {
		pr_err("tran_fp: misc_register failed: %d\n", ret);
		return ret;
	}

	tran_fp_kobj = kobject_create_and_add("tran_fp", kernel_kobj);
	if (tran_fp_kobj) {
		ret = sysfs_create_file(tran_fp_kobj, &vendor_name_attr.attr);
		if (ret)
			pr_err("tran_fp: sysfs_create_file failed: %d\n", ret);
	}

	pr_info("tran_fp: probe success! irq=%d rst_gpio=%d irq_gpio=%d\n",
		fp->irq, fp->reset_gpio, fp->irq_gpio);
	return 0;
}

static int tran_fp_remove(struct spi_device *spi)
{
	struct tran_fp_data *fp = spi_get_drvdata(spi);

	misc_deregister(&tran_fp_misc);
	fb_unregister_client(&fp->fb_notifier);
	return 0;
}

static const struct of_device_id tran_fp_of_match[] = {
	{ .compatible = "tran_fp" },
	{},
};
MODULE_DEVICE_TABLE(of, tran_fp_of_match);

static struct spi_driver tran_fp_driver = {
	.driver = {
		.name = "tran_fp",
		.owner = THIS_MODULE,
		.of_match_table = tran_fp_of_match,
	},
	.probe = tran_fp_probe,
	.remove = tran_fp_remove,
};

module_spi_driver(tran_fp_driver);
MODULE_LICENSE("GPL v2");
