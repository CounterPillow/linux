// SPDX-License-Identifier: GPL-2.0
#define DEBUG

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>

#include "rkdjpeg.h"
#include "rkdjpeg_regs.h"

#define DRIVER_NAME "rkdjpeg"

static int rkdjpeg_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "rkdjpeg JPEG decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" DRIVER_NAME, sizeof(cap->bus_info));

	return 0;
}

static const struct v4l2_ioctl_ops rkdjpeg_ioctl_ops = {
	.vidioc_querycap	= rkdjpeg_querycap,
};

static int rkdjpeg_open(struct file *file)
{
	/* FIXME: implement this. Should open a v4l2 m2m context */
	/* Also enable the clocks and set up the hardware here? */
	return 0;
}

static int rkdjpeg_release(struct file *file)
{
	/* FIXME: implement this.
	 *  - end any currently ongoing stream
	 *  - free the context
	 *  - uninit the hardware
	 *  - stop the clocks
	 */
	return 0;
}

static const struct v4l2_file_operations rkdjpeg_fops = {
	.owner		= THIS_MODULE,
	.open		= rkdjpeg_open,
	.release	= rkdjpeg_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

int rkdjpeg_register_video_dev(struct rkdjpeg_dev *rkdj)
{
	struct video_device *vfd;
	int ret;

	vfd = devm_kzalloc(rkdj->dev, sizeof(*vfd), GFP_KERNEL);
	if (!vfd) {
		dev_dbg(rkdj->dev, "alive in error path\n");
		v4l2_err(&rkdj->v4l2_dev, "Failed to allocate video device\n");
		return -ENOMEM;
	}

	vfd->fops = &rkdjpeg_fops;
	vfd->ioctl_ops = &rkdjpeg_ioctl_ops;
	vfd->release = video_device_release_empty;	/* FIXME: check if correct */
	vfd->lock = &rkdj->rkdjpeg_mutex;
	vfd->v4l2_dev = &rkdj->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	video_set_drvdata(vfd, rkdj);

	rkdj->vfd = vfd;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&rkdj->v4l2_dev, "Failed to register video device\n");
		return ret;
	}

	return 0;
}

irqreturn_t rkdjpeg_jpeg_irq_handler(int irq, void *data)
{
	struct rkdjpeg_dev *rkdj = (struct rkdjpeg_dev*) data;
	int ret;
	unsigned int intr;

	ret = regmap_read(rkdj->regmap, RKDJPEG_REG_INT, &intr);
	if (ret) {
		dev_err(rkdj->dev, "Interrupt register read error: %d\n", ret);
		goto reset_int_and_skedaddle;
	}

	/* TODO: handle timeout interrupt */
	/* TODO: handle input data error interrupt */
	/* TODO: handle AXI bus error interrupt */
	/* TODO: handle picture ready interrupt */

reset_int_and_skedaddle:
	/* Reset sw_dec_irq_raw, as the TRM says */
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_ENABLE_RAW,
				RKDJPEG_MASK_INT_ENABLE_RAW);
	if (ret < 0)
		dev_err(rkdj->dev,
			"resetting interrupt enable register failed, this is very bad\n");
	return IRQ_HANDLED;
}

static int rkdjpeg_get_hw_version(struct rkdjpeg_dev *rkdj, int* version,
				  int* bit_depth, int* minor_version)
{
	int ret;
	int val;

	ret = regmap_read(rkdj->regmap, RKDJPEG_REG_ID, &val);
	if (ret)
		return ret;

	*version = val & RKDJPEG_MASK_PROD_NUM;
	*bit_depth = (val & RKDJPEG_MASK_MAX_BIT_DEPTH) ? 12 : 8;
	*minor_version = val & RKDJPEG_MASK_MINOR_VER;

	return 0;
}

static bool rkdjpeg_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RKDJPEG_REG_ID:
		return false;
	default:
		return true;
	}
}

static const struct regmap_config rkdjpeg_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RKDJPEG_REG_PERF_WRK_CNT,
	.writeable_reg = rkdjpeg_writeable_reg,
};

static const struct of_device_id of_rkdjpeg_match[] = {
	{ .compatible = "rockchip,rk3568-rkdjpeg"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_rkdjpeg_match);

int rkdjpeg_init_hw(struct rkdjpeg_dev *rkdj)
{
	int ret;
	/* Enable interrupt */
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_ENABLE_RAW,
				RKDJPEG_MASK_INT_ENABLE_RAW);
	if (ret < 0) {
		dev_err(rkdj->dev,
			"couldn't write to the interrupt control register: %d\n",
			ret);
		return ret;
	}

	/* Enable timeout */
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_TIMEOUT_EN,
				RKDJPEG_MASK_INT_TIMEOUT_EN);
	if (ret < 0) {
		dev_err(rkdj->dev,
			"couldn't enable the timeout interrupt: %d\n", ret);
		return ret;
	}

	return 0;
}

int rkdjpeg_uninit_hw(struct rkdjpeg_dev *rkdj)
{
	int ret;

	/* Disable timeout */
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_TIMEOUT_EN, 0);
	if (ret < 0) {
		dev_err(rkdj->dev,
			"couldn't disable the timeout interrupt: %d\n", ret);
		return ret;
	}

	/* Disable interrupt */
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_ENABLE_RAW, 0);
	if (ret < 0) {
		dev_err(rkdj->dev,
			"couldn't write to the interrupt control register: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int rkdjpeg_probe(struct platform_device *pdev)
{
	struct rkdjpeg_dev *rkdj;
	struct resource *res;
	void __iomem *regs;
	int irq, ret;

	rkdj = devm_kzalloc(&pdev->dev, sizeof(*rkdj), GFP_KERNEL);
	if (!rkdj)
		return -ENOMEM;

	rkdj->dev = &pdev->dev;
	rkdj->pdev = pdev;
	mutex_init(&rkdj->rkdjpeg_mutex);

	platform_set_drvdata(pdev, rkdj);

	rkdj->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(rkdj->hclk)) {
		return dev_err_probe(rkdj->dev, PTR_ERR(rkdj->hclk),
				     "Failed to get clock hclk\n");
	}

	ret = clk_prepare_enable(rkdj->hclk);
	if (ret) {
		return dev_err_probe(rkdj->dev, ret,
				     "Failed to enable clock hclk\n");
	}

	rkdj->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(rkdj->aclk)) {
		ret = dev_err_probe(rkdj->dev, PTR_ERR(rkdj->aclk),
				    "Failed to get clock aclk\n");
		goto err_disable_hclk;
	}

	ret = clk_prepare_enable(rkdj->aclk);
	if (ret) {
		ret = dev_err_probe(rkdj->dev, ret,
				    "Failed to enable clock aclk\n");
		goto err_disable_hclk;
	}

	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs)) {
		ret = dev_err_probe(rkdj->dev, PTR_ERR(regs),
				    "Failed to get resource IORESOURCE_MEM\n");
		goto err_disable_aclk;
	}

	rkdj->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					     &rkdjpeg_regmap_config);
	if (IS_ERR(rkdj->regmap)) {
		ret = dev_err_probe(rkdj->dev, PTR_ERR(rkdj->regmap),
				    "Failed to initialise regmap\n");
		goto err_disable_aclk;
	}

	/* JPEG IRQ */
	irq = platform_get_irq_byname(pdev, "jpeg");
	if (irq < 0) {
		dev_err(&pdev->dev, "error retrieving 'jpeg' interrupt: %d\n",
			irq);
		ret = irq;
		goto err_disable_aclk;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					rkdjpeg_jpeg_irq_handler, IRQF_ONESHOT,
					DRIVER_NAME "-jpeg", rkdj);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request jpeg irq\n");
		goto err_disable_aclk;
	}

	ret = v4l2_device_register(&pdev->dev, &rkdj->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2 device\n");
		goto err_disable_aclk;
	}

	ret = rkdjpeg_register_video_dev(rkdj);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register video device (%d)\n", ret);
		goto err_unregister_video_dev;
	}

	ret = rkdjpeg_init_hw(rkdj);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialise hardware (%d)\n", ret);
		/* intentionally try to undo all the reg writes here */
		goto err_uninit_hw;
	}

	int prod_num, bit_depth, minor_ver;
	ret = rkdjpeg_get_hw_version(rkdj, &prod_num, &bit_depth, &minor_ver);
	if (ret < 0) {
		dev_err(rkdj->dev, "Actual error! %d\n", ret);
		goto err_uninit_hw;
	}
	dev_dbg(rkdj->dev, "Initialised version=%#4x minor=%#2x max bit depth=%d\n",
		prod_num, minor_ver, bit_depth);

	return 0;

err_uninit_hw:
	ret = rkdjpeg_uninit_hw(rkdj);
	if (ret)
		dev_err(rkdj->dev, "uninitialising hardware failed (%d)\n", ret);

err_unregister_video_dev:
	video_unregister_device(rkdj->vfd);
	v4l2_device_unregister(&rkdj->v4l2_dev);

err_disable_aclk:
	clk_disable_unprepare(rkdj->aclk);

err_disable_hclk:
	clk_disable_unprepare(rkdj->hclk);

	return ret;
}

static int rkdjpeg_remove(struct platform_device *pdev)
{
	struct rkdjpeg_dev *rkdj = platform_get_drvdata(pdev);
	int ret;

	video_unregister_device(rkdj->vfd);
	v4l2_device_unregister(&rkdj->v4l2_dev);

	ret = rkdjpeg_uninit_hw(rkdj);

	clk_disable_unprepare(rkdj->aclk);
	clk_disable_unprepare(rkdj->hclk);

	return 0;
}

static struct platform_driver rkdjpeg_driver = {
	.probe = rkdjpeg_probe,
	.remove = rkdjpeg_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = of_match_ptr(of_rkdjpeg_match),
	},
};
module_platform_driver(rkdjpeg_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nicolas Frattaroli <frattaroli.nicolas@gmail.com>");
MODULE_DESCRIPTION("Rockchip JPEG decoder driver");
