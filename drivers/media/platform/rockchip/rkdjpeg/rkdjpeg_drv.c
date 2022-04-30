// SPDX-License-Identifier: GPL-2.0

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "rkdjpeg.h"
#include "rkdjpeg_regs.h"

#define DRIVER_NAME "rkdjpeg"

#define DEBUG

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

	rkdj->hclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(rkdj->hclk)) {
		ret = dev_err_probe(rkdj->dev, PTR_ERR(rkdj->hclk),
				    "Failed to get clock hclk\n");
		goto err_disable_hclk;
	}

	ret = clk_prepare_enable(rkdj->hclk);
	if (ret) {
		ret = dev_err_probe(rkdj->dev, ret,
				    "Failed to enable clock hclk\n");
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

	/* Enable timeout interrupt */
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_ENABLE_RAW,
				RKDJPEG_MASK_INT_ENABLE_RAW);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"couldn't write to the interrupt control register: %d\n",
			ret);
		goto err_disable_aclk;
	}

	int prod_num, bit_depth, minor_ver;
	ret = rkdjpeg_get_hw_version(rkdj, &prod_num, &bit_depth, &minor_ver);
	if (ret < 0) {
		dev_err(rkdj->dev, "Actual error! %d\n", ret);
		goto err_disable_timeout_int;
	}
	dev_err(rkdj->dev, "Initialised version=%#4x minor=%#2x max bit depth=%d\n",
		prod_num, minor_ver, bit_depth);

	return 0;

err_disable_timeout_int:
	ret = regmap_write_bits(rkdj->regmap, RKDJPEG_REG_INT,
				RKDJPEG_MASK_INT_ENABLE_RAW, 0);
err_disable_aclk:
	clk_disable_unprepare(rkdj->aclk);
err_disable_hclk:
	clk_disable_unprepare(rkdj->hclk);

	return ret;
}

static int rkdjpeg_remove(struct platform_device *pdev)
{
	struct rkdjpeg_dev *rkdj = platform_get_drvdata(pdev);
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
