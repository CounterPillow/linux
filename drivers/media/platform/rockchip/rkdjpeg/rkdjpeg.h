#ifndef RKDJPEG_H_
#define RKDJPEG_H_

#include <linux/clk.h>
#include <linux/platform_device.h>

struct rkdjpeg_dev {
	struct platform_device *pdev;
	struct device *dev;
	struct regmap *regmap;
	struct clk *hclk;
	struct clk *aclk;
};

#endif /* RKDJPEG_H_ */
