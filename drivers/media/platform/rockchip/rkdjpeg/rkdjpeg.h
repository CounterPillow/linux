#ifndef RKDJPEG_H_
#define RKDJPEG_H_

#include "media/v4l2-device.h"
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>

struct rkdjpeg_dev {
	struct platform_device *pdev;
	struct device *dev;
	struct regmap *regmap;
	struct clk *hclk;
	struct clk *aclk;
	struct v4l2_device v4l2_dev;
	struct video_device *vfd;

	struct mutex rkdjpeg_mutex;	/* video_device lock */
};

#endif /* RKDJPEG_H_ */
