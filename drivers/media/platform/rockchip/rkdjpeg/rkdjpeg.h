#ifndef _RKDJPEG_H
#define _RKDJPEG_H

#include "media/v4l2-device.h"
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>

#define RKDJPEG_FMT_FLAG_OUTPUT		BIT(0)
#define RKDJPEG_FMT_FLAG_CAPTURE	BIT(1)

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

/**
 * struct rkdjpeg_variant - Implementation variant of the rkdjpeg hardware
 * @cap_fmts:		decoder capture (decoded data out) formats
 * @num_cap_fmts;	number of cap_fmts
 * @out_fmts;		decoder output (coded data in) formats
 * @num_out_fmts;	number of out_fmts
 * @mpps;		approximate decoder speed in million pixels per second
 * @min_width;		minimum image width the hardware can decode
 * @min_height;		minimum image height the hardware can decode
 * @max_width;		maximum image width the hardware can decode
 * @max_height;		maximum image height the hardware can decode
 * @step_size;		decoder supported resolution step size
 */
struct rkdjpeg_variant {
	const u32 *cap_fmts;
	const u32 num_cap_fmts;
	const u32 *out_fmts;
	const u32 num_out_fmts;
	const int mpps;
	const int min_width;
	const int min_height;
	const int max_width;
	const int max_height;
	const int step_size;
};

#endif /* _RKDJPEG_H */
