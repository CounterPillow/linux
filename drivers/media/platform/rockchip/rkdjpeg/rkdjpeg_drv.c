// SPDX-License-Identifier: GPL-2.0
#define DEBUG

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "rkdjpeg.h"
#include "rkdjpeg_regs.h"

#define DRIVER_NAME "rkdjpeg"

static const u32 rkdjpeg_rk3568_cap_fmts[] = {
	V4L2_PIX_FMT_NV12M,
	V4L2_PIX_FMT_NV16M,
	V4L2_PIX_FMT_NV24M
};

static const u32 rkdjpeg_rk3568_out_fmts[] = {
	V4L2_PIX_FMT_JPEG,
	V4L2_PIX_FMT_MJPEG
};

static const struct rkdjpeg_variant rkdjpeg_rk3568_variant = {
	.cap_fmts = rkdjpeg_rk3568_cap_fmts,
	.num_cap_fmts = ARRAY_SIZE(rkdjpeg_rk3568_cap_fmts),
	.out_fmts = rkdjpeg_rk3568_out_fmts,
	.num_out_fmts = ARRAY_SIZE(rkdjpeg_rk3568_out_fmts),
	.mpps = 240,
	.min_width = 48,
	.min_height = 48,
	.max_width = 65536,
	.max_height = 65536,
	.step_size = 8
};

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

static int rkdjpeg_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "rkdjpeg JPEG decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" DRIVER_NAME, sizeof(cap->bus_info));

	return 0;

}

static int rkdjpeg_enum_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	struct rkdjpeg_ctx *ctx =
		container_of(file->private_data, struct rkdjpeg_ctx, fh);
	struct rkdjpeg_dev *rkdj = ctx->rkdj;
	int i;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return -EINVAL;
	}

	for (i = 0; i < rkdj->variant->num_cap_fmts; i++) {
		if (f->index == i) {
			f->pixelformat = rkdj->variant->cap_fmts[i];
			return 0;
		}
	}

	return -EINVAL;
}

static int rkdjpeg_enum_fmt_vid_out(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	struct rkdjpeg_ctx *ctx =
		container_of(file->private_data, struct rkdjpeg_ctx, fh);
	struct rkdjpeg_dev *rkdj = ctx->rkdj;
	int i;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		return -EINVAL;
	}

	for (i = 0; i < rkdj->variant->num_out_fmts; i++) {
		if (f->index == i) {
			f->pixelformat = rkdj->variant->out_fmts[i];
			return 0;
		}
	}

	return -EINVAL;
}

static bool rkdjpeg_format_supported(struct rkdjpeg_dev *rkdj, bool capture,
				     bool output, u32 pixel_format)
{
	int i;

	if (capture) {
		for (i = 0; i < rkdj->variant->num_cap_fmts; i++) {
			if (pixel_format == rkdj->variant->cap_fmts[i])
				return true;
		}
	}
	if (output) {
		for (i = 0; i < rkdj->variant->num_out_fmts; i++) {
			if (pixel_format == rkdj->variant->out_fmts[i])
				return true;
		}
	}

	return false;
}

static int rkdjpeg_enum_framesizes(struct file *file, void *priv,
				   struct v4l2_frmsizeenum *fsize)
{
	struct rkdjpeg_ctx *ctx =
		container_of(file->private_data, struct rkdjpeg_ctx, fh);
	struct rkdjpeg_dev *rkdj = ctx->rkdj;

	if (fsize->index != 0)
		return -EINVAL;

	if (!rkdjpeg_format_supported(rkdj, true, true, fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = rkdj->variant->min_width;
	fsize->stepwise.min_height = rkdj->variant->min_height;
	fsize->stepwise.max_width = rkdj->variant->max_width;
	fsize->stepwise.max_height = rkdj->variant->max_height;
	fsize->stepwise.step_width = rkdj->variant->step_size;
	fsize->stepwise.step_height = rkdj->variant->step_size;

	return 0;
}

static const struct v4l2_ioctl_ops rkdjpeg_ioctl_ops = {
	.vidioc_querycap		= rkdjpeg_querycap,
	.vidioc_enum_fmt_vid_cap	= rkdjpeg_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= rkdjpeg_enum_fmt_vid_out,
	.vidioc_enum_framesizes		= rkdjpeg_enum_framesizes,
};

static int rkdjpeg_enable_clocks(struct rkdjpeg_dev *rkdj)
{
	int ret;

	ret = clk_enable(rkdj->hclk);
	if (ret) {
		dev_err(rkdj->dev, "Failed to enable clock hclk: %d\n", ret);
		return ret;
	}

	ret = clk_enable(rkdj->aclk);
	if (ret) {
		dev_err(rkdj->dev, "Failed to enable clock aclk: %d\n", ret);
		/* Disable the previously enabled hclk in this error path */
		clk_disable(rkdj->hclk);
	}

	return ret;
}

static void rkdjpeg_disable_clocks(struct rkdjpeg_dev *rkdj)
{
	clk_disable(rkdj->aclk);
	clk_disable(rkdj->hclk);
}

const struct vb2_ops rkdjpeg_queue_ops = {
};

static int
queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct rkdjpeg_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &rkdjpeg_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;

	src_vq->buf_struct_size = sizeof(struct rkdjpeg_src_buf);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->rkdj->rkdjpeg_mutex;
	src_vq->dev = ctx->rkdj->v4l2_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &rkdjpeg_queue_ops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->rkdj->rkdjpeg_mutex;
	dst_vq->dev = ctx->rkdj->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

static int rkdjpeg_open(struct file *file)
{
	struct rkdjpeg_dev *rkdj = video_get_drvdata(video_devdata(file));
	struct video_device *vdev = video_devdata(file);
	struct rkdjpeg_ctx *ctx;
#ifdef DEBUG
	int prod_num, bit_depth, minor_ver;
#endif
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->rkdj = rkdj;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(rkdj->m2m_dev, ctx, queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctx_free;
	}

	v4l2_fh_init(&ctx->fh, vdev);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	rkdjpeg_enable_clocks(rkdj);
	ret = rkdjpeg_init_hw(rkdj);
	if (ret < 0) {
		dev_err(rkdj->dev, "failed to initialise hardware (%d)\n", ret);
		/* intentionally try to undo all the reg writes here */
		goto err_disable_clk_and_hw;
	}

#ifdef DEBUG
	ret = rkdjpeg_get_hw_version(rkdj, &prod_num, &bit_depth, &minor_ver);
	if (ret < 0) {
		dev_err(rkdj->dev, "Error getting hw version info: %d\n", ret);
		goto err_disable_clk_and_hw;
	}
	dev_dbg(rkdj->dev, "Initialised version=%#4x minor=%#2x max bit depth=%d\n",
		prod_num, minor_ver, bit_depth);
#endif

	return 0;

err_disable_clk_and_hw:
	rkdjpeg_uninit_hw(rkdj);
	rkdjpeg_disable_clocks(rkdj);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
err_ctx_free:
	kfree(ctx);
	return ret;
}

static int rkdjpeg_release(struct file *file)
{
	struct rkdjpeg_dev *rkdj = video_get_drvdata(video_devdata(file));
	int ret;
	struct rkdjpeg_ctx *ctx =
		container_of(file->private_data, struct rkdjpeg_ctx, fh);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	/* FIXME: implement this.
	 *  - end any currently ongoing stream
	 *  - control handler stuff?
	 */
	ret = rkdjpeg_uninit_hw(rkdj);
	if (ret < 0) {
		dev_err(rkdj->dev, "failed to uninitialise hardware (%d)\n", ret);
	};

	rkdjpeg_disable_clocks(rkdj);

	return ret;
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
	vfd->release = video_device_release;	/* FIXME: check if correct */
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
	{ .compatible = "rockchip,rk3568-rkdjpeg", .data = &rkdjpeg_rk3568_variant},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_rkdjpeg_match);

static int rkdjpeg_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct rkdjpeg_dev *rkdj;
	struct resource *res;
	void __iomem *regs;
	int irq, ret;

	rkdj = devm_kzalloc(&pdev->dev, sizeof(*rkdj), GFP_KERNEL);
	if (!rkdj)
		return -ENOMEM;

	rkdj->dev = &pdev->dev;
	rkdj->pdev = pdev;

	of_id = of_match_device(of_rkdjpeg_match, &pdev->dev);
	if (!of_id)
		return -EINVAL;

	rkdj->variant = (struct rkdjpeg_variant *)of_id->data;

	mutex_init(&rkdj->rkdjpeg_mutex);

	platform_set_drvdata(pdev, rkdj);

	rkdj->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(rkdj->hclk)) {
		return dev_err_probe(rkdj->dev, PTR_ERR(rkdj->hclk),
				     "Failed to get clock hclk\n");
	}

	ret = clk_prepare(rkdj->hclk);
	if (ret) {
		return dev_err_probe(rkdj->dev, ret,
				     "Failed to prepare clock hclk\n");
	}

	rkdj->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(rkdj->aclk)) {
		ret = dev_err_probe(rkdj->dev, PTR_ERR(rkdj->aclk),
				    "Failed to get clock aclk\n");
		goto err_disable_hclk;
	}

	ret = clk_prepare(rkdj->aclk);
	if (ret) {
		ret = dev_err_probe(rkdj->dev, ret,
				    "Failed to prepare clock aclk\n");
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

	/*
	 * Cargo-culted from Hantro. Hardware can only handle 32 bit
	 * addresses, so I think this should appropriately restrict it
	 */
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Could not set DMA coherent mask.\n");
		return ret;
	}
	vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));

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

	return 0;

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
