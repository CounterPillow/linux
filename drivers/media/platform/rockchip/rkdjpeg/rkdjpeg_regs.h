#ifndef RKDJPEG_REGS_H_
#define RKDJPEG_REGS_H_

#include <linux/bits.h>

#define RKDJPEG_REG_ID			0x00
#define RKDJPEG_MASK_PROD_NUM		GENMASK(31, 16)
#define RKDJPEG_MASK_MAX_BIT_DEPTH	BIT(8)
#define RKDJPEG_MASK_MINOR_VER		GENMASK(7, 0)

#define RKDJPEG_REG_INT			0x04

/* Decoder has timed out.
 * Only valid when sw_dec_timeout_e is 1 */
#define RKDJPEG_MASK_INT_TIMEOUT	BIT(12)
/* Decoder has encountered an error in the input data */
#define RKDJPEG_MASK_INT_DEC_ERROR	BIT(11)
/* Decoder has encountered an error on the AXI bus */
#define RKDJPEG_MASK_INT_BUS_ERROR	BIT(10)
/* Output picture data is ready */
#define RKDJPEG_MASK_INT_READY		BIT(9)
/* Decoder has interrupts enabled */
#define RKDJPEG_MASK_INT_ENABLED	BIT(8)
/* For enabling the decoder interrupt */
#define RKDJPEG_MASK_INT_ENABLE_RAW	BIT(6)
/* Pulse high to send a soft reset
 * TRM says don't write 0 to this. */
#define RKDJPEG_MASK_INT_SOFTRST	BIT(5)
/* For enabling the timeout interrupt */
#define RKDJPEG_MASK_INT_TIMEOUT_EN	BIT(2)
/* Set this high to start the decode.
 * Hardware resets it once decoding finishes or aborts */
#define RKDJPEG_MASK_INT_DECODE_EN	BIT(0)


#define RKDJPEG_REG_PERF_WRK_CNT	0x9c

#endif /* RKDJPEG_REGS_H_ */
