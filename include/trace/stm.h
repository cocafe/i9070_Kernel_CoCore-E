/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson STM Trace driver
 *
 * Author: Pierre Peiffer <pierre.peiffer@stericsson.com> for ST-Ericsson.
 *         Philippe Langlais <philippe.langlais@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2.
 */

#ifndef STM_H
#define STM_H

#include <linux/types.h>

#define STM_DEV_NAME "stm"

/* One single channel mapping */
struct stm_channel {
	union {
		__u8  no_stamp8;
		__u16 no_stamp16;
		__u32 no_stamp32;
		__u64 no_stamp64;
	};
	union {
		__u8   stamp8;
		__u16 stamp16;
		__u32 stamp32;
		__u64 stamp64;
	};
};

/* Possible trace modes */
#define STM_SW_LOSSLESS	0 /* Software mode: lossless data but intrusive */
#define STM_HW_LOSSY	1 /* Hardware mode: lossy data but less intrusive */

/* Possible clock setting */
enum clock_div {
	STM_CLOCK_DIV2 = 0,
	STM_CLOCK_DIV4,
	STM_CLOCK_DIV6,
	STM_CLOCK_DIV8,
	STM_CLOCK_DIV10,
	STM_CLOCK_DIV12,
	STM_CLOCK_DIV14,
	STM_CLOCK_DIV16,
};

/* ioctl commands */
#define STM_CONNECTION            _IOW('t', 0, enum stm_connection_type)
#define STM_DISABLE               _IO('t', 1)
#define STM_GET_NB_MAX_CHANNELS   _IOR('t', 2, int)
#define STM_GET_NB_FREE_CHANNELS  _IOR('t', 3, int)
#define STM_GET_CHANNEL_NO        _IOR('t', 4, int)
#define STM_SET_CLOCK_DIV         _IOW('t', 5, enum clock_div)
#define STM_GET_CTRL_REG          _IOR('t', 6, int)
#define STM_ENABLE_SRC            _IOWR('t', 7, int)
#define STM_GET_FREE_CHANNEL      _IOW('t', 8, int)
#define STM_RELEASE_CHANNEL       _IOW('t', 9, int)
#define STM_SET_MODE              _IOWR('t', 10, int)
#define STM_GET_MODE              _IOR('t', 11, int)

enum stm_connection_type {
	STM_DISCONNECT = 0,
	STM_DEFAULT_CONNECTION = 1,
	STM_STE_MODEM_ON_MIPI34_NONE_ON_MIPI60 = 2,
	STM_STE_APE_ON_MIPI34_NONE_ON_MIPI60 = 3,
	STM_STE_MODEM_ON_MIPI34_APE_ON_MIPI60 = 4,
	STM_STE_MODEM_ON_MICROSD = 5,
	STM_STE_APE_ON_MICROSD = 6,
	STM_STE_INVALID_CONNECTION = 0xff
};

#ifdef __KERNEL__

struct stm_platform_data {
	u32        regs_phys_base;
	u32        channels_phys_base;
	u32        id_mask;
	u32        masters_enabled;
	const s16 *channels_reserved;
	int        channels_reserved_sz;
	int        (*stm_connection)(enum stm_connection_type);
};

/* Channels base address */
extern volatile struct stm_channel __iomem *stm_channels;

/* Provides stm_trace_XX() and stm_tracet_XX() trace API */
#define DEFLLTFUN(size) \
static inline void stm_trace_##size(int channel, __u##size data) \
{ \
	stm_channels[channel].no_stamp##size = data; \
} \
static inline void stm_tracet_##size(int channel, __u##size data) \
{ \
	stm_channels[channel].stamp##size = data; \
} \

DEFLLTFUN(8);
DEFLLTFUN(16);
DEFLLTFUN(32);
DEFLLTFUN(64);

/*
 * Trace a buffer on a given channel
 * with auto time stamping on the last byte(s) only
 */
int stm_trace_buffer_onchannel(int channel, const void *data, size_t length);
/*
 * Trace a buffer on a dynamically allocated channel
 * with auto time stamping on the last byte(s) only
 * Dynamic channel are allocated in the 128 highest channels
 */
int stm_trace_buffer(const void *data, size_t length);

/* printk equivalent for STM */
int stm_printk(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

#if defined(CONFIG_STM_PRINTK)
#define stm_dup_printk(buf, length) \
	stm_trace_buffer_onchannel(CONFIG_STM_PRINTK_CHANNEL, buf, length)

#else
static inline int stm_dup_printk(char *buf, size_t size)
{
	return 0;
}
#endif

#if defined(CONFIG_STM_TRACE_PRINTK)
static inline int stm_trace_printk_buf(
		unsigned long ip, const char *buf, size_t size)
{
	stm_trace_32(CONFIG_STM_TRACE_PRINTK_CHANNEL, ip);
	return stm_trace_buffer_onchannel(CONFIG_STM_TRACE_PRINTK_CHANNEL,
			buf, size);
}

static inline int stm_trace_bprintk_buf(
		unsigned long ip, const char *fmt, const void *buf, size_t size)
{
	stm_trace_64(CONFIG_STM_TRACE_BPRINTK_CHANNEL, ((u64)ip<<32)+(u32)fmt);
	return stm_trace_buffer_onchannel(CONFIG_STM_TRACE_PRINTK_CHANNEL,
			buf, size);
}
#else
static inline int stm_trace_printk_buf(
		unsigned long ip, const char *buf, size_t size)
{
	return 0;
}

static inline int stm_trace_bprintk_buf(
		unsigned long ip, const char *fmt, const void *buf, size_t size)
{
	return 0;
}
#endif

#if defined(CONFIG_STM_FTRACE)
static inline void stm_ftrace(unsigned long ip, unsigned long parent_ip)
{
	stm_tracet_64(CONFIG_STM_FTRACE_CHANNEL, (((__u64)ip)<<32) + parent_ip);
}
#else
static inline void stm_ftrace(unsigned long ip, unsigned long parent_ip)
{
}
#endif

#if defined(CONFIG_STM_CTX_SWITCH)
static inline void stm_sched_switch(u32 prev_pid, u8 prev_prio, u8 prev_state,
			u32 next_pid, u8 next_prio, u8 next_state, u32 next_cpu)
{
	stm_trace_64(CONFIG_STM_CTX_SWITCH_CHANNEL,
			(((__u64)prev_pid)<<32) + next_pid);
	stm_tracet_64(CONFIG_STM_CTX_SWITCH_CHANNEL, (((__u64)next_cpu)<<32)
			+ (prev_prio<<24) + (prev_state<<16)
			+ (next_prio<<8) + next_state);
}
#else
static inline void stm_sched_switch(u32 prev_pid, u8 prev_prio, u8 prev_state,
			u32 next_pid, u8 next_prio, u8 next_state, u32 next_cpu)
{
}
#endif

#if defined(CONFIG_STM_WAKEUP)
static inline void stm_sched_wakeup(u32 prev_pid, u8 prev_prio, u8 prev_state,
			u32 next_pid, u8 next_prio, u8 next_state, u32 next_cpu)
{
	stm_trace_64(CONFIG_STM_WAKEUP_CHANNEL,
			(((__u64)prev_pid)<<32) + next_pid);
	stm_tracet_64(CONFIG_STM_WAKEUP_CHANNEL, (((__u64)next_cpu)<<32)
			+ (prev_prio<<24) + (prev_state<<16)
			+ (next_prio<<8) + next_state);
}
#else
static inline void stm_sched_wakeup(u32 prev_pid, u8 prev_prio, u8 prev_state,
			u32 next_pid, u8 next_prio, u8 next_state, u32 next_cpu)
{
}
#endif

#if defined(CONFIG_STM_STACK_TRACE)
static inline void stm_stack_trace(unsigned long *callers)
{
	while (*(callers + 1) != ULONG_MAX) {
		stm_trace_32(CONFIG_STM_STACK_TRACE_CHANNEL, *callers++);
	}
	/* Time stamp the latest */
	stm_tracet_32(CONFIG_STM_STACK_TRACE_CHANNEL, *callers);
}
#else
static inline void stm_stack_trace(unsigned long *callers)
{
}
#endif

/* Alloc/Free STM channel */
int stm_alloc_channel(int offset);
void stm_free_channel(int channel);

#endif /* __KERNEL__ */

#endif /* STM_H */
