#ifndef PANTECH_DEBUG_H
#define PANTECH_DEBUG_H

#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/mm_types.h>

extern int pantech_debug_enable;
extern int pantech_debug_init(void);

#define PANTECH_MAX_STACKFRAME_DEPTH 8
#define PANTECH_DBG_LOG_BUF_SIZE 0x100000 

#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG 
#define SCHED_LOG_MAX 1024*2
struct sched_log {
	unsigned long long time;
	char comm[TASK_COMM_LEN];
	pid_t pid;
};
#endif
#ifdef CONFIG_PANTECH_DEBUG_IRQ_LOG 
struct irq_log {
	unsigned long long time;
	unsigned long long end_time;
	unsigned long long elapsed_time;
	int irq;
	void *fn;
	int en;
	int preempt_count;
	void *context;
};
#endif
#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  
#define DCVS_LOG_MAX 512
struct dcvs_debug {
	unsigned long long time;
	int cpu_no;
	unsigned int prev_freq;
	unsigned int new_freq;
};
#endif

#define RPM_LOG_MAX 512
struct rpm_debug {
	unsigned long long time;
    unsigned int msgid;       // msgid
    unsigned int set;       // 0: active set, 1: sleep set
    unsigned int restype;   // resource type
    unsigned int resid;     // resource id
    unsigned int key;       // kvp key
    unsigned int value;     // kvp value
    int errno;              // errno : 0 is ok.
};

// ## you must have same structures in rpm-smd.c ##
struct rpm_request_header_debug {
	uint32_t service_type;
	uint32_t request_len;
};

enum msm_rpm_set_debug {
	MSM_RPM_CTX_ACTIVE_SET_DEBUG,
	MSM_RPM_CTX_SLEEP_SET_DEBUG,
};

struct rpm_message_header_debug {
	uint32_t msg_id;
	enum msm_rpm_set_debug set;
	uint32_t resource_type;
	uint32_t resource_id;
	uint32_t data_len;
};

struct msm_rpm_kvp_data_debug {
	uint32_t key;
	uint32_t nbytes; /* number of bytes */
	uint8_t *value;
	bool valid;
};

struct msm_rpm_request_debug {
	struct rpm_request_header_debug req_hdr;
	struct rpm_message_header_debug msg_hdr;
	struct msm_rpm_kvp_data_debug *kvp;
	uint32_t num_elements;
	uint32_t write_idx;
	uint8_t *buf;
	uint32_t numbytes;
};



#define PAGEALLOC_LOG_MAX 1024*2
struct pagealloc_debug {
	unsigned long long time;
	unsigned int was_free;
	unsigned int caller[PANTECH_MAX_STACKFRAME_DEPTH];
	unsigned int alloc_order;
	unsigned int free_order;
	unsigned int zone;
	unsigned int high_free;
	unsigned int low_free;
	struct page *page;
};


#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  
extern void pantech_debug_task_sched_log_short_msg(char *msg);
extern void pantech_debug_task_sched_log(int cpu, struct task_struct *task);
extern void pantech_debug_sched_log_init(void);
#define pantechdbg_sched_msg(fmt, ...) \
	do { \
		char ___buf[16]; \
		snprintf(___buf, sizeof(___buf), fmt, ##__VA_ARGS__); \
		pantech_debug_task_sched_log_short_msg(___buf); \
	} while (0)
#else
static inline void pantech_debug_task_sched_log(int cpu, struct task_struct *task)
{
}
static inline void pantech_debug_sched_log_init(void)
{
}
#define pantechdbg_sched_msg(fmt, ...)
#endif

#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  
extern void pantech_debug_irq_sched_log(unsigned int irq, void *fn, int en, unsigned long long start_time);
#else
static inline void pantech_debug_irq_sched_log(unsigned int irq, void *fn, int en)
{
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  
extern void pantech_debug_dcvs_log(int cpu_no, unsigned int prev_freq,
			unsigned int new_freq);
#else
static inline void pantech_debug_dcvs_log(int cpu_no, unsigned int prev_freq,
					unsigned int new_freq)
{
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_RPM_LOG 
extern void pantech_debug_rpm_ack_log(unsigned int msgid, int errno);
extern void pantech_debug_rpm_req_log(struct msm_rpm_request_debug *cdata);
#else
static inline void pantech_debug_rpm_ack_log(unsigned int msgid, int errno)
{
}
static inline void pantech_debug_rpm_req_log(struct msm_rpm_request_debug *cdata)
{
}
#endif

extern void pantech_debug_pagealloc_log(struct page *p, unsigned int order, unsigned int zone, 
	unsigned int high_free, unsigned int low_free, unsigned int was_free);

#endif

