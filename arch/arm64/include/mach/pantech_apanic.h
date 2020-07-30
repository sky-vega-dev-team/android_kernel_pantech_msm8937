#ifndef PANTECH_APANIC_H
#define PANTECH_APANIC_H

#include <linux/sched.h>

#define SCHED_LOG_MAX 1024
#define PANTECH_INFO_MAGIC 0xdead1004

// (+) p16652 add for logcat parser
struct msg_log {
	void *kernellog_buf_adr;
	void *mainlogcat_buf_adr;
    void *mainlogcat_w_off;
    uint32_t mainlogcat_size;
	void *systemlogcat_buf_adr;
    void *systemlogcat_w_off;
    uint32_t systemlogcat_size;
};
struct sched_log {
	unsigned long long time;
	char comm[TASK_COMM_LEN];
	pid_t pid;
};
struct irq_log {
	unsigned int softirq_vec;
	unsigned int func;
	unsigned long long start_time;
	unsigned long long elapsed_time;
	int irqs_flag;
};
extern void pantech_force_dump_key(unsigned int code, int value);
#endif

