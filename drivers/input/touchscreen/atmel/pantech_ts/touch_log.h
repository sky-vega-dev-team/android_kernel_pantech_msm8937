#include <linux/proc_fs.h>

typedef struct {
	int         size;   /* maximum number of elements           */
	int         start;  /* index of oldest element              */
	int         end;    /* index at which to write new element  */
	char *elems;  /* vector of elements                   */
} CircularBuffer;

void cbInit(CircularBuffer *cb, int size) {
	cb->size  = size + 1; /* include empty elem */
	cb->start = 0;
	cb->end   = 0;
	cb->elems = (char*)kmalloc(cb->size * sizeof(char), GFP_KERNEL | GFP_ATOMIC);
}

void cbFree(CircularBuffer *cb) {
	kfree(cb->elems); /* OK if null */ 
}

int cbIsFull(CircularBuffer *cb) {
	return (cb->end + 1) % cb->size == cb->start; 
}

int cbIsEmpty(CircularBuffer *cb) {
	return cb->end == cb->start; 
}

/* Write an element, overwriting oldest element if buffer is full. App can
 *    choose to avoid the overwrite by checking cbIsFull(). */
void cbWrite(CircularBuffer *cb, char *elem) {
	cb->elems[cb->end] = *elem;
	cb->end = (cb->end + 1) % cb->size;
	if (cb->end == cb->start)
		cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
}

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void cbRead(CircularBuffer *cb, char *elem) {
	*elem = cb->elems[cb->start];
	cb->start = (cb->start + 1) % cb->size;
}

CircularBuffer cb;
spinlock_t cb_spinlock;

int read_log(char *page, char **start, off_t off, int count, int *eof, void *data_unused) {
	char *buf;
	char elem = {0};
	buf = page;


	spin_lock(&cb_spinlock);
	while (!cbIsEmpty(&cb)) {
		cbRead(&cb, &elem);
		buf += sprintf(buf, &elem);
	}
	spin_unlock(&cb_spinlock);
	*eof = 1;
	return buf - page;
}

char touch_info_vendor[] = "atmel";
char touch_info_chipset[] = "mxt641t";
char touch_info_modelID[] = "ef71";

static ssize_t read_touch_info(struct file *file, char __user *buff, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
   	char buf[128] = {0,};

	sprintf(buf, "Vendor: \t%s\nChipset: \t%s\nModelID: \t%s\n", touch_info_vendor, touch_info_chipset, touch_info_modelID);

    ret = simple_read_from_buffer(buff, count, ppos, buf, 128);

	return ret;
}

char printproc_buf[1024];

void printp(const char *fmt, ...) {
	int count = 0;
	int i;
	va_list args;
	spin_lock(&cb_spinlock);
	va_start(args, fmt);
	count += vsnprintf(printproc_buf, 1024, fmt, args);
	for (i = 0; i<count; i++) {
		cbWrite(&cb, &printproc_buf[i]);
	}
	va_end(args);
	spin_unlock(&cb_spinlock);
}

static const struct file_operations touchinfo_fops = {
	.read = read_touch_info,
};

void init_proc(void) { 
/*
	int testBufferSize = 1024;

	struct proc_dir_entry *touch_log_ent;
	struct proc_dir_entry *touch_info_ent;
	
	touch_log_ent = create_proc_entry("touchlog", S_IFREG|S_IRUGO, 0); 
	touch_log_ent->read_proc = read_log;
	
	touch_info_ent = create_proc_entry("touchinfo", S_IFREG|S_IRUGO, 0); 
	touch_info_ent->read_proc = read_touch_info;

	spin_lock_init(&cb_spinlock);
	cbInit(&cb, testBufferSize);
*/

    proc_create("touchinfo", S_IFREG|S_IRUGO, NULL, &touchinfo_fops);

}

void remove_proc(void) {
	remove_proc_entry("touchlog", 0);
	remove_proc_entry("touchinfo", 0);
	cbFree(&cb);
}
