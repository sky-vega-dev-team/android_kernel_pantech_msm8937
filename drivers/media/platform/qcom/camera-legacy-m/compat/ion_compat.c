// SPDX-License-Identifier: GPL-2.0-only
/*
 * /dev/ion for the vendor camera stack: the 2016 mm-camera daemon and the
 * QCamera2 HAL allocate their buffers through ION. Mainline has no ION, so
 * serve the ioctls they use on top of plain page-backed dma-bufs:
 *
 *  - the pre-4.12 ABI with handles: ALLOC, FREE, MAP, SHARE, IMPORT and
 *    the msm cache operations through CUSTOM;
 *  - the 4.12+ ABI where ALLOC hands back a dma-buf fd and HEAP_QUERY
 *    lists the heaps.
 *
 * The heap masks are ignored: every buffer comes from the page allocator,
 * and the IOMMUs in front of the camera blocks map it. Buffers without
 * ION_FLAG_CACHED are mapped write-combined into userspace, as ION did.
 */
#include <linux/compat.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#define ION_FLAG_CACHED		1

#define ION_IOC_MAGIC		'I'
#define ION_IOC_MSM_MAGIC	'M'

/* pre-4.12 ABI */
typedef int ion_user_handle_t;

struct ion_allocation_data_legacy {
	size_t len;
	size_t align;
	unsigned int heap_id_mask;
	unsigned int flags;
	ion_user_handle_t handle;
};

struct ion_fd_data {
	ion_user_handle_t handle;
	int fd;
};

struct ion_handle_data {
	ion_user_handle_t handle;
};

struct ion_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

struct ion_flush_data {
	ion_user_handle_t handle;
	int fd;
	void __user *vaddr;
	unsigned int offset;
	unsigned int length;
};

/* 4.12+ ABI */
struct ion_allocation_data_new {
	__u64 len;
	__u32 heap_id_mask;
	__u32 flags;
	__u32 fd;
	__u32 unused;
};

struct ion_heap_data {
	char name[32];
	__u32 type;
	__u32 heap_id;
	__u32 reserved0;
	__u32 reserved1;
	__u32 reserved2;
};

struct ion_heap_query {
	__u32 cnt;
	__u32 reserved0;
	__u64 heaps;
	__u32 reserved1;
	__u32 reserved2;
};

#define ION_IOC_ALLOC_NR	0
#define ION_IOC_FREE		_IOWR(ION_IOC_MAGIC, 1, struct ion_handle_data)
#define ION_IOC_MAP		_IOWR(ION_IOC_MAGIC, 2, struct ion_fd_data)
#define ION_IOC_SHARE		_IOWR(ION_IOC_MAGIC, 4, struct ion_fd_data)
#define ION_IOC_IMPORT		_IOWR(ION_IOC_MAGIC, 5, struct ion_fd_data)
#define ION_IOC_CUSTOM_NR	6
#define ION_IOC_SYNC		_IOWR(ION_IOC_MAGIC, 7, struct ion_fd_data)
#define ION_IOC_HEAP_QUERY	_IOWR(ION_IOC_MAGIC, 8, struct ion_heap_query)

#define ION_IOC_CLEAN_CACHES_NR		0
#define ION_IOC_INV_CACHES_NR		1
#define ION_IOC_CLEAN_INV_CACHES_NR	2

#ifdef CONFIG_COMPAT
struct compat_ion_allocation_data_legacy {
	compat_size_t len;
	compat_size_t align;
	compat_uint_t heap_id_mask;
	compat_uint_t flags;
	compat_int_t handle;
};

struct compat_ion_custom_data {
	compat_uint_t cmd;
	compat_ulong_t arg;
};

struct compat_ion_flush_data {
	compat_int_t handle;
	compat_int_t fd;
	compat_uptr_t vaddr;
	compat_uint_t offset;
	compat_uint_t length;
};
#endif

struct ionc_buffer {
	struct mutex lock;
	struct list_head attachments;
	size_t len;
	bool cached;
	struct sg_table table;
	struct page **pages;
	unsigned int npages;
	void *vaddr;
	int vmap_cnt;
};

struct ionc_attachment {
	struct device *dev;
	struct sg_table table;
	struct list_head list;
	bool mapped;
};

struct ionc_client {
	struct mutex lock;
	struct idr handles;
};

static struct miscdevice ionc_misc;

/* dma-buf exporter */

static int ionc_dup_table(struct sg_table *from, struct sg_table *to)
{
	struct scatterlist *sg, *new_sg;
	int ret, i;

	ret = sg_alloc_table(to, from->orig_nents, GFP_KERNEL);
	if (ret)
		return ret;

	new_sg = to->sgl;
	for_each_sgtable_sg(from, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	return 0;
}

static int ionc_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *att)
{
	struct ionc_buffer *buf = dmabuf->priv;
	struct ionc_attachment *a;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = ionc_dup_table(&buf->table, &a->table);
	if (ret) {
		kfree(a);
		return ret;
	}

	a->dev = att->dev;
	INIT_LIST_HEAD(&a->list);
	att->priv = a;

	mutex_lock(&buf->lock);
	list_add(&a->list, &buf->attachments);
	mutex_unlock(&buf->lock);

	return 0;
}

static void ionc_detach(struct dma_buf *dmabuf, struct dma_buf_attachment *att)
{
	struct ionc_buffer *buf = dmabuf->priv;
	struct ionc_attachment *a = att->priv;

	mutex_lock(&buf->lock);
	list_del(&a->list);
	mutex_unlock(&buf->lock);

	sg_free_table(&a->table);
	kfree(a);
}

static struct sg_table *ionc_map_dma_buf(struct dma_buf_attachment *att,
					 enum dma_data_direction dir)
{
	struct ionc_attachment *a = att->priv;
	int ret;

	ret = dma_map_sgtable(att->dev, &a->table, dir, 0);
	if (ret)
		return ERR_PTR(ret);

	a->mapped = true;
	return &a->table;
}

static void ionc_unmap_dma_buf(struct dma_buf_attachment *att,
			       struct sg_table *table,
			       enum dma_data_direction dir)
{
	struct ionc_attachment *a = att->priv;

	a->mapped = false;
	dma_unmap_sgtable(att->dev, table, dir, 0);
}

static int ionc_begin_cpu_access(struct dma_buf *dmabuf,
				 enum dma_data_direction dir)
{
	struct ionc_buffer *buf = dmabuf->priv;
	struct ionc_attachment *a;

	mutex_lock(&buf->lock);
	if (buf->vmap_cnt)
		invalidate_kernel_vmap_range(buf->vaddr, buf->len);
	list_for_each_entry(a, &buf->attachments, list)
		if (a->mapped)
			dma_sync_sgtable_for_cpu(a->dev, &a->table, dir);
	mutex_unlock(&buf->lock);

	return 0;
}

static int ionc_end_cpu_access(struct dma_buf *dmabuf,
			       enum dma_data_direction dir)
{
	struct ionc_buffer *buf = dmabuf->priv;
	struct ionc_attachment *a;

	mutex_lock(&buf->lock);
	if (buf->vmap_cnt)
		flush_kernel_vmap_range(buf->vaddr, buf->len);
	list_for_each_entry(a, &buf->attachments, list)
		if (a->mapped)
			dma_sync_sgtable_for_device(a->dev, &a->table, dir);
	mutex_unlock(&buf->lock);

	return 0;
}

static int ionc_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ionc_buffer *buf = dmabuf->priv;
	unsigned long addr = vma->vm_start;
	unsigned long pgoff = vma->vm_pgoff;
	unsigned int i;
	int ret;

	if (!buf->cached)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	for (i = pgoff; i < buf->npages && addr < vma->vm_end; i++) {
		ret = remap_pfn_range(vma, addr, page_to_pfn(buf->pages[i]),
				      PAGE_SIZE, vma->vm_page_prot);
		if (ret)
			return ret;
		addr += PAGE_SIZE;
	}

	return 0;
}

static int ionc_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ionc_buffer *buf = dmabuf->priv;
	void *vaddr;

	mutex_lock(&buf->lock);
	if (!buf->vmap_cnt) {
		vaddr = vmap(buf->pages, buf->npages, VM_MAP,
			     buf->cached ? PAGE_KERNEL :
					   pgprot_writecombine(PAGE_KERNEL));
		if (!vaddr) {
			mutex_unlock(&buf->lock);
			return -ENOMEM;
		}
		buf->vaddr = vaddr;
	}
	buf->vmap_cnt++;
	iosys_map_set_vaddr(map, buf->vaddr);
	mutex_unlock(&buf->lock);

	return 0;
}

static void ionc_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ionc_buffer *buf = dmabuf->priv;

	mutex_lock(&buf->lock);
	if (!--buf->vmap_cnt) {
		vunmap(buf->vaddr);
		buf->vaddr = NULL;
	}
	mutex_unlock(&buf->lock);
	iosys_map_clear(map);
}

static void ionc_release(struct dma_buf *dmabuf)
{
	struct ionc_buffer *buf = dmabuf->priv;
	unsigned int i;

	dma_unmap_sgtable(ionc_misc.this_device, &buf->table,
			  DMA_BIDIRECTIONAL, 0);
	for (i = 0; i < buf->npages; i++)
		__free_page(buf->pages[i]);
	sg_free_table(&buf->table);
	kvfree(buf->pages);
	kfree(buf);
}

static const struct dma_buf_ops ionc_buf_ops = {
	.attach = ionc_attach,
	.detach = ionc_detach,
	.map_dma_buf = ionc_map_dma_buf,
	.unmap_dma_buf = ionc_unmap_dma_buf,
	.begin_cpu_access = ionc_begin_cpu_access,
	.end_cpu_access = ionc_end_cpu_access,
	.mmap = ionc_mmap,
	.vmap = ionc_vmap,
	.vunmap = ionc_vunmap,
	.release = ionc_release,
};

static struct dma_buf *ionc_alloc(size_t len, unsigned int flags)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp);
	struct ionc_buffer *buf;
	struct dma_buf *dmabuf;
	unsigned int i;
	int ret;

	len = PAGE_ALIGN(len);
	if (!len || len > SZ_512M)
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	mutex_init(&buf->lock);
	INIT_LIST_HEAD(&buf->attachments);
	buf->len = len;
	buf->cached = flags & ION_FLAG_CACHED;
	buf->npages = len >> PAGE_SHIFT;
	buf->pages = kvmalloc_array(buf->npages, sizeof(*buf->pages),
				    GFP_KERNEL);
	if (!buf->pages) {
		ret = -ENOMEM;
		goto err_buf;
	}

	for (i = 0; i < buf->npages; i++) {
		buf->pages[i] = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
		if (!buf->pages[i]) {
			ret = -ENOMEM;
			goto err_pages;
		}
	}

	ret = sg_alloc_table_from_pages(&buf->table, buf->pages, buf->npages,
					0, len, GFP_KERNEL);
	if (ret)
		goto err_pages;

	/*
	 * Map the pages once for this device: it pushes the zeroed lines out
	 * of the cache before anything uses the buffer uncached, and serves
	 * the cache operations of the msm CUSTOM ioctls.
	 */
	ret = dma_map_sgtable(ionc_misc.this_device, &buf->table,
			      DMA_BIDIRECTIONAL, 0);
	if (ret)
		goto err_table;

	exp.ops = &ionc_buf_ops;
	exp.size = len;
	exp.flags = O_RDWR;
	exp.priv = buf;
	exp.exp_name = "ion";
	dmabuf = dma_buf_export(&exp);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_map;
	}

	return dmabuf;

err_map:
	dma_unmap_sgtable(ionc_misc.this_device, &buf->table,
			  DMA_BIDIRECTIONAL, 0);
err_table:
	sg_free_table(&buf->table);
err_pages:
	while (i--)
		__free_page(buf->pages[i]);
	kvfree(buf->pages);
err_buf:
	kfree(buf);
	return ERR_PTR(ret);
}

/* handles */

static int ionc_handle_add(struct ionc_client *client, struct dma_buf *dmabuf)
{
	int id;

	mutex_lock(&client->lock);
	id = idr_alloc(&client->handles, dmabuf, 1, 0, GFP_KERNEL);
	mutex_unlock(&client->lock);

	return id;
}

/* Returns a reference to the dma-buf of a handle */
static struct dma_buf *ionc_handle_get(struct ionc_client *client, int handle)
{
	struct dma_buf *dmabuf;

	mutex_lock(&client->lock);
	dmabuf = idr_find(&client->handles, handle);
	if (dmabuf)
		get_dma_buf(dmabuf);
	mutex_unlock(&client->lock);

	return dmabuf ? dmabuf : ERR_PTR(-EINVAL);
}

static int ionc_handle_free(struct ionc_client *client, int handle)
{
	struct dma_buf *dmabuf;

	mutex_lock(&client->lock);
	dmabuf = idr_remove(&client->handles, handle);
	mutex_unlock(&client->lock);

	if (!dmabuf)
		return -EINVAL;

	dma_buf_put(dmabuf);
	return 0;
}

static int ionc_share(struct ionc_client *client, int handle)
{
	struct dma_buf *dmabuf;
	int fd;

	dmabuf = ionc_handle_get(client, handle);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0)
		dma_buf_put(dmabuf);

	return fd;
}

static int ionc_import(struct ionc_client *client, int fd)
{
	struct dma_buf *dmabuf, *cur;
	int id, ret;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	/* ION gave the same handle for a buffer imported twice */
	mutex_lock(&client->lock);
	idr_for_each_entry(&client->handles, cur, id) {
		if (cur == dmabuf) {
			mutex_unlock(&client->lock);
			dma_buf_put(dmabuf);
			return id;
		}
	}
	mutex_unlock(&client->lock);

	ret = ionc_handle_add(client, dmabuf);
	if (ret < 0)
		dma_buf_put(dmabuf);

	return ret;
}

static int ionc_cache_op(struct ionc_client *client, unsigned int nr,
			 int handle, int fd)
{
	struct dma_buf *dmabuf;
	struct ionc_buffer *buf;

	dmabuf = fd >= 0 ? dma_buf_get(fd) : ionc_handle_get(client, handle);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if (dmabuf->ops != &ionc_buf_ops) {
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	buf = dmabuf->priv;
	if (buf->cached) {
		struct device *dev = ionc_misc.this_device;

		switch (nr) {
		case ION_IOC_CLEAN_CACHES_NR:
			dma_sync_sgtable_for_device(dev, &buf->table,
						    DMA_TO_DEVICE);
			break;
		case ION_IOC_INV_CACHES_NR:
			dma_sync_sgtable_for_cpu(dev, &buf->table,
						 DMA_FROM_DEVICE);
			break;
		default:
			dma_sync_sgtable_for_device(dev, &buf->table,
						    DMA_BIDIRECTIONAL);
			dma_sync_sgtable_for_cpu(dev, &buf->table,
						 DMA_BIDIRECTIONAL);
			break;
		}
	}

	dma_buf_put(dmabuf);
	return 0;
}

/* ioctls */

static long ionc_alloc_legacy(struct ionc_client *client, size_t len,
			      unsigned int flags)
{
	struct dma_buf *dmabuf;
	int handle;

	dmabuf = ionc_alloc(len, flags);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	handle = ionc_handle_add(client, dmabuf);
	if (handle < 0)
		dma_buf_put(dmabuf);

	return handle;
}

static long ionc_heap_query(void __user *uarg)
{
	struct ion_heap_query q;
	struct ion_heap_data heap = {
		.name = "system",
		.type = 0,		/* ION_HEAP_TYPE_SYSTEM */
		.heap_id = 25,		/* ION_SYSTEM_HEAP_ID */
	};

	if (copy_from_user(&q, uarg, sizeof(q)))
		return -EFAULT;

	if (q.heaps && q.cnt) {
		if (copy_to_user(u64_to_user_ptr(q.heaps), &heap, sizeof(heap)))
			return -EFAULT;
	}
	q.cnt = 1;

	return copy_to_user(uarg, &q, sizeof(q)) ? -EFAULT : 0;
}

static long ionc_ioctl_common(struct ionc_client *client, unsigned int cmd,
			      void __user *uarg)
{
	switch (cmd) {
	case ION_IOC_FREE: {
		struct ion_handle_data data;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		return ionc_handle_free(client, data.handle);
	}
	case ION_IOC_MAP:
	case ION_IOC_SHARE: {
		struct ion_fd_data data;
		int fd;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		fd = ionc_share(client, data.handle);
		if (fd < 0)
			return fd;
		data.fd = fd;
		return copy_to_user(uarg, &data, sizeof(data)) ? -EFAULT : 0;
	}
	case ION_IOC_IMPORT: {
		struct ion_fd_data data;
		int handle;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		handle = ionc_import(client, data.fd);
		if (handle < 0)
			return handle;
		data.handle = handle;
		return copy_to_user(uarg, &data, sizeof(data)) ? -EFAULT : 0;
	}
	case ION_IOC_SYNC:
		return 0;
	case ION_IOC_HEAP_QUERY:
		return ionc_heap_query(uarg);
	}

	return -ENOTTY;
}

static long ionc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ionc_client *client = file->private_data;
	void __user *uarg = (void __user *)arg;

	if (_IOC_TYPE(cmd) == ION_IOC_MAGIC && _IOC_NR(cmd) == ION_IOC_ALLOC_NR) {
		if (_IOC_SIZE(cmd) == sizeof(struct ion_allocation_data_new)) {
			struct ion_allocation_data_new data;
			struct dma_buf *dmabuf;
			int fd;

			if (copy_from_user(&data, uarg, sizeof(data)))
				return -EFAULT;
			dmabuf = ionc_alloc(data.len, data.flags);
			if (IS_ERR(dmabuf))
				return PTR_ERR(dmabuf);
			fd = dma_buf_fd(dmabuf, O_CLOEXEC);
			if (fd < 0) {
				dma_buf_put(dmabuf);
				return fd;
			}
			data.fd = fd;
			return copy_to_user(uarg, &data, sizeof(data)) ?
				-EFAULT : 0;
		} else if (_IOC_SIZE(cmd) ==
			   sizeof(struct ion_allocation_data_legacy)) {
			struct ion_allocation_data_legacy data;
			long handle;

			if (copy_from_user(&data, uarg, sizeof(data)))
				return -EFAULT;
			handle = ionc_alloc_legacy(client, data.len, data.flags);
			if (handle < 0)
				return handle;
			data.handle = handle;
			return copy_to_user(uarg, &data, sizeof(data)) ?
				-EFAULT : 0;
		}
		return -ENOTTY;
	}

	if (_IOC_TYPE(cmd) == ION_IOC_MAGIC &&
	    _IOC_NR(cmd) == ION_IOC_CUSTOM_NR &&
	    _IOC_SIZE(cmd) == sizeof(struct ion_custom_data)) {
		struct ion_custom_data custom;
		struct ion_flush_data flush;

		if (copy_from_user(&custom, uarg, sizeof(custom)))
			return -EFAULT;
		if (_IOC_TYPE(custom.cmd) != ION_IOC_MSM_MAGIC ||
		    _IOC_SIZE(custom.cmd) != sizeof(flush))
			return -ENOTTY;
		if (copy_from_user(&flush, (void __user *)custom.arg,
				   sizeof(flush)))
			return -EFAULT;
		return ionc_cache_op(client, _IOC_NR(custom.cmd),
				     flush.handle, flush.fd);
	}

	return ionc_ioctl_common(client, cmd, uarg);
}

#ifdef CONFIG_COMPAT
static long ionc_compat_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct ionc_client *client = file->private_data;
	void __user *uarg = compat_ptr(arg);

	if (_IOC_TYPE(cmd) == ION_IOC_MAGIC && _IOC_NR(cmd) == ION_IOC_ALLOC_NR &&
	    _IOC_SIZE(cmd) == sizeof(struct compat_ion_allocation_data_legacy)) {
		struct compat_ion_allocation_data_legacy data;
		long handle;

		if (copy_from_user(&data, uarg, sizeof(data)))
			return -EFAULT;
		handle = ionc_alloc_legacy(client, data.len, data.flags);
		if (handle < 0)
			return handle;
		data.handle = handle;
		return copy_to_user(uarg, &data, sizeof(data)) ? -EFAULT : 0;
	}

	if (_IOC_TYPE(cmd) == ION_IOC_MAGIC &&
	    _IOC_NR(cmd) == ION_IOC_CUSTOM_NR &&
	    _IOC_SIZE(cmd) == sizeof(struct compat_ion_custom_data)) {
		struct compat_ion_custom_data custom;
		struct compat_ion_flush_data flush;

		if (copy_from_user(&custom, uarg, sizeof(custom)))
			return -EFAULT;
		if (_IOC_TYPE(custom.cmd) != ION_IOC_MSM_MAGIC ||
		    _IOC_SIZE(custom.cmd) != sizeof(flush))
			return -ENOTTY;
		if (copy_from_user(&flush, compat_ptr(custom.arg),
				   sizeof(flush)))
			return -EFAULT;
		return ionc_cache_op(client, _IOC_NR(custom.cmd),
				     flush.handle, flush.fd);
	}

	/* The new ALLOC, the fd/handle ioctls and HEAP_QUERY match */
	return ionc_ioctl(file, cmd, (unsigned long)uarg);
}
#endif

static int ionc_open(struct inode *inode, struct file *file)
{
	struct ionc_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	mutex_init(&client->lock);
	idr_init(&client->handles);
	file->private_data = client;

	return 0;
}

static int ionc_release_file(struct inode *inode, struct file *file)
{
	struct ionc_client *client = file->private_data;
	struct dma_buf *dmabuf;
	int id;

	idr_for_each_entry(&client->handles, dmabuf, id)
		dma_buf_put(dmabuf);
	idr_destroy(&client->handles);
	kfree(client);

	return 0;
}

static const struct file_operations ionc_fops = {
	.owner = THIS_MODULE,
	.open = ionc_open,
	.release = ionc_release_file,
	.unlocked_ioctl = ionc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ionc_compat_ioctl,
#endif
};

static struct miscdevice ionc_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ion",
	.fops = &ionc_fops,
};

static int __init ionc_init(void)
{
	struct device *dev;
	int ret;

	ret = misc_register(&ionc_misc);
	if (ret)
		return ret;

	/* The device the buffers are mapped to for cache maintenance */
	dev = ionc_misc.this_device;
	dev->coherent_dma_mask = DMA_BIT_MASK(64);
	dev->dma_mask = &dev->coherent_dma_mask;

	return 0;
}
device_initcall(ionc_init);

MODULE_IMPORT_NS("DMA_BUF");
