// SPDX-License-Identifier: GPL-2.0
/*
 * Cam* device helpers of the SigmaStar BSP (drivers/sstar/cam_sysfs) and
 * the 4.9 device model objects the vendor code dereferences.
 *
 * The BSP helpers are 1:1 calls into the kernel, but mhal reads and writes
 * 4.9 struct class, device and platform_device fields inline. The vendor
 * code therefore gets 4.9-layout shadows of the real objects:
 * - class: suspend@36, resume@40 written by the vendor (not called)
 * - device: parent@0, kobj@8 (name@8), platform_data@96, driver_data@100,
 *   of_node@352, devt@360, class@408
 * - platform_device: name@0, id@4, dev@16
 * Platform drivers, device attributes and misc devices registered by the
 * vendor get trampolines that hand out the shadows and run the vendor
 * callbacks with osm_enter().
 */
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "osm_compat.h"
#include "osm_sstar.h"

#define CLASS49_SIZE		64
#define DEV49_PARENT		0
#define DEV49_KOBJ		8
#define DEV49_INIT_NAME		44
#define DEV49_PLATFORM_DATA	96
#define DEV49_DRIVER_DATA	100
#define DEV49_OF_NODE		352
#define DEV49_DEVT		360
#define DEV49_CLASS		408
#define DEV49_SIZE		448
#define PDEV49_NAME		0
#define PDEV49_ID		4
#define PDEV49_DEV		16
#define PDEV49_SIZE		(PDEV49_DEV + DEV49_SIZE)

/* struct platform_driver of 4.9 */
#define PDRV49_PROBE		0
#define PDRV49_REMOVE		1
#define PDRV49_SHUTDOWN		2
#define PDRV49_SUSPEND		3
#define PDRV49_RESUME		4
#define PDRV49_NAME		5
#define PDRV49_OF_MATCH		11

#define OSM_MAGIC_CLASS		0x6f736d63

static inline void put32(u8 *b, unsigned int off, u32 v) { *(u32 *)(b + off) = v; }
static inline u32 get32(const u8 *b, unsigned int off) { return *(const u32 *)(b + off); }

struct osm_class49 {
	u8 raw[CLASS49_SIZE];
	u32 magic;
	struct class *real;
};

/* a device shadow, alone or inside a platform_device shadow */
struct osm_dev49 {
	struct list_head node;
	struct device *real;
	struct platform_device *pdev;	/* platform device shadow */
	bool created;			/* by CamDeviceCreate() */
	u8 raw[PDEV49_SIZE];		/* device at raw + dev_off */
	unsigned int dev_off;
};

static LIST_HEAD(osm_devs);
static DEFINE_MUTEX(osm_devs_lock);

static struct class *osm_class(void *class49)
{
	struct osm_class49 *c = class49;

	if (WARN_ON_ONCE(!c || c->magic != OSM_MAGIC_CLASS))
		return NULL;
	return c->real;
}

static u8 *dev49_raw(struct osm_dev49 *d)
{
	return d->raw + d->dev_off;
}

/* osm_devs_lock held */
static struct osm_dev49 *dev49_find(const void *dev49)
{
	struct osm_dev49 *d;

	list_for_each_entry(d, &osm_devs, node)
		if (dev49_raw(d) == dev49 || d->raw == dev49)
			return d;
	return NULL;
}

static struct osm_dev49 *dev49_find_real(struct device *real)
{
	struct osm_dev49 *d;

	list_for_each_entry(d, &osm_devs, node)
		if (d->real == real)
			return d;
	return NULL;
}

/* 4.9 device of the vendor code to the real one (NULL stays NULL) */
static struct device *dev_real(const void *dev49)
{
	struct osm_dev49 *d;

	if (!dev49)
		return NULL;
	mutex_lock(&osm_devs_lock);
	d = dev49_find(dev49);
	mutex_unlock(&osm_devs_lock);
	WARN_ONCE(!d, "osm: unknown 4.9 device %px\n", dev49);
	return d ? d->real : NULL;
}

static struct platform_device *pdev_real(const void *pdev49)
{
	struct osm_dev49 *d;

	mutex_lock(&osm_devs_lock);
	d = dev49_find(pdev49);
	mutex_unlock(&osm_devs_lock);
	if (WARN_ONCE(!d || !d->pdev, "osm: unknown 4.9 platform_device %px\n", pdev49))
		return NULL;
	return d->pdev;
}

/* &dev49->kobj of a shadow to the real kobject, other kobjects are real */
static struct kobject *kobj_real(void *kobj49)
{
	struct osm_dev49 *d;

	if (!kobj49)
		return NULL;
	mutex_lock(&osm_devs_lock);
	d = dev49_find((u8 *)kobj49 - DEV49_KOBJ);
	mutex_unlock(&osm_devs_lock);
	return d ? &d->real->kobj : kobj49;
}

static void dev49_fill(u8 *raw, struct device *dev)
{
	put32(raw, DEV49_KOBJ, (u32)dev_name(dev));
	put32(raw, DEV49_INIT_NAME, (u32)dev_name(dev));
	put32(raw, DEV49_PLATFORM_DATA, (u32)dev->platform_data);
	put32(raw, DEV49_DRIVER_DATA, (u32)dev_get_drvdata(dev));
	put32(raw, DEV49_OF_NODE, (u32)dev->of_node);
	put32(raw, DEV49_DEVT, dev->devt);
}

/* shadow of a device, created on first use */
static struct osm_dev49 *dev49_get(struct device *dev, struct platform_device *pdev)
{
	struct osm_dev49 *d;

	mutex_lock(&osm_devs_lock);
	d = dev49_find_real(dev);
	if (d)
		goto out;
	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		goto out;
	d->real = dev;
	d->pdev = pdev;
	d->dev_off = pdev ? PDEV49_DEV : 0;
	if (pdev) {
		put32(d->raw, PDEV49_NAME, (u32)pdev->name);
		put32(d->raw, PDEV49_ID, pdev->id);
	}
	dev49_fill(dev49_raw(d), dev);
	list_add(&d->node, &osm_devs);
	mutex_unlock(&osm_devs_lock);
	/* the vendor follows dev->parent (isp_probe links to its kobject) */
	if (dev->parent) {
		struct osm_dev49 *p = dev49_get(dev->parent, NULL);

		if (p)
			put32(dev49_raw(d), DEV49_PARENT, (u32)dev49_raw(p));
	}
	return d;
out:
	mutex_unlock(&osm_devs_lock);
	return d;
}

static void dev49_put(struct osm_dev49 *d)
{
	mutex_lock(&osm_devs_lock);
	list_del(&d->node);
	mutex_unlock(&osm_devs_lock);
	kfree(d);
}

/* Classes */

void *CamClassCreate(struct module *owner, const char *name)
{
	struct osm_class49 *c;
	struct class *real;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);
	real = class_create(name);
	if (IS_ERR(real)) {
		kfree(c);
		return real;
	}
	c->magic = OSM_MAGIC_CLASS;
	c->real = real;
	/* 4.9 class->name; the vendor PM callbacks are not used */
	*(const char **)c->raw = name;
	return c;
}
EXPORT_SYMBOL(CamClassCreate);

void CamClassDestroy(void *class49)
{
	struct class *real = osm_class(class49);

	if (!real)
		return;
	class_destroy(real);
	kfree(class49);
}
EXPORT_SYMBOL(CamClassDestroy);

/* class "mstar" of the BSP msys driver */
static void *msys_class;
static DEFINE_MUTEX(msys_class_lock);

void *msys_get_sysfs_class(void)
{
	mutex_lock(&msys_class_lock);
	if (!msys_class) {
		void *c = CamClassCreate(NULL, "mstar");

		if (!IS_ERR(c))
			msys_class = c;
	}
	mutex_unlock(&msys_class_lock);
	return msys_class;
}
EXPORT_SYMBOL(msys_get_sysfs_class);

/* Devices */

void *CamDeviceCreate(void *class49, void *parent49, dev_t devt,
		      void *drvdata, const char *fmt, ...)
{
	struct class *real = osm_class(class49);
	struct osm_dev49 *d;
	struct device *dev;
	va_list args;
	char *name;

	if (!real)
		return ERR_PTR(-EINVAL);
	va_start(args, fmt);
	name = kvasprintf(GFP_KERNEL, fmt, args);
	va_end(args);
	if (!name)
		return ERR_PTR(-ENOMEM);
	dev = device_create(real, dev_real(parent49), devt, drvdata, "%s", name);
	kfree(name);
	if (IS_ERR(dev))
		return dev;
	d = dev49_get(dev, NULL);
	if (!d) {
		device_destroy(real, devt);
		return ERR_PTR(-ENOMEM);
	}
	d->created = true;
	put32(d->raw, DEV49_CLASS, (u32)class49);
	return d->raw;
}
EXPORT_SYMBOL(CamDeviceCreate);

static void dev49_put_devt(dev_t devt)
{
	struct osm_dev49 *d, *t;

	mutex_lock(&osm_devs_lock);
	list_for_each_entry_safe(d, t, &osm_devs, node) {
		if (d->created && d->real->devt == devt) {
			list_del(&d->node);
			kfree(d);
		}
	}
	mutex_unlock(&osm_devs_lock);
}

void CamDeviceDestroy(void *class49, dev_t devt)
{
	struct class *real = osm_class(class49);

	if (!real)
		return;
	dev49_put_devt(devt);
	device_destroy(real, devt);
}
EXPORT_SYMBOL(CamDeviceDestroy);

void CamDeviceUnregister(void *dev49)
{
	struct device *dev = dev_real(dev49);

	if (!dev)
		return;
	dev49_put_devt(dev->devt);
	device_unregister(dev);
}
EXPORT_SYMBOL(CamDeviceUnregister);

void *CamDevmKmalloc(void *dev49, size_t size, unsigned int gfp49)
{
	return devm_kmalloc(dev_real(dev49), size, osm_gfp(gfp49));
}
EXPORT_SYMBOL(CamDevmKmalloc);

void CamDevmKfree(void *dev49, void *p)
{
	devm_kfree(dev_real(dev49), p);
}
EXPORT_SYMBOL(CamDevmKfree);

/* Device attributes: the vendor show/store get the 4.9 device */
struct osm_devattr {
	struct list_head node;
	struct device_attribute attr;
	struct device_attribute *a49;
	struct device *dev;
};

static LIST_HEAD(osm_devattrs);
static DEFINE_MUTEX(osm_devattrs_lock);

static void *dev49_of(struct device *dev)
{
	struct osm_dev49 *d = dev49_get(dev, NULL);

	return d ? dev49_raw(d) : NULL;
}

static ssize_t osm_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct osm_devattr *a = container_of(attr, struct osm_devattr, attr);
	struct osm_entry e;
	ssize_t r;

	osm_enter(&e);
	r = a->a49->show(dev49_of(dev), a->a49, buf);
	osm_leave(&e);
	return r;
}

static ssize_t osm_attr_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct osm_devattr *a = container_of(attr, struct osm_devattr, attr);
	struct osm_entry e;
	ssize_t r;

	osm_enter(&e);
	r = a->a49->store(dev49_of(dev), a->a49, buf, count);
	osm_leave(&e);
	return r;
}

int CamDeviceCreateFile(void *dev49, struct device_attribute *a49)
{
	struct device *dev = dev_real(dev49);
	struct osm_devattr *a;
	int ret;

	if (!dev)
		return -EINVAL;
	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	a->a49 = a49;
	a->dev = dev;
	sysfs_attr_init(&a->attr.attr);
	a->attr.attr.name = a49->attr.name;
	a->attr.attr.mode = a49->attr.mode;
	if (a49->show)
		a->attr.show = osm_attr_show;
	if (a49->store)
		a->attr.store = osm_attr_store;
	ret = device_create_file(dev, &a->attr);
	if (ret) {
		kfree(a);
		return ret;
	}
	mutex_lock(&osm_devattrs_lock);
	list_add(&a->node, &osm_devattrs);
	mutex_unlock(&osm_devattrs_lock);
	return 0;
}
EXPORT_SYMBOL(CamDeviceCreateFile);

void CamDeviceRemoveFile(void *dev49, const struct device_attribute *a49)
{
	struct device *dev = dev_real(dev49);
	struct osm_devattr *a, *t;

	mutex_lock(&osm_devattrs_lock);
	list_for_each_entry_safe(a, t, &osm_devattrs, node) {
		if (a->dev == dev && a->a49 == a49) {
			list_del(&a->node);
			device_remove_file(dev, &a->attr);
			kfree(a);
		}
	}
	mutex_unlock(&osm_devattrs_lock);
}
EXPORT_SYMBOL(CamDeviceRemoveFile);

/* Kobjects and plain sysfs files (kobj_attribute, called without osm_enter) */

struct kobject *CamKobjectCreateAndAdd(const char *name, void *parent49)
{
	return kobject_create_and_add(name, kobj_real(parent49));
}
EXPORT_SYMBOL(CamKobjectCreateAndAdd);

int CamSysfsCreateFiles(void *kobj49, const struct attribute **attr)
{
	return sysfs_create_files(kobj_real(kobj49), attr);
}
EXPORT_SYMBOL(CamSysfsCreateFiles);

void CamSysfsRemoveFiles(void *kobj49, const struct attribute **attr)
{
	sysfs_remove_files(kobj_real(kobj49), attr);
}
EXPORT_SYMBOL(CamSysfsRemoveFiles);

int CamSysfsCreateLink(void *kobj49, void *target49, const char *name)
{
	return sysfs_create_link(kobj_real(kobj49), kobj_real(target49), name);
}
EXPORT_SYMBOL(CamSysfsCreateLink);

/* Platform drivers */

struct osm_pdrv {
	struct list_head node;
	void *const *drv49;
	struct platform_driver drv;
};

static LIST_HEAD(osm_pdrvs);
static DEFINE_MUTEX(osm_pdrvs_lock);

static void *const *pdrv49(struct platform_device *pdev)
{
	return container_of(to_platform_driver(pdev->dev.driver), struct osm_pdrv, drv)->drv49;
}

static int osm_pdrv_call(struct platform_device *pdev, int slot, void *pdev49, int arg)
{
	int (*fn)(void *, int) = pdrv49(pdev)[slot];
	struct osm_entry e;
	int ret;

	osm_enter(&e);
	ret = fn(pdev49, arg);
	osm_leave(&e);
	return ret;
}

static int osm_pdrv_probe(struct platform_device *pdev)
{
	struct osm_dev49 *d = dev49_get(&pdev->dev, pdev);
	int ret;

	if (!d)
		return -ENOMEM;
	ret = osm_pdrv_call(pdev, PDRV49_PROBE, d->raw, 0);
	if (ret)
		dev49_put(d);
	return ret;
}

static void osm_pdrv_remove(struct platform_device *pdev)
{
	struct osm_dev49 *d = dev49_get(&pdev->dev, pdev);

	if (!d)
		return;
	if (pdrv49(pdev)[PDRV49_REMOVE])
		osm_pdrv_call(pdev, PDRV49_REMOVE, d->raw, 0);
	dev49_put(d);
}

static void osm_pdrv_shutdown(struct platform_device *pdev)
{
	struct osm_dev49 *d = dev49_get(&pdev->dev, pdev);

	if (d)
		osm_pdrv_call(pdev, PDRV49_SHUTDOWN, d->raw, 0);
}

static int osm_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct osm_dev49 *d = dev49_get(&pdev->dev, pdev);

	return d ? osm_pdrv_call(pdev, PDRV49_SUSPEND, d->raw, state.event) : 0;
}

static int osm_pdrv_resume(struct platform_device *pdev)
{
	struct osm_dev49 *d = dev49_get(&pdev->dev, pdev);

	return d ? osm_pdrv_call(pdev, PDRV49_RESUME, d->raw, 0) : 0;
}

int CamPlatformDriverRegister(void *const *drv49)
{
	struct osm_pdrv *p;
	int ret;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->drv49 = drv49;
	p->drv.driver.name = drv49[PDRV49_NAME];
	/* struct of_device_id has the same layout */
	p->drv.driver.of_match_table = drv49[PDRV49_OF_MATCH];
	p->drv.probe = osm_pdrv_probe;
	p->drv.remove = osm_pdrv_remove;
	if (drv49[PDRV49_SHUTDOWN])
		p->drv.shutdown = osm_pdrv_shutdown;
	if (drv49[PDRV49_SUSPEND])
		p->drv.suspend = osm_pdrv_suspend;
	if (drv49[PDRV49_RESUME])
		p->drv.resume = osm_pdrv_resume;
	mutex_lock(&osm_pdrvs_lock);
	list_add(&p->node, &osm_pdrvs);
	mutex_unlock(&osm_pdrvs_lock);
	ret = platform_driver_register(&p->drv);
	if (ret) {
		mutex_lock(&osm_pdrvs_lock);
		list_del(&p->node);
		mutex_unlock(&osm_pdrvs_lock);
		kfree(p);
	}
	return ret;
}
EXPORT_SYMBOL(CamPlatformDriverRegister);

void CamPlatformDriverUnregister(void *const *drv49)
{
	struct osm_pdrv *p, *t, *found = NULL;

	mutex_lock(&osm_pdrvs_lock);
	list_for_each_entry_safe(p, t, &osm_pdrvs, node) {
		if (p->drv49 == drv49) {
			list_del(&p->node);
			found = p;
			break;
		}
	}
	mutex_unlock(&osm_pdrvs_lock);
	if (found) {
		platform_driver_unregister(&found->drv);
		kfree(found);
	}
}
EXPORT_SYMBOL(CamPlatformDriverUnregister);

/* of_find_device_by_node() from vendor code: the 4.9 platform_device */
void *osm_of_find_device_by_node(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	struct osm_dev49 *d;

	if (!pdev)
		return NULL;
	d = dev49_get(&pdev->dev, pdev);
	return d ? d->raw : NULL;
}
EXPORT_SYMBOL(osm_of_find_device_by_node);

struct resource *CamPlatformGetResource(void *pdev49, unsigned int type, unsigned int num)
{
	struct platform_device *pdev = pdev_real(pdev49);

	return pdev ? platform_get_resource(pdev, type, num) : NULL;
}
EXPORT_SYMBOL(CamPlatformGetResource);

int CamPlatformGetIrqByname(void *pdev49, const char *name)
{
	struct platform_device *pdev = pdev_real(pdev49);

	return pdev ? platform_get_irq_byname(pdev, name) : -EINVAL;
}
EXPORT_SYMBOL(CamPlatformGetIrqByname);

/* Device tree helpers: device_node and resource have the 4.9 layout */

int CamOfAddressToResource(struct device_node *dev, int index, struct resource *r)
{
	return of_address_to_resource(dev, index, r);
}
EXPORT_SYMBOL(CamOfAddressToResource);

int CamOfIrqToResource(struct device_node *dev, int index, struct resource *r)
{
	return of_irq_to_resource(dev, index, r);
}
EXPORT_SYMBOL(CamOfIrqToResource);

unsigned int CamIrqOfParseAndMap(struct device_node *dev, int index)
{
	return irq_of_parse_and_map(dev, index);
}
EXPORT_SYMBOL(CamIrqOfParseAndMap);

int CamofPropertyReadU32(const struct device_node *np, const char *propname, u32 *out_value)
{
	return of_property_read_u32(np, propname, out_value);
}
EXPORT_SYMBOL(CamofPropertyReadU32);

int CamOfPropertyReadU32Index(const struct device_node *np, const char *propname,
			      u32 index, u32 *out_value)
{
	return of_property_read_u32_index(np, propname, index, out_value);
}
EXPORT_SYMBOL(CamOfPropertyReadU32Index);

int CamOfPropertyReadU32Array(const struct device_node *np, const char *propname,
			      u32 *out_values, size_t sz)
{
	return of_property_read_u32_array(np, propname, out_values, sz);
}
EXPORT_SYMBOL(CamOfPropertyReadU32Array);

/* Legacy GPIO numbers of the BSP do not match gpiolib: not supported yet */
int CamGpioRequest(unsigned int gpio, const char *label)
{
	pr_warn_once("osm: CamGpioRequest(%u, %s) ignored\n", gpio, label);
	return 0;
}
EXPORT_SYMBOL(CamGpioRequest);

/* Misc devices: same struct layout, 4.9 fops and parent */
struct osm_misc {
	struct list_head node;
	struct miscdevice *m49;
	struct miscdevice m;
};

static LIST_HEAD(osm_miscs);
static DEFINE_MUTEX(osm_miscs_lock);

int osm_misc_register(struct miscdevice *m49)
{
	struct osm_misc *o;
	int ret;

	o = kzalloc(sizeof(*o), GFP_KERNEL);
	if (!o)
		return -ENOMEM;
	o->m49 = m49;
	o->m.minor = m49->minor;
	o->m.name = m49->name;
	o->m.fops = osm_fops49(m49->fops);
	o->m.parent = dev_real(m49->parent);
	o->m.groups = m49->groups;
	o->m.nodename = m49->nodename;
	o->m.mode = m49->mode;
	ret = misc_register(&o->m);
	if (ret) {
		kfree(o);
		return ret;
	}
	m49->minor = o->m.minor;
	/* isp_probe links sysfs entries to misc->this_device */
	m49->this_device = dev49_of(o->m.this_device);
	mutex_lock(&osm_miscs_lock);
	list_add(&o->node, &osm_miscs);
	mutex_unlock(&osm_miscs_lock);
	return 0;
}
EXPORT_SYMBOL(osm_misc_register);

void osm_misc_deregister(struct miscdevice *m49)
{
	struct osm_misc *o, *t;

	mutex_lock(&osm_miscs_lock);
	list_for_each_entry_safe(o, t, &osm_miscs, node) {
		if (o->m49 == m49) {
			struct osm_dev49 *d;

			list_del(&o->node);
			mutex_lock(&osm_devs_lock);
			d = dev49_find_real(o->m.this_device);
			if (d)
				list_del(&d->node);
			mutex_unlock(&osm_devs_lock);
			kfree(d);
			misc_deregister(&o->m);
			kfree(o);
		}
	}
	mutex_unlock(&osm_miscs_lock);
}
EXPORT_SYMBOL(osm_misc_deregister);

void osm_cam_sysfs_init(void)
{
	osm_kobj_real = kobj_real;
}

void osm_cam_sysfs_exit(void)
{
	osm_kobj_real = NULL;
	if (msys_class)
		CamClassDestroy(msys_class);
}
