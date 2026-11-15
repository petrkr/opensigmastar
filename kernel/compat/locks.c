// SPDX-License-Identifier: GPL-2.0
/*
 * 4.9 wait queues and semaphores of the vendor modules.
 *
 * The vendor kernel has DEBUG_SPINLOCK, so spinlock_t is 16 bytes there
 * and wait_queue_head_t and struct semaphore are 12 bytes larger than now.
 * The current structure is kept at offset 12, which puts the wait list of
 * a wait queue head where the inline waitqueue_active() of the vendor code
 * looks (offset 16), and the semaphore count where 4.9 has it. The lock
 * word then overlaps the 4.9 lock owner, which the 4.9 initialisers set to
 * -1: an unlocked ticket lock.
 */
#include <linux/module.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/limits.h>
#include "osm_compat.h"

#define OSM_LOCK49_PAD	12

wait_queue_head_t *osm_wq(void *wq49)
{
	return wq49 ? (wait_queue_head_t *)((u8 *)wq49 + OSM_LOCK49_PAD) : NULL;
}
EXPORT_SYMBOL(osm_wq);

void osm___init_waitqueue_head(void *wq49, const char *name, struct lock_class_key *key);
/* debugging: the last wait queue heads initialised by vendor code */
#define OSM_WQ_RING 16
static void *osm_wq_inited[OSM_WQ_RING];
static atomic_t osm_wq_inited_n = ATOMIC_INIT(0);

void osm_wq_dump_inited(void)
{
	int i;

	for (i = 0; i < OSM_WQ_RING; i++)
		pr_info("osm: inited wq49 %px\n", osm_wq_inited[i]);
}
EXPORT_SYMBOL(osm_wq_dump_inited);

void osm___init_waitqueue_head(void *wq49, const char *name, struct lock_class_key *key)
{
	osm_wq_inited[atomic_inc_return(&osm_wq_inited_n) % OSM_WQ_RING] = wq49;
	/* the vendor code also takes the 4.9 lock itself: unlocked raw lock */
	memset(wq49, 0, OSM_LOCK49_PAD);
	init_waitqueue_head(osm_wq(wq49));
}
EXPORT_SYMBOL(osm___init_waitqueue_head);

void osm___wake_up(void *wq49, unsigned int mode, int nr, void *key)
{
	__wake_up(osm_wq(wq49), mode, nr, key);
}
EXPORT_SYMBOL(osm___wake_up);

long osm_prepare_to_wait_event(void *wq49, struct wait_queue_entry *wq_entry, int state)
{
	return prepare_to_wait_event(osm_wq(wq49), wq_entry, state);
}
EXPORT_SYMBOL(osm_prepare_to_wait_event);

void osm_finish_wait(void *wq49, struct wait_queue_entry *wq_entry)
{
	finish_wait(osm_wq(wq49), wq_entry);
}
EXPORT_SYMBOL(osm_finish_wait);

/*
 * 4.9 semaphore: lock (16), count, wait_list. The current one at offset
 * 12 has first_waiter where the 4.9 wait list starts, which the 4.9
 * initialiser points to itself: clear that on use.
 */
static struct semaphore *osm_sem(void *sem49)
{
	struct semaphore *sem = (struct semaphore *)((u8 *)sem49 + OSM_LOCK49_PAD);
	void *self = (u8 *)sem49 + 20;

	static_assert(sizeof(struct semaphore) <= 28 - OSM_LOCK49_PAD);
	cmpxchg((void **)&sem->first_waiter, self, NULL);
	return sem;
}

void osm_down(void *sem49)
{
	down(osm_sem(sem49));
}
EXPORT_SYMBOL(osm_down);

void osm_up(void *sem49)
{
	up(osm_sem(sem49));
}
EXPORT_SYMBOL(osm_up);

/*
 * struct completion of 4.9: done, then a 4.9 wait queue head (28 bytes).
 * The vendor code inits it inline (done = 0, __init_waitqueue_head) and
 * polls on its wait queue, so it stays a 4.9 completion here.
 */
struct osm_completion49 {
	unsigned int done;
	u8 wait[24];
};

void osm_completion49_init(void *c49)
{
	struct osm_completion49 *c = c49;

	c->done = 0;
	osm___init_waitqueue_head(c->wait, NULL, NULL);
}
EXPORT_SYMBOL(osm_completion49_init);

void osm_completion49_complete(void *c49, bool all)
{
	struct osm_completion49 *c = c49;
	wait_queue_head_t *wq = osm_wq(c->wait);
	unsigned long flags;

	spin_lock_irqsave(&wq->lock, flags);
	if (all)
		c->done = UINT_MAX;
	else if (c->done != UINT_MAX)
		c->done++;
	spin_unlock_irqrestore(&wq->lock, flags);
	if (all)
		wake_up_all(wq);
	else
		wake_up(wq);
}
EXPORT_SYMBOL(osm_completion49_complete);

static bool osm_completion49_take(struct osm_completion49 *c)
{
	wait_queue_head_t *wq = osm_wq(c->wait);
	unsigned long flags;
	bool ret = false;

	spin_lock_irqsave(&wq->lock, flags);
	if (c->done) {
		if (c->done != UINT_MAX)
			c->done--;
		ret = true;
	}
	spin_unlock_irqrestore(&wq->lock, flags);
	return ret;
}

/*
 * Wait like wait_for_completion*(): returns the remaining jiffies (at least
 * 1) when completed, 0 on timeout, -ERESTARTSYS when interrupted.
 */
long osm_completion49_wait(void *c49, long timeout, bool interruptible)
{
	struct osm_completion49 *c = c49;
	wait_queue_head_t *wq = osm_wq(c->wait);
	long ret;

	for (;;) {
		if (osm_completion49_take(c))
			return timeout ? timeout : 1;
		if (interruptible)
			ret = wait_event_interruptible_timeout(*wq, READ_ONCE(c->done), timeout);
		else
			ret = wait_event_timeout(*wq, READ_ONCE(c->done), timeout);
		if (ret < 0)
			return ret;
		if (!ret && !READ_ONCE(c->done))
			return 0;
		timeout = ret ? ret : 1;
	}
}
EXPORT_SYMBOL(osm_completion49_wait);

bool osm_completion49_done(void *c49)
{
	return READ_ONCE(((struct osm_completion49 *)c49)->done) != 0;
}
EXPORT_SYMBOL(osm_completion49_done);

unsigned long osm_wait_for_completion_timeout(void *c49, unsigned long timeout)
{
	return osm_completion49_wait(c49, timeout, false);
}
EXPORT_SYMBOL(osm_wait_for_completion_timeout);
