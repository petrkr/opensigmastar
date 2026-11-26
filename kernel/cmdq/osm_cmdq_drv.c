// SPDX-License-Identifier: GPL-2.0
/*
 * osm_cmdq: driver for the four SSC338Q command queue engines (CMDQ0..3)
 * behind the MHAL_CMDQ_* interface of the SigmaStar mhal blob.
 *
 * A CMDQ engine executes a ring of 64 bit commands (register write with
 * mask, poll, wait for a trigger event) in step with the hardware blocks
 * that raise the trigger events (ISP, scaler, JPEG, ...). The mhal blob and
 * the mi_* modules reach it through MHAL_CMDQ_GetSysCmdqService(), which
 * returns a table of functions (struct cmdq_if). With scripts/relink.py
 * --cmdq own the blob's own implementation is replaced by this module, which
 * must be loaded before mhal.
 *
 * The command and status registers of the four engines are not independent:
 * the failing opcode and current command registers (0xf0..0x10c) show what
 * any engine executes, and bit 8 of the status word (0x114) is set while an
 * enabled engine has not been started, so an error is only taken from it
 * after the first kick.
 *
 * Register block per engine (DT node sstar,cmdq<N>, reg = bank): 16 bit
 * registers at a 4 byte stride, a 32 bit value is a low half at +0 and a
 * high half at +4 (written high half first, then the low half, which latches
 * the pair). Offsets below are those byte offsets.
 *
 * Ring: start/end/write pointers in 16 byte units of the MIU bus address,
 * the engine consumes the commands between its read pointer and the write
 * pointer each time the kick strobe is set. The ring memory is carved from
 * the pool the mi_sys module hands over with MHAL_CMDQ_InitCmdqMmapInfo().
 */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include "osm_riu.h"

/* osm_sstar */
void Chip_Flush_Cache_Range(unsigned long addr, unsigned long size);

#define NENG		4

/* engine registers (byte offsets) */
#define R_CTRL		0x00	/* hi bit0: enable, lo bits1:0 engine count */
#define R_MODE		0x08	/* lo: bit2 ring; hi (0x0c): bit1 kick, bit8 latch read pointer */
#define R_STRB		0x0c
#define R_START		0x10
#define R_END		0x18
#define R_WPTR		0x20
#define R_RPTR		0x28
#define R_MIU		0x40
#define R_CFG2		0x80
#define R_TOUT_LO	0xa0
#define R_TOUT_HI	0xa4
#define R_RESET		0xc0	/* hi (0xc4) bit0: engine reset, active low */
#define R_OPC		0xf0	/* hi (0xf4) low byte: opcode of the failed command */
#define R_STAT		0x110	/* hi (0x114): bit8 error, bit11 idle */
#define R_IMASK		0x118	/* hi (0x11c): 1 = masked */
#define R_ICLR		0x120
#define R_TIMER		0x128
#define R_DUMMY		0x1e0	/* software scratch register */

#define IRQ_ERR		0x1700	/* error flags of the status word */
#define OP_WRITE	0x10
#define OP_WAIT		0x20
#define OP_POLL_EQ	0x30
#define OP_POLL_NEQ	0xb0

/* RIU address of an engine register: bank (RIU word address) | reg index * 2 */
#define RIU_BANK(n)	(((n) == 0 ? 0x1120 : 0x1120 + 3 + ((n) - 1)) << 8)

/* chip register that the vendor driver sets (bit 0) before the first use */
#define CHIP_EN_PA	0x1f20025c

#define IF_WORDS	25
#define IF_PRIV		18

struct cmdq_eng {
	int id;
	struct device *dev;
	void __iomem *base;
	int irq;
	struct clk *clk;
	bool probed;
	bool used;
	bool direct;
	bool kicked;
	struct mutex lock;

	u32 area;		/* bus address of the ring memory */
	void *virt;
	u32 size;
	u32 start, end;		/* bus addresses, end = last 16 byte slot */
	u32 cur;		/* write cursor, bus address */
	u32 flushed;		/* up to here the ring is cache clean */
	u32 rdp;		/* last read pointer seen, 16 byte units */
	u32 wptr;		/* kicked write pointer, 16 byte units */
	u32 ncmd;		/* commands since the last kick */
	u32 cap;		/* ring capacity in commands */

	u32 iface[IF_WORDS];
	atomic_t cnt[IF_WORDS];
	atomic_t sum[IF_WORDS];
	atomic_t trace[IF_WORDS];
	atomic_t errirq;
};

static struct cmdq_eng engs[NENG];

/* ring memory handed over by mi_sys */
static struct {
	bool valid;
	u32 bus;
	void *virt;
	u32 size;
} pool;
static DEFINE_MUTEX(glock);
static u32 id_used;
static uint tout_lo = 0;
static uint tout_hi = 0xff07;
module_param(tout_lo, uint, 0444);
module_param(tout_hi, uint, 0444);
static int ntrace = 3;
module_param(ntrace, int, 0644);
MODULE_PARM_DESC(ntrace, "log the arguments of the first N calls of every interface member");

/* mi_sys ring pool areas */
static struct {
	u32 bus, size;
	bool used;
} area[NENG];

static atomic_t api_cnt[16];

/* ---- register access ---- */

static inline u16 r16(struct cmdq_eng *e, unsigned int off)
{
	return readw(e->base + off);
}

static inline void w16(struct cmdq_eng *e, unsigned int off, u16 v)
{
	writew(v, e->base + off);
}

/* 32 bit register update: high half first, then the low half latches it */
static void set32(struct cmdq_eng *e, unsigned int off, u32 clr, u32 set)
{
	u32 v = r16(e, off) | (u32)r16(e, off + 4) << 16;

	v = (v & ~clr) | set;
	w16(e, off + 4, v >> 16);
	w16(e, off, v & 0xffff);
}

static u32 r32(struct cmdq_eng *e, unsigned int off)
{
	return r16(e, off) | (u32)r16(e, off + 4) << 16;
}

static u32 hw_status(struct cmdq_eng *e)
{
	u16 st = r16(e, R_STAT + 4);

	r16(e, R_STAT);
	if (st & 0x100) {
		st &= ~0x100;
		switch (r16(e, R_OPC + 4) & 0xff) {
		case OP_POLL_NEQ:
			st |= 0x100;
			break;
		case OP_POLL_EQ:
			st |= 0x200;
			break;
		case OP_WAIT:
			st |= 0x400;
			break;
		case OP_WRITE:
			st |= 0x1000;
			break;
		}
	}
	return st;
}

static u32 hw_read_ptr(struct cmdq_eng *e)
{
	set32(e, R_MODE, 0, 0x100 << 16);
	return r32(e, R_RPTR);
}

/* engine soft reset: bit 0 of the high half low, then high again */
static void hw_reset_pulse(struct cmdq_eng *e)
{
	set32(e, R_RESET, 1 << 16, 0);
	set32(e, R_RESET, 0, 1 << 16);
}

static void hw_clear_irq(struct cmdq_eng *e, u16 flags)
{
	if (flags & IRQ_ERR)
		flags |= IRQ_ERR;
	w16(e, R_ICLR + 4, r16(e, R_ICLR + 4));
	w16(e, R_ICLR, r16(e, R_ICLR) | flags);
}

static void hw_mask_irq(struct cmdq_eng *e, u16 mask)
{
	w16(e, R_IMASK + 4, mask);
	w16(e, R_IMASK, r16(e, R_IMASK));
}

/* ---- ring ---- */

static void *ring_va(struct cmdq_eng *e, u32 bus)
{
	return e->virt + (bus - e->area);
}

static void put_cmd(struct cmdq_eng *e, u8 b0, u8 b1, u8 b2, u8 b3, u8 b4, u8 b5,
		    u8 b6, u8 b7)
{
	u8 *p = ring_va(e, e->cur);

	p[0] = b0;
	p[1] = b1;
	p[2] = b2;
	p[3] = b3;
	p[4] = b4;
	p[5] = b5;
	p[6] = b6;
	p[7] = b7;
	e->ncmd++;
	e->cur += 8;
	if (e->cur >= e->end + 16)
		e->cur = e->start;
}

static void put_reg_cmd(struct cmdq_eng *e, u8 op, u32 addr, u16 val, u16 mask)
{
	put_cmd(e, ~mask & 0xff, (~mask >> 8) & 0xff, val & 0xff, val >> 8,
		(addr >> 1) & 0xff, (addr >> 9) & 0xff, (addr >> 17) & 0x7f, op);
}

/* free command slots if at least n (rounded up to a multiple of 2) fit, else 0 */
static u32 ring_avail(struct cmdq_eng *e, u32 n)
{
	u32 rd, need = (n + 1) & ~1u, free;

	if (e->direct)
		return n;
	if (e->kicked) {
		e->rdp = hw_read_ptr(e);
		rd = e->rdp << 4;
	} else {
		rd = e->start;
	}
	if (rd > e->end || rd < e->start) {
		dev_err(e->dev, "read pointer %#x outside %#x..%#x\n", rd, e->start, e->end);
		return 0;
	}
	if (e->cur == rd)
		return e->cap >= need ? e->cap : 0;
	if (e->cur < rd) {
		free = (rd - e->cur) >> 3;
	} else {
		free = ((e->end - e->start - e->cur) + rd) >> 3;
		free++;
	}
	return need <= free ? free : 0;
}

/* like the vendor driver: wait up to 1.5 s for the engine to make room */
static u32 ring_wait_avail(struct cmdq_eng *e, u32 n)
{
	u32 r = ring_avail(e, n);
	int i;

	for (i = 0; !r && !e->direct && i < 150; i++) {
		msleep(10);
		r = ring_avail(e, n);
	}
	if (!r && !e->direct)
		dev_err(e->dev, "ring full: %u slots needed, cur %#x rd %#x wr %#x\n",
			n, e->cur, e->rdp << 4, e->wptr << 4);
	return r;
}

static void hw_dump(struct cmdq_eng *e)
{
	u32 rd = hw_read_ptr(e) << 4;
	u32 *c = ring_va(e, rd < e->start || rd > e->end ? e->start : rd);

	dev_err(e->dev, "id %d: rd %#x wr %#x opc %#x/%#x errcmd %#x %#x stat %#x tout %#x/%#x cmd %08x %08x\n",
		e->id, rd, r32(e, R_WPTR) << 4, r16(e, R_OPC), r16(e, R_OPC + 4), r32(e, 0x100),
		r32(e, 0x108), r32(e, R_STAT), r16(e, R_TOUT_HI), r16(e, R_TOUT_LO), c[0], c[1]);
}

static int do_kick(struct cmdq_eng *e)
{
	u32 st, n;
	int ret;

	if (e->direct)
		return 0;
	mutex_lock(&e->lock);
	if (e->cur & 0xf) {
		if (!ring_avail(e, 1)) {
			ret = -1;
			goto out;
		}
		put_cmd(e, 0xff, 0xff, 0, 0, 0, 0, 0, 0);
	}
	/* bit 8 of the status is set while the engine has not started yet */
	st = e->kicked ? hw_status(e) : 0;
	if (st & IRQ_ERR) {
		dev_err(e->dev, "id %d: error status %#x before kick, command %#x\n", e->id,
			st, r16(e, R_OPC + 4));
		hw_dump(e);
		ret = -1;
		goto out;
	}
	/* publish the commands queued since the last kick */
	if (e->cur != e->flushed) {
		u32 from = e->flushed & ~63u, to = (e->cur + 63) & ~63u;

		if (to > e->end + 16)
			to = e->end + 16;
		if (e->cur >= e->flushed) {
			Chip_Flush_Cache_Range((unsigned long)ring_va(e, from), to - from);
		} else {
			Chip_Flush_Cache_Range((unsigned long)ring_va(e, from), e->end + 16 - from);
			Chip_Flush_Cache_Range((unsigned long)ring_va(e, e->start),
					       ((e->cur + 63) & ~63u) - e->start);
		}
		e->flushed = e->cur;
	}
	n = e->ncmd;
	e->wptr = e->cur >> 4;
	e->kicked = true;
	e->ncmd = 0;
	hw_mask_irq(e, 0xfeff);
	w16(e, R_WPTR + 4, e->wptr >> 16);
	w16(e, R_WPTR, e->wptr & 0xffff);
	set32(e, R_MODE, 0, 2 << 16);
	ret = n;
out:
	mutex_unlock(&e->lock);
	return ret;
}

/* ---- interface members ---- */

static inline struct cmdq_eng *iface_eng(void *iface)
{
	return (struct cmdq_eng *)((u32 *)iface)[IF_PRIV];
}

#define MEMBER(w, name) \
	static long name(void *i, long a, long b, long c, long d)

static void note(struct cmdq_eng *e, int w, const char *name, long a, long b, long c, long d)
{
	atomic_inc(&e->cnt[w]);
	if (atomic_inc_return(&e->trace[w]) <= ntrace)
		dev_info(e->dev, "%llu ~ms id %d %s(%#lx, %#lx, %#lx, %#lx) st %#x\n",
			 ktime_get_ns() >> 20, e->id, name, a, b, c, d, r16(e, R_STAT + 4));
}

MEMBER(3, m_check_avail)	/* (n) -> free slots or 0 */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 3, "CheckBufAvailable", a, b, c, d);
	if (e->direct)
		return a;
	return ring_wait_avail(e, a);
}

static int queue_write(struct cmdq_eng *e, u8 op, u32 addr, u16 val, u16 mask)
{
	if (!ring_wait_avail(e, 1))
		return -1;
	put_reg_cmd(e, op, addr, val, mask);
	return 0;
}

static int do_write_mask(struct cmdq_eng *e, u32 addr, u16 val, u16 mask)
{
	int ret;

	if (e->direct) {
		void __iomem *p = osm_riu + (addr << 1);
		u16 old = readw(p);

		writew((old & ~mask) | (val & mask), p);
		return 0;
	}
	mutex_lock(&e->lock);
	ret = queue_write(e, OP_WRITE, addr, val, mask);
	mutex_unlock(&e->lock);
	return ret;
}

MEMBER(4, m_write_dummy)	/* (u16 value) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 4, "WriteDummyReg", a, b, c, d);
	if (e->direct) {
		w16(e, R_DUMMY + 4, 0);
		w16(e, R_DUMMY, a);
		return 0;
	}
	return do_write_mask(e, RIU_BANK(e->id) | 0xf0, a, 0xffff);
}

MEMBER(5, m_read_dummy)		/* (u16 *value) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 5, "ReadDummyReg", 0, 0, 0, 0);
	r16(e, R_DUMMY + 4);
	*(u16 *)a = r16(e, R_DUMMY);
	return 0;
}

MEMBER(6, m_write_mask)		/* (addr, value, mask) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 6, "WriteRegMask", a, b, c, d);
	return do_write_mask(e, a, b, c);
}

struct reg_mask {
	u32 addr;
	u16 val;
	u16 mask;
};

static long write_multi(struct cmdq_eng *e, const struct reg_mask *r, u32 n, bool masked)
{
	u32 done = 0, avail, chunk, k;

	if (e->direct) {
		for (k = 0; k < n; k++) {
			void __iomem *p = osm_riu + (r[k].addr << 1);
			u16 m = masked ? r[k].mask : 0xffff;

			writew((readw(p) & ~m) | (r[k].val & m), p);
		}
		return 0;
	}
	mutex_lock(&e->lock);
	while (done < n) {
		/* keep 4 slots for the padding command of the kick */
		avail = ring_wait_avail(e, 1);
		avail = avail > 4 ? avail - 4 : 0;
		if (!avail) {
			dev_err(e->dev, "ring full after %u of %u register writes\n", done, n);
			mutex_unlock(&e->lock);
			return done;
		}
		chunk = min(avail, n - done);
		for (k = 0; k < chunk; k++, done++)
			put_reg_cmd(e, OP_WRITE, r[done].addr, r[done].val,
				    masked ? r[done].mask : 0xffff);
	}
	mutex_unlock(&e->lock);
	return n;	/* the callers compare with the count */
}

MEMBER(7, m_write_mask_multi)	/* ({addr, val, mask}[], n) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 7, "WriteRegMaskMulti", a, b, c, d);
	atomic_add(b, &e->sum[7]);
	return write_multi(e, (const struct reg_mask *)a, b, true);
}

MEMBER(8, m_write)		/* (addr, value) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 8, "WriteReg", a, b, c, d);
	return do_write_mask(e, a, b, 0xffff);
}

MEMBER(9, m_write_multi)	/* ({addr, val}[], n) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 9, "WriteRegMulti", a, b, c, d);
	atomic_add(b, &e->sum[9]);
	return write_multi(e, (const struct reg_mask *)a, b, false);
}

/*
 * "Bridge" registers the vendor driver writes (as ring commands) around every
 * WAIT and POLL: per engine one slot in the CMDQ0 bank (bank 0x1120, reg
 * 0x0e/0x0f/0x2d/0x2e) that gets the logical event / opcode, and for polls
 * four registers in bank 0x1004 per (engine, eq/neq) with value, mask and
 * address. Rows: 0..3 poll eq, 4..7 poll neq (value, mask, addr high, addr low),
 * 8 event slot.
 */
static const u8 bridge_reg[9][NENG] = {
	{ 0x10, 0x11, 0x12, 0x13 }, { 0x18, 0x19, 0x1a, 0x1b },
	{ 0x20, 0x22, 0x24, 0x26 }, { 0x21, 0x23, 0x25, 0x27 },
	{ 0x14, 0x15, 0x16, 0x17 }, { 0x1c, 0x1d, 0x1e, 0x1f },
	{ 0x28, 0x2a, 0x2c, 0x2e }, { 0x29, 0x2b, 0x2d, 0x2f },
	{ 0x0e, 0x0f, 0x2d, 0x2e },
};
#define BRIDGE_POLL(e, row)	(0x100400 + 2 * bridge_reg[row][(e)->id])
#define BRIDGE_EVENT(e)		(0x112000 + 2 * bridge_reg[8][(e)->id])

MEMBER(10, m_poll)		/* (addr, value, mask, eq) */
{
	struct cmdq_eng *e = iface_eng(i);
	int ret;

	note(e, 10, "PollRegBits", a, b, c, d);
	if (e->direct)
		return 0;
	mutex_lock(&e->lock);
	{
		int r0 = d ? 0 : 4;
		u16 v[4] = { b, c, (u32)a >> 16, a & 0xffff };
		int k;

		ret = 0;
		for (k = 0; k < 4 && !ret; k++)
			ret = queue_write(e, OP_WRITE, BRIDGE_POLL(e, r0 + k), v[k], 0xffff);
		if (!ret)
			ret = queue_write(e, OP_WRITE, BRIDGE_EVENT(e), d ? OP_POLL_EQ : OP_POLL_NEQ,
					  0xffff);
	}
	if (!ret)
		ret = queue_write(e, d ? OP_POLL_EQ : OP_POLL_NEQ, a, b, c);
	mutex_unlock(&e->lock);
	return ret;
}

/* trigger events: logical id -> bit of the trigger bus (same for all engines) */
static const struct {
	u8 id;
	u16 mask;
} trig[] = {
	{ 25, 0x0001 }, { 15, 0x0002 }, { 24, 0x0004 }, { 23, 0x0008 },
	{ 22, 0x0010 }, { 21, 0x0020 }, { 29, 0x0040 }, { 20, 0x0080 },
	{ 19, 0x0100 }, { 18, 0x0200 }, { 6, 0x0400 }, { 17, 0x0800 },
	{ 28, 0x1000 }, { 27, 0x2000 }, { 26, 0x4000 }, { 1, 0x8000 },
};

static int clear_trigger(struct cmdq_eng *e, u16 mask)
{
	int k;

	if (e->direct)
		return 0;
	mutex_lock(&e->lock);
	if (!ring_wait_avail(e, 4)) {
		mutex_unlock(&e->lock);
		return -1;
	}
	/* four writes of the event to the trigger clear register of the engine */
	for (k = 0; k < 4; k++)
		put_reg_cmd(e, OP_WRITE, RIU_BANK(e->id) | 0xa8, mask, 0xffff);
	mutex_unlock(&e->lock);
	return 0;
}

MEMBER(11, m_wait_event)	/* (logical event) */
{
	struct cmdq_eng *e = iface_eng(i);
	u16 mask = 0;
	int k;

	note(e, 11, "AddWaitEvent", a, b, c, d);
	if (e->direct)
		return 0;
	for (k = 0; k < ARRAY_SIZE(trig); k++)
		if (trig[k].id == (a & 0xff))
			mask = trig[k].mask;
	if (!mask) {
		dev_err(e->dev, "unknown trigger event %ld\n", a);
		return -1;
	}
	mutex_lock(&e->lock);
	if (queue_write(e, OP_WRITE, BRIDGE_EVENT(e),
			(a & 0xff) == 254 ? 0x30 : (a & 0xff) == 255 ? 0xb0 : (a & 0xff) << 8 | 0x20,
			0xffff) || !ring_wait_avail(e, 1)) {
		mutex_unlock(&e->lock);
		return -1;
	}
	put_cmd(e, ~mask & 0xff, (~mask >> 8) & 0xff, 0, 0, 0, 0, 0, OP_WAIT);
	mutex_unlock(&e->lock);
	return clear_trigger(e, mask);
}

MEMBER(12, m_abort)		/* drop the commands queued since the last kick */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 12, "AbortBuffer", a, b, c, d);
	if (e->direct)
		return 0;
	mutex_lock(&e->lock);
	e->ncmd = 0;
	e->cur = e->wptr << 4;
	mutex_unlock(&e->lock);
	return 0;
}

MEMBER(13, m_reset)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 13, "ResetEngine", a, b, c, d);
	mutex_lock(&e->lock);
	e->ncmd = 0;
	e->cur = e->start;
	e->flushed = e->start;
	e->rdp = 0;
	e->wptr = 0;
	e->kicked = false;
	hw_reset_pulse(e);
	hw_clear_irq(e, IRQ_ERR);
	hw_mask_irq(e, 0xfeff);
	mutex_unlock(&e->lock);
	return 0;
}

MEMBER(14, m_status)		/* (u32 *status) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 14, "ReadStatus", a, b, c, d);
	mutex_lock(&e->lock);
	*(u32 *)a = hw_status(e);
	mutex_unlock(&e->lock);
	return 0;
}

MEMBER(15, m_kick)
{
	struct cmdq_eng *e = iface_eng(i);
	int n;

	note(e, 15, "KickOff", a, b, c, d);
	n = do_kick(e);
	atomic_add(n > 0 ? n : 0, &e->sum[15]);
	return n;
}

MEMBER(16, m_clear_trigger)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 16, "ClearTriggerEvent", a, b, c, d);
	return clear_trigger(e, 0xffff);
}

MEMBER(17, m_idle)		/* (u8 *idle) */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 17, "IsEmptyIdle", 0, 0, 0, 0);
	if (e->direct) {
		*(u8 *)a = 0;
		return 0;
	}
	mutex_lock(&e->lock);
	r16(e, R_STAT);
	*(u8 *)a = !!(r16(e, R_STAT + 4) & BIT(11));
	mutex_unlock(&e->lock);
	return 0;
}

MEMBER(19, m_kick_tag)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 19, "KickOffByTag", a, b, c, d);
	return do_kick(e);
}

MEMBER(20, m_delay)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 20, "CmdDelay", a, b, c, d);
	dev_warn_once(e->dev, "CmdDelay is not implemented\n");
	return -1;
}

MEMBER(21, m_cur_number)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 21, "GetCurrentNumber", a, b, c, d);
	return e->ncmd;
}

MEMBER(22, m_poll_time)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 22, "PollRegBits_ByTime", a, b, c, d);
	dev_warn_once(e->dev, "PollRegBits_ByTime is not implemented\n");
	return -1;
}

MEMBER(23, m_event_dummy_addr)	/* RIU address of the hardware dummy register */
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 23, "GetAppEventDummyRegAddr", a, b, c, d);
	return RIU_BANK(e->id) | 0xee;
}

MEMBER(24, m_weight)
{
	struct cmdq_eng *e = iface_eng(i);

	note(e, 24, "SetWeight", a, b, c, d);
	return 0;
}

static const void *const members[IF_WORDS] = {
	[3] = m_check_avail, [4] = m_write_dummy, [5] = m_read_dummy,
	[6] = m_write_mask, [7] = m_write_mask_multi, [8] = m_write,
	[9] = m_write_multi, [10] = m_poll, [11] = m_wait_event, [12] = m_abort,
	[13] = m_reset, [14] = m_status, [15] = m_kick, [16] = m_clear_trigger,
	[17] = m_idle, [19] = m_kick_tag, [20] = m_delay, [21] = m_cur_number,
	[22] = m_poll_time, [23] = m_event_dummy_addr, [24] = m_weight,
};

static const char *const member_name[IF_WORDS] = {
	[3] = "CheckBufAvailable", [4] = "WriteDummyReg", [5] = "ReadDummyReg",
	[6] = "WriteRegMask", [7] = "WriteRegMaskMulti", [8] = "WriteReg",
	[9] = "WriteRegMulti", [10] = "PollRegBits", [11] = "AddWaitEvent",
	[12] = "AbortBuffer", [13] = "ResetEngine", [14] = "ReadStatus",
	[15] = "KickOff", [16] = "ClearTriggerEvent", [17] = "IsEmptyIdle",
	[19] = "KickOffByTag", [20] = "CmdDelay", [21] = "GetCurrentNumber",
	[22] = "PollRegBits_ByTime", [23] = "GetAppEventDummyRegAddr",
	[24] = "SetWeight",
};

/* ---- hardware set-up ---- */

static void hw_init_chip(void)
{
	int k;

	/* first user: all engines quiet, error flags cleared and masked */
	writew(readw(osm_riu + (CHIP_EN_PA - 0x1f000000)) | 1, osm_riu + (CHIP_EN_PA - 0x1f000000));
	for (k = 0; k < NENG; k++) {
		if (!engs[k].probed)
			continue;
		hw_clear_irq(&engs[k], IRQ_ERR);
		hw_mask_irq(&engs[k], 0xffff);
	}
	if (engs[0].probed) {
		u32 v = r32(&engs[0], R_CTRL);

		if ((v & 3) <= 1)
			set32(&engs[0], R_CTRL, 0, 2);
	}
}

static void hw_enable(struct cmdq_eng *e)
{
	set32(e, R_CTRL, 0, 1 << 16);
	set32(e, R_MIU, 0, 0x190d << 16);
	set32(e, R_CFG2, 0, 0x80);
	w16(e, R_TOUT_HI, tout_hi);
	w16(e, R_TOUT_LO, tout_lo);
	hw_reset_pulse(e);
	set32(e, R_MODE, 0, 4);
	set32(e, R_TIMER, 0, 0xff);
	w16(e, R_ICLR, 0xffff);
	hw_clear_irq(e, IRQ_ERR);
	hw_mask_irq(e, 0xfeff);
}

static irqreturn_t cmdq_irq(int irq, void *data)
{
	struct cmdq_eng *e = data;
	u32 st;

	if (!e->used)
		return IRQ_NONE;
	st = hw_status(e);
	if (!(st & IRQ_ERR))
		return IRQ_NONE;
	atomic_inc(&e->errirq);
	dev_err_ratelimited(e->dev, "id %d error status %#x, command %#x, read pointer %#x\n",
			    e->id, st, r16(e, R_OPC + 4), r32(e, R_RPTR));
	hw_clear_irq(e, IRQ_ERR);
	/* keep the shared line quiet until the next kick */
	hw_mask_irq(e, 0xffff);
	return IRQ_HANDLED;
}

/* ---- MHAL_CMDQ_* ---- */

/* vendor ABI, called by mhal and mi_sys; no header declares them */
int MHAL_CMDQ_GetCmdqHWNumber(void);
int MHAL_CMDQ_GetSpecificId(int id);
int MHAL_CMDQ_GetFreeId(void);
int MHAL_CMDQ_ReleaseId(int id);
int MHAL_CMDQ_InitCmdqMmapInfo(const u32 *info);
int MHAL_CMDQ_DeinitCmdqMmapInfo(void);
void *MHAL_CMDQ_GetSysCmdqService(int id, const u32 *desc, u32 force_riu);
int MHAL_CMDQ_ReleaseSysCmdqService(int id);
int MHAL_CMDQ_DirectModeEarlyRun(long a, long b, long c, long d);
int MHAL_CMDQ_DirectModeEarlyRunStop(long a, long b, long c, long d);
int MHAL_CMDQ_GetEarlyRunStatus(long a, long b, long c, long d);
int MHAL_CMDQ_DirectModeByteModeCmd(long a, long b, long c, long d);

int MHAL_CMDQ_GetCmdqHWNumber(void)
{
	atomic_inc(&api_cnt[0]);
	return NENG;
}
EXPORT_SYMBOL(MHAL_CMDQ_GetCmdqHWNumber);

int MHAL_CMDQ_GetSpecificId(int id)
{
	int k;

	atomic_inc(&api_cnt[2]);
	mutex_lock(&glock);
	if (id < 0) {
		for (k = 0; k < NENG; k++)
			if (!(id_used & BIT(k)))
				break;
		id = k < NENG ? k : -1;
	} else if (id >= NENG || (id_used & BIT(id))) {
		id = -1;
	}
	if (id >= 0)
		id_used |= BIT(id);
	mutex_unlock(&glock);
	return id;
}
EXPORT_SYMBOL(MHAL_CMDQ_GetSpecificId);

int MHAL_CMDQ_GetFreeId(void)
{
	atomic_inc(&api_cnt[1]);
	return MHAL_CMDQ_GetSpecificId(-1);
}
EXPORT_SYMBOL(MHAL_CMDQ_GetFreeId);

int MHAL_CMDQ_ReleaseId(int id)
{
	atomic_inc(&api_cnt[3]);
	if (id >= 0 && id < NENG) {
		mutex_lock(&glock);
		id_used &= ~BIT(id);
		mutex_unlock(&glock);
	}
	return 0;
}
EXPORT_SYMBOL(MHAL_CMDQ_ReleaseId);

/* info: u32 words, [1] bus address, [2] kernel virtual address, [3] size */
int MHAL_CMDQ_InitCmdqMmapInfo(const u32 *info)
{
	atomic_inc(&api_cnt[4]);
	mutex_lock(&glock);
	if (!pool.valid) {
		pool.bus = info[1];
		pool.virt = (void *)info[2];
		pool.size = info[3];
		pool.valid = true;
		pr_info("osm_cmdq: ring pool bus %#x virt %p size %#x\n", pool.bus, pool.virt,
			pool.size);
	}
	mutex_unlock(&glock);
	return 0;
}
EXPORT_SYMBOL(MHAL_CMDQ_InitCmdqMmapInfo);

int MHAL_CMDQ_DeinitCmdqMmapInfo(void)
{
	int k, ret = 0;

	atomic_inc(&api_cnt[5]);
	mutex_lock(&glock);
	for (k = 0; k < NENG; k++)
		if (engs[k].used)
			ret = -1;
	if (!ret)
		pool.valid = false;
	mutex_unlock(&glock);
	return ret;
}
EXPORT_SYMBOL(MHAL_CMDQ_DeinitCmdqMmapInfo);

/* first fit in the pool, 16 byte aligned */
static int pool_alloc(struct cmdq_eng *e, u32 size)
{
	u32 pos = pool.bus;
	int k, best;

	size = ALIGN(size, 16);
	for (;;) {
		best = -1;
		for (k = 0; k < NENG; k++) {
			if (!area[k].used)
				continue;
			if (area[k].bus < pos + size && area[k].bus + area[k].size > pos &&
			    (best < 0 || area[k].bus + area[k].size > area[best].bus + area[best].size))
				best = k;
		}
		if (best < 0)
			break;
		pos = area[best].bus + area[best].size;
	}
	if (pos + size > pool.bus + pool.size)
		return -ENOMEM;
	area[e->id].bus = pos;
	area[e->id].size = size;
	area[e->id].used = true;
	e->area = pos;
	e->size = size;
	e->virt = pool.virt + (pos - pool.bus);
	return 0;
}

void *MHAL_CMDQ_GetSysCmdqService(int id, const u32 *desc, u32 force_riu)
{
	struct cmdq_eng *e;
	u32 size, align, k;
	bool first;

	atomic_inc(&api_cnt[6]);
	if (id < 0 || id >= NENG || !engs[id].probed) {
		pr_err("osm_cmdq: no engine %d\n", id);
		return NULL;
	}
	e = &engs[id];
	if (!pool.valid) {
		dev_err(e->dev, "no ring pool\n");
		return NULL;
	}
	if (!desc || !desc[0] || !((const u16 *)desc)[2]) {
		dev_err(e->dev, "invalid buffer description\n");
		return NULL;
	}
	size = desc[0];
	align = ((const u16 *)desc)[2];
	mutex_lock(&glock);
	if (e->used) {
		mutex_unlock(&glock);
		dev_err(e->dev, "id %d already in use\n", id);
		return NULL;
	}
	first = true;
	for (k = 0; k < NENG; k++)
		if (engs[k].used)
			first = false;
	size = ALIGN(size, align);
	if (pool_alloc(e, size)) {
		mutex_unlock(&glock);
		dev_err(e->dev, "no ring memory for %u bytes\n", size);
		return NULL;
	}
	if (first)
		hw_init_chip();
	e->direct = !!force_riu;
	e->start = e->area;
	e->end = e->area + e->size - 16;
	e->cur = e->start;
	e->flushed = e->start;
	e->rdp = 0;
	e->wptr = 0;
	e->ncmd = 0;
	e->cap = e->size >> 3;
	e->kicked = false;
	memset(e->virt, 0, e->size);
	hw_enable(e);
	/* ring memory: bus address in the MIU, MIU0 */
	set32(e, R_MIU, 2, 0);
	w16(e, R_START + 4, (e->start >> 4) >> 16);
	w16(e, R_START, (e->start >> 4) & 0xffff);
	w16(e, R_END + 4, (e->end >> 4) >> 16);
	w16(e, R_END, (e->end >> 4) & 0xffff);
	w16(e, R_WPTR + 4, (e->start >> 4) >> 16);
	w16(e, R_WPTR, (e->start >> 4) & 0xffff);
	hw_reset_pulse(e);
	Chip_Flush_Cache_Range((unsigned long)e->virt, e->size);
	memset(e->iface, 0, sizeof(e->iface));
	for (k = 0; k < IF_WORDS; k++)
		e->iface[k] = (u32)members[k];
	e->iface[IF_PRIV] = (u32)e;
	e->used = true;
	mutex_unlock(&glock);
	dev_info(e->dev, "id %d: ring bus %#x size %u virt %p, %s\n", id, e->area, e->size,
		 e->virt, e->direct ? "direct" : "queued");
	return e->iface;
}
EXPORT_SYMBOL(MHAL_CMDQ_GetSysCmdqService);

int MHAL_CMDQ_ReleaseSysCmdqService(int id)
{
	struct cmdq_eng *e;

	atomic_inc(&api_cnt[7]);
	if (id < 0 || id >= NENG)
		return -1;
	e = &engs[id];
	mutex_lock(&glock);
	if (!e->used) {
		mutex_unlock(&glock);
		return -1;
	}
	hw_mask_irq(e, 0xffff);
	set32(e, R_CTRL, 1 << 16, 0);
	e->used = false;
	area[id].used = false;
	mutex_unlock(&glock);
	dev_info(e->dev, "id %d released\n", id);
	return 0;
}
EXPORT_SYMBOL(MHAL_CMDQ_ReleaseSysCmdqService);

/* early run / byte mode: not used by the stack, not implemented */
#define UNSUPPORTED(name, n) \
	int MHAL_CMDQ_##name(long a, long b, long c, long d) \
	{ \
		atomic_inc(&api_cnt[n]); \
		pr_warn_once("osm_cmdq: MHAL_CMDQ_" #name " is not implemented\n"); \
		return -1; \
	} \
	EXPORT_SYMBOL(MHAL_CMDQ_##name)

UNSUPPORTED(DirectModeEarlyRun, 8);
UNSUPPORTED(DirectModeEarlyRunStop, 9);
UNSUPPORTED(GetEarlyRunStatus, 10);
UNSUPPORTED(DirectModeByteModeCmd, 11);

static void dump_stats(void)
{
	static const char *const api[12] = {
		"GetCmdqHWNumber", "GetFreeId", "GetSpecificId", "ReleaseId",
		"InitCmdqMmapInfo", "DeinitCmdqMmapInfo", "GetSysCmdqService",
		"ReleaseSysCmdqService", "DirectModeEarlyRun", "EarlyRunStop",
		"GetEarlyRunStatus", "DirectModeByteModeCmd",
	};
	int k, w;

	for (k = 0; k < 12; k++)
		pr_info("osm_cmdq: api %-24s %d\n", api[k], atomic_read(&api_cnt[k]));
	for (k = 0; k < NENG; k++) {
		if (!engs[k].probed)
			continue;
		for (w = 3; w < IF_WORDS; w++)
			if (atomic_read(&engs[k].cnt[w]))
				pr_info("osm_cmdq: id %d %-24s %d (sum %d)\n", k, member_name[w],
					atomic_read(&engs[k].cnt[w]), atomic_read(&engs[k].sum[w]));
		pr_info("osm_cmdq: id %d error interrupts %d\n", k, atomic_read(&engs[k].errirq));
	}
}

static int dump_set(const char *val, const struct kernel_param *kp)
{
	dump_stats();
	return 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(dump, &dump_ops, NULL, 0200);

/* ---- platform driver: one device per sstar,cmdq<N> node ---- */

static int cmdq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long id = (unsigned long)of_device_get_match_data(dev);
	struct cmdq_eng *e = &engs[id];
	int ret;

	e->id = id;
	e->dev = dev;
	mutex_init(&e->lock);
	e->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(e->base))
		return PTR_ERR(e->base);
	e->irq = platform_get_irq(pdev, 0);
	if (e->irq < 0)
		return e->irq;
	e->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(e->clk))
		return PTR_ERR(e->clk);
	hw_mask_irq(e, 0xffff);
	ret = devm_request_irq(dev, e->irq, cmdq_irq, IRQF_SHARED, dev_name(dev), e);
	if (ret)
		return ret;
	e->probed = true;
	dev_info(dev, "engine %lu, irq %d\n", id, e->irq);
	return 0;
}

static const struct of_device_id cmdq_of_match[] = {
	{ .compatible = "sstar,cmdq0", .data = (void *)0 },
	{ .compatible = "sstar,cmdq1", .data = (void *)1 },
	{ .compatible = "sstar,cmdq2", .data = (void *)2 },
	{ .compatible = "sstar,cmdq3", .data = (void *)3 },
	{ }
};
MODULE_DEVICE_TABLE(of, cmdq_of_match);

static struct platform_driver cmdq_driver = {
	.probe = cmdq_probe,
	.driver = {
		.name = "osm_cmdq",
		.of_match_table = cmdq_of_match,
	},
};

static int __init osm_cmdq_init(void)
{
	return platform_driver_register(&cmdq_driver);
}

static void __exit osm_cmdq_exit(void)
{
	dump_stats();
	platform_driver_unregister(&cmdq_driver);
}

module_init(osm_cmdq_init);
module_exit(osm_cmdq_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SSC338Q command queue engines for the mhal MHAL_CMDQ_* interface");
