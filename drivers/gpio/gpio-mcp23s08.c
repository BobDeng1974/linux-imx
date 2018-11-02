/*
 * MCP23S08 SPI/GPIO gpio expander driver
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcp23s08.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/circ_buf.h>
#include <linux/kthread.h>
#include <linux/delay.h>

/**
 * MCP types supported by driver
 */
#define MCP_TYPE_S08	0
#define MCP_TYPE_S17	1
#define MCP_TYPE_008	2
#define MCP_TYPE_017	3

/* Registers are all 8 bits wide.
 *
 * The mcp23s17 has twice as many bits, and can be configured to work
 * with either 16 bit registers or with two adjacent 8 bit banks.
 */
#define MCP_IODIR	0x00		/* init/reset:  all ones */
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#define IOCON_SEQOP	(1 << 5)
#define IOCON_HAEN	(1 << 3)
#define IOCON_ODR	(1 << 2)
#define IOCON_INTPOL	(1 << 1)
#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a

struct mcp23s08;

struct mcp23s08_ops {
	int	(*read)(struct mcp23s08 *mcp, unsigned reg);
	int	(*write)(struct mcp23s08 *mcp, unsigned reg, unsigned val);
	int	(*read_regs)(struct mcp23s08 *mcp, unsigned reg,
			     u16 *vals, unsigned n);
};

struct mcp23s08 {
	u8			addr;

	u16			cache[11];
	/* lock protects the cached values */
	struct mutex		lock;

	struct gpio_chip	chip;

	const struct mcp23s08_ops	*ops;
	void			*data; /* ops specific data */
};

/* A given spi_device can represent up to eight mcp23sxx chips
 * sharing the same chipselect but using different addresses
 * (e.g. chips #0 and #3 might be populated, but not #1 or $2).
 * Driver data holds all the per-chip data.
 */
struct mcp23s08_driver_data {
	unsigned		ngpio;
	struct mcp23s08		*mcp[8];
	struct mcp23s08		chip[];
};

/* CSI STUFF:
 *
 * Optimized way to get input mask. Otherwise a read on every input
 * pin is needed.
 */
#define REVISION "CSI: $Id: gpio-mcp23s08.c 15649 2015-12-03 13:52:02Z daniel.hilst $"
#define LOG_msg(pri, fmt, args...) \
        ({pr_ ## pri("mcp23s08 [" #pri "] %s@%d: " fmt "\n", __func__, __LINE__, ##args);})

#ifdef CONFIG_TRACING
# define LOG_debug_irq(fmt, args...) trace_printk("mcp23s08: %s@%d: " fmt "\n", __func__, __LINE__, ##args)
#else
# define LOG_debug_irq(fmt, args...)
#endif

#define LOG_debug(fmt, args...) LOG_msg(debug, fmt, ##args)
#define LOG_info(fmt, args...)  LOG_msg(info,  fmt, ##args)
#define LOG_warn(fmt, args...)  LOG_msg(warn,  fmt, ##args)
#define LOG_err(fmt, args...)   LOG_msg(err,   fmt, ##args)

static inline void __to_binary(char byte, char *buf, size_t bufsiz)
{
        int i;

        for (i = 0; i < bufsiz-1; i++)
                buf[i] = ((byte & (1 << i)) ? '1' : '0');
        buf[i] = '\0';
}

/* @TODO: remove this */
static struct mcp23s08 *mcp_global = NULL;
int mcp_getmask(void)
{
        int mask;
        if (!mcp_global)
                return -1;

        mask = mcp_global->ops->read(mcp_global, MCP_GPIO);
        if (mask < 0)
                return -2;

        return (0xff & ~(mask));
}
EXPORT_SYMBOL_GPL(mcp_getmask);

/* @TODO: remove this */
static DECLARE_WAIT_QUEUE_HEAD(mcp_irq_wq);
static unsigned mcp_input_change = 0;
static unsigned mcp_intcap;
static unsigned mcp_lost_irq = 0;
static unsigned mcp_total_irq = 0;
static unsigned mcp_waited_irq = 0;
int mcp_wait_irq(unsigned long timeout, unsigned mask, unsigned *intcap)
{
        int status;

        mcp_waited_irq++;
        do {
                unsigned long bfwait = jiffies;

                mcp_input_change = 0; /* reset condition */
                status = wait_event_interruptible_timeout(mcp_irq_wq, /* wait irq */
                                                          mcp_input_change,
                                                          timeout);
                if (status == 0) /* timeout */
                        return -ETIME;

                if ((status > 0) && /* interrupt happen before timeout */
                    mcp_input_change) {
                        *intcap = mcp_intcap;
                        return 0;
                }

                if (status == -ERESTARTSYS)
                        return -ERESTARTSYS;
                
                timeout = (bfwait + timeout) - jiffies;
        } while (timeout > 0);
        return -ETIME;
}
EXPORT_SYMBOL_GPL(mcp_wait_irq);

/* Circular queue, allocated at _probe()
 *
 *  NOTE: Applies only to mcp_global!
 */
static struct   circ_buf mcp_circ;
static               int mcp_circ_hit_full = 0;
static               int mcp_circ_produced = 0;
static               int mcp_circ_consumed = 0;
static               int mcp_circ_new_data = 0;
static DECLARE_WAIT_QUEUE_HEAD(mcp_circ_wq);
static        spinlock_t mcp_circ_prod_lock;
static struct      mutex mcp_circ_cons_lock;
int mcp_getlast(int *msk, unsigned int timeout)
{
        int rc;
        unsigned long head, tail;
        unsigned long tout = timeout * HZ / 1000; /* convert from ms to jiffies */

        BUG_ON(!msk);

        mutex_lock(&mcp_circ_cons_lock);
        for (;;) {
                head = ACCESS_ONCE(mcp_circ.head); /* read head */
                tail = mcp_circ.tail;              /* read tail */
        
                if (CIRC_CNT(head, tail, PAGE_SIZE) >= 1) {
                        char gpiobin[7];

                        smp_read_barrier_depends(); /* complete the reads before this line */
                        *msk = mcp_circ.buf[tail];  /* read the mask from queue */
                        smp_mb();
                        mcp_circ_consumed++;
                        mcp_circ.tail = (tail + 1) & (PAGE_SIZE - 1); /* update tail */
                        __to_binary(*msk, gpiobin, sizeof(gpiobin));
                        LOG_debug("GPIO: %s consumed", *msk);
                        rc = 0; /* return success */
                        break;  /* break loop */
                } else { 
                        LOG_debug("queue empty waiting");
                        mcp_circ_new_data = 0;
                        rc = wait_event_interruptible_timeout(mcp_circ_wq, mcp_circ_new_data, tout);
                        if (rc == -ERESTARTSYS) { /* interrupted  */
                                break;            /* return */
                        } else if (rc == 0) {     /* timeout */
                                rc = -ETIME;     
                                break;       /* return */
                        } else if (rc > 0) {      /* new input mask! */
                                continue; /* queue is not empty
                                           * anymore, start loop again
                                           * to get a new head and
                                           * tail */
                        }
                }
        }
        mutex_unlock(&mcp_circ_cons_lock);
        LOG_debug("returning %d", rc);
        return rc;
}
EXPORT_SYMBOL_GPL(mcp_getlast);

void mcp_flush_queue(void)
{
        LOG_debug("flushing events queue");
        mcp_circ.head = mcp_circ.tail = 0;
        mcp_circ_hit_full = 0;
        mcp_circ_produced = 0;
        mcp_circ_consumed = 0;
        memset(mcp_circ.buf, '\0', PAGE_SIZE);
}
EXPORT_SYMBOL_GPL(mcp_flush_queue);

/* Sysfs ---------------------------------------------------------------- */
ssize_t show_queue(struct device *dev, struct device_attribute *attr, char *buf)
{
        int head = ACCESS_ONCE(mcp_circ.head);
        int tail = ACCESS_ONCE(mcp_circ.tail);

        return snprintf(buf, PAGE_SIZE,
                        "Queue Size:      %5lu\n"
                        "Queue Available: %5lu\n"
                        "Queue Total:     %5lu\n"
                        "Queue Full hits: %5d\n"
                        "Events produced: %5d\n"
                        "Events consumed: %5d\n",
                        CIRC_CNT(head, tail, PAGE_SIZE),
                        CIRC_SPACE(head, tail, PAGE_SIZE),
                        PAGE_SIZE,
                        mcp_circ_hit_full,
                        mcp_circ_produced,
                        mcp_circ_consumed
                );
}
static DEVICE_ATTR(queue, 0400, show_queue, NULL);

ssize_t show_lost_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE,
                        "IRQs:   % 5d\n"
                        "Waited: % 5d\n"
                        "Lost:   % 5d\n",
                        mcp_total_irq,
                        mcp_waited_irq,
                        mcp_lost_irq);
}

ssize_t store_lost_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        mcp_lost_irq = 0;
        mcp_total_irq = 0;
        mcp_waited_irq = 0;
        return count;
}
static DEVICE_ATTR(lost_irq, 0600, show_lost_irq, store_lost_irq);

ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	int				addr, reg, off = 0;
        struct spi_device               *spi = to_spi_device(dev);
	struct mcp23s08_driver_data	*data = spi_get_drvdata(spi);
        int type = spi_get_device_id(spi)->driver_data;

        for (addr = 0; data->mcp[addr]; addr++) {
                struct mcp23s08	*mcp = data->mcp[addr];

                mutex_lock(&mcp->lock);
                mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
                mutex_unlock(&mcp->lock);

                off += snprintf(buf + off, PAGE_SIZE - off, "CHIP %d:\n--\n", addr);

                if (type == MCP_TYPE_S17)
                        off += snprintf(buf + off, PAGE_SIZE - off, "REG =>  B A\n");

                for (reg = 0; reg < ARRAY_SIZE(mcp->cache); reg++)
                        off += snprintf(buf + off, PAGE_SIZE - off, "%02xh => %04xh\n",
                                        type == MCP_TYPE_S17 ? reg << 1 : reg,
                                        mcp->cache[reg]);

        }

        return off;
}

ssize_t store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
        struct spi_device               *spi = to_spi_device(dev);
	struct mcp23s08_driver_data	*data = spi_get_drvdata(spi);
        u8                              regaddr;
        u16                             regval;
        int                             chipaddr, status;
        struct mcp23s08                 *mcp;
        int                             type = spi_get_device_id(spi)->driver_data;

        status = sscanf(buf, "%x:%hhx:%hx", &chipaddr, &regaddr, &regval);

        if ((chipaddr < 0  ||
             chipaddr > 3) ||
            (regaddr < 0   ||
             regaddr > 22))
                return -EINVAL;

        if (type == MCP_TYPE_S17)
                regaddr >>= 1;

        mcp = data->mcp[chipaddr];
        mcp->ops->write(mcp, regaddr, regval);

        return count;
}
static DEVICE_ATTR(registers, 0600, show_reg, store_reg);

static struct attribute *mcp_attrs[] = {
        &dev_attr_registers.attr,
        &dev_attr_lost_irq.attr,
        &dev_attr_queue.attr,
        NULL,
};

static struct attribute_group mcp_attr_group = {
        .attrs = mcp_attrs,
}
;
/* IRQ/polling ------------------------------------------------------------- */
static void mcp23s17_produce_enqueue(u8 evt)
{
        unsigned long flags;
        spin_lock_irqsave(&mcp_circ_prod_lock, flags);
        do {
                int head = mcp_circ.head;
                int tail = ACCESS_ONCE(mcp_circ.tail);
 
                /* Check if there is space on buffer. */
                if (CIRC_SPACE(head, tail, PAGE_SIZE) >= 1) {
                        u8 v = (0xff & ~(evt));
                        if (mcp_intcap == v)
                                LOG_debug_irq("mcp23s08: fake input change %02x, %02x",
                                              mcp_intcap, v);
                        mcp_intcap = v;
                        mcp_circ.buf[head] = v;
                        smp_wmb();
                        mcp_circ_produced++;
                        mcp_circ.head = (head + 1) & (PAGE_SIZE - 1);
                } else  {
                        mcp_circ_hit_full++; /* keep track of full hits */
                        break;
                }
        } while (0);
        spin_unlock_irqrestore(&mcp_circ_prod_lock, flags); 
}

static struct task_struct *mcp_polling_taskp;
static int polling = 1;
module_param(polling, int, 0);
MODULE_PARM_DESC(polling, "Use polling instead of interrupt, 1 = polling, 0 = IRQ, default 1");
static int polling_cycle_ms = 10;
module_param(polling_cycle_ms, int, 0);
MODULE_PARM_DESC(polling_cycle_ms, "Cycle in ms between two reads while polling, default 10ms");
static int mcp_poll(void *data)
{
        struct mcp23s08 *mcp = data;
        int old_gpio = 0;

        while (!kthread_should_stop()) {
                int gpio = mcp->ops->read(mcp, MCP_GPIO);

                if (gpio < 0) {
                        LOG_err("SPI error while polling");
                        continue;
                }

                if (gpio != old_gpio) {
                        char gpiobin[7];
                        __to_binary(~(0xff & gpio), gpiobin, sizeof(gpiobin));
                        LOG_debug("GPIO: %s produced", gpiobin);
                        mcp23s17_produce_enqueue(gpio);
                        old_gpio = gpio;
                        mcp_circ_new_data = 1;
                        wake_up_interruptible(&mcp_circ_wq);

                        mcp_input_change = 1;
                        wake_up_interruptible(&mcp_irq_wq);
                }

                msleep(polling_cycle_ms);
        }

        return 0;
}

static irqreturn_t mcp_hard_irq(int irq, void *handle)
{
        mcp_total_irq++;
        LOG_debug_irq("IRQ triggered, #%d", mcp_total_irq);
        return IRQ_WAKE_THREAD;
}


static int unlock_irq_tries = 25;
module_param(unlock_irq_tries, int, 0600);
MODULE_PARM_DESC(unlock_irq_tries, "Number of unlock IRQ tries before giveup");
static irqreturn_t mcp_irq(int irq, void *handle)
{
        u8 gpio;
        struct mcp23s08 *mcp = handle;
        int tries = unlock_irq_tries;
        char gpiobin[7];

        do {
                gpio = mcp->ops->read(mcp, MCP_GPIO); /* clear pending IRQ */
                __to_binary(~(0xff & gpio), gpiobin, sizeof(gpiobin));
#if defined(CONFIG_TRACING)
                        LOG_debug_irq("GPIO: %s (#%d)",
                                      gpiobin, mcp_total_irq);
#else
                        LOG_debug("GPIO: %s (#%d)",
                                  gpiobin, mcp_total_irq);
#endif
                if (gpio >= 0) { /* gpio has been readed with success */
                        mcp23s17_produce_enqueue(gpio); /* enqueue new value */
                } else {
                        LOG_debug("SPI error");
                }
        } while (gpio < 0 && tries--); /* retry while gpio read fail
                                        * and there are tries remaining */

        if (unlikely(!tries)) { /* failed after @unlock_irq_tries times */
                LOG_err("CAN'T COMMUNICATE TO SPI!!!");
                WARN_ON(!tries);
        }

        mcp_circ_new_data = 1;
        wake_up_interruptible(&mcp_circ_wq);

        mcp_input_change = 1;
        wake_up_interruptible(&mcp_irq_wq);

        return IRQ_HANDLED;
}

/* end of CSI STUFF */
/*----------------------------------------------------------------------*/

#if IS_ENABLED(CONFIG_I2C)

static int mcp23008_read(struct mcp23s08 *mcp, unsigned reg)
{
	return i2c_smbus_read_byte_data(mcp->data, reg);
}

static int mcp23008_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	return i2c_smbus_write_byte_data(mcp->data, reg, val);
}

static int
mcp23008_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	while (n--) {
		int ret = mcp23008_read(mcp, reg++);
		if (ret < 0)
			return ret;
		*vals++ = ret;
	}

	return 0;
}

static int mcp23017_read(struct mcp23s08 *mcp, unsigned reg)
{
	return i2c_smbus_read_word_data(mcp->data, reg << 1);
}

static int mcp23017_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	return i2c_smbus_write_word_data(mcp->data, reg << 1, val);
}

static int
mcp23017_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	while (n--) {
		int ret = mcp23017_read(mcp, reg++);
		if (ret < 0)
			return ret;
		*vals++ = ret;
	}

	return 0;
}

static const struct mcp23s08_ops mcp23008_ops = {
	.read		= mcp23008_read,
	.write		= mcp23008_write,
	.read_regs	= mcp23008_read_regs,
};

static const struct mcp23s08_ops mcp23017_ops = {
	.read		= mcp23017_read,
	.write		= mcp23017_write,
	.read_regs	= mcp23017_read_regs,
};

#endif /* CONFIG_I2C */

/*----------------------------------------------------------------------*/

#ifdef CONFIG_SPI_MASTER

static int mcp23s08_read(struct mcp23s08 *mcp, unsigned reg)
{
	u8	tx[2], rx[1];
	int	status;

	tx[0] = mcp->addr | 0x01;
	tx[1] = reg;
	status = spi_write_then_read(mcp->data, tx, sizeof tx, rx, sizeof rx);
	return (status < 0) ? status : rx[0];
}

static int mcp23s08_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	u8	tx[3];

	tx[0] = mcp->addr;
	tx[1] = reg;
	tx[2] = val;
	return spi_write_then_read(mcp->data, tx, sizeof tx, NULL, 0);
}

static int
mcp23s08_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	u8	tx[2], *tmp;
	int	status;

	if ((n + reg) > sizeof mcp->cache)
		return -EINVAL;
	tx[0] = mcp->addr | 0x01;
	tx[1] = reg;

	tmp = (u8 *)vals;
	status = spi_write_then_read(mcp->data, tx, sizeof tx, tmp, n);
	if (status >= 0) {
		while (n--)
			vals[n] = tmp[n]; /* expand to 16bit */
	}
	return status;
}

static int mcp23s17_read(struct mcp23s08 *mcp, unsigned reg)
{
	u8	tx[2], rx[2];
	int	status;

	tx[0] = mcp->addr | 0x01;
	tx[1] = reg << 1;
	status = spi_write_then_read(mcp->data, tx, sizeof tx, rx, sizeof rx);
	return (status < 0) ? status : (rx[0] | (rx[1] << 8));
}

static int mcp23s17_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	u8	tx[4];

	tx[0] = mcp->addr;
	tx[1] = reg << 1;
	tx[2] = val;
	tx[3] = val >> 8;
	return spi_write_then_read(mcp->data, tx, sizeof tx, NULL, 0);
}

static int
mcp23s17_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	u8	tx[2];
	int	status;

	if ((n + reg) > sizeof mcp->cache)
		return -EINVAL;
	tx[0] = mcp->addr | 0x01;
	tx[1] = reg << 1;

	status = spi_write_then_read(mcp->data, tx, sizeof tx,
				     (u8 *)vals, n * 2);
	if (status >= 0) {
		while (n--)
			vals[n] = __le16_to_cpu((__le16)vals[n]);
	}

	return status;
}

static const struct mcp23s08_ops mcp23s08_ops = {
	.read		= mcp23s08_read,
	.write		= mcp23s08_write,
	.read_regs	= mcp23s08_read_regs,
};

static const struct mcp23s08_ops mcp23s17_ops = {
	.read		= mcp23s17_read,
	.write		= mcp23s17_write,
	.read_regs	= mcp23s17_read_regs,
};

#endif /* CONFIG_SPI_MASTER */

/*----------------------------------------------------------------------*/

static int mcp23s08_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	int status;

	mutex_lock(&mcp->lock);
	mcp->cache[MCP_IODIR] |= (1 << offset);
	status = mcp->ops->write(mcp, MCP_IODIR, mcp->cache[MCP_IODIR]);
	mutex_unlock(&mcp->lock);
	return status;
}

static int mcp23s08_get(struct gpio_chip *chip, unsigned offset)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	int status;

	mutex_lock(&mcp->lock);

	/* REVISIT reading this clears any IRQ ... */
	status = mcp->ops->read(mcp, MCP_GPIO);
	if (status < 0)
		status = 0;
	else {
		mcp->cache[MCP_GPIO] = status;
		status = !!(status & (1 << offset));
	}
	mutex_unlock(&mcp->lock);
	return status;
}

static int __mcp23s08_set(struct mcp23s08 *mcp, unsigned mask, int value)
{
	unsigned olat = mcp->cache[MCP_OLAT];

	if (value)
		olat |= mask;
	else
		olat &= ~mask;
	mcp->cache[MCP_OLAT] = olat;
	return mcp->ops->write(mcp, MCP_OLAT, olat);
}

static void mcp23s08_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	unsigned mask = 1 << offset;

	mutex_lock(&mcp->lock);
	__mcp23s08_set(mcp, mask, value);
	mutex_unlock(&mcp->lock);
}

static int
mcp23s08_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	unsigned mask = 1 << offset;
	int status;

	mutex_lock(&mcp->lock);
	status = __mcp23s08_set(mcp, mask, value);
	if (status == 0) {
		mcp->cache[MCP_IODIR] &= ~mask;
		status = mcp->ops->write(mcp, MCP_IODIR, mcp->cache[MCP_IODIR]);
	}
	mutex_unlock(&mcp->lock);
	return status;
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_DEBUG_FS

#include <linux/seq_file.h>

/*
 * This shows more info than the generic gpio dump code:
 * pullups, deglitching, open drain drive.
 */
static void mcp23s08_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct mcp23s08	*mcp;
	char		bank;
	int		t;
	unsigned	mask;

	mcp = container_of(chip, struct mcp23s08, chip);

	/* NOTE: we only handle one bank for now ... */
	bank = '0' + ((mcp->addr >> 1) & 0x7);

	mutex_lock(&mcp->lock);
	t = mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
	if (t < 0) {
		seq_printf(s, " I/O ERROR %d\n", t);
		goto done;
	}

	for (t = 0, mask = 1; t < chip->ngpio; t++, mask <<= 1) {
		const char	*label;

		label = gpiochip_is_requested(chip, t);
		if (!label)
			continue;

		seq_printf(s, " gpio-%-3d P%c.%d (%-12s) %s %s %s",
			chip->base + t, bank, t, label,
			(mcp->cache[MCP_IODIR] & mask) ? "in " : "out",
			(mcp->cache[MCP_GPIO] & mask) ? "hi" : "lo",
			(mcp->cache[MCP_GPPU] & mask) ? "up" : "  ");
		/* NOTE:  ignoring the irq-related registers */
		seq_printf(s, "\n");
	}
done:
	mutex_unlock(&mcp->lock);
}

#else
#define mcp23s08_dbg_show	NULL
#endif

/*----------------------------------------------------------------------*/

static int mcp23s08_probe_one(struct mcp23s08 *mcp, struct device *dev,
			      void *data, unsigned addr,
			      unsigned type, unsigned base, unsigned pullups)
{
	int status;

        LOG_debug("");
        LOG_debug("initializing driver data lock");
	mutex_init(&mcp->lock);

	mcp->data = data;
	mcp->addr = addr;

	mcp->chip.direction_input = mcp23s08_direction_input;
	mcp->chip.get = mcp23s08_get;
	mcp->chip.direction_output = mcp23s08_direction_output;
	mcp->chip.set = mcp23s08_set;
	mcp->chip.dbg_show = mcp23s08_dbg_show;
#ifdef CONFIG_OF
	mcp->chip.of_gpio_n_cells = 2;
	mcp->chip.of_node = dev->of_node;
#endif

	switch (type) {
#ifdef CONFIG_SPI_MASTER
	case MCP_TYPE_S08:
		mcp->ops = &mcp23s08_ops;
		mcp->chip.ngpio = 8;
		mcp->chip.label = "mcp23s08";
		break;

	case MCP_TYPE_S17:
		mcp->ops = &mcp23s17_ops;
		mcp->chip.ngpio = 16;
		mcp->chip.label = "mcp23s17";
		break;
#endif /* CONFIG_SPI_MASTER */

#if IS_ENABLED(CONFIG_I2C)
	case MCP_TYPE_008:
		mcp->ops = &mcp23008_ops;
		mcp->chip.ngpio = 8;
		mcp->chip.label = "mcp23008";
		break;

	case MCP_TYPE_017:
		mcp->ops = &mcp23017_ops;
		mcp->chip.ngpio = 16;
		mcp->chip.label = "mcp23017";
		break;
#endif /* CONFIG_I2C */

	default:
		dev_err(dev, "invalid device type (%d)\n", type);
		return -EINVAL;
	}
        LOG_debug("Chip type %s", mcp->chip.label);

	mcp->chip.base = base;
	mcp->chip.can_sleep = 1;
	mcp->chip.parent = dev;
	mcp->chip.owner = THIS_MODULE;

	/* verify MCP_IOCON.SEQOP = 0, so sequential reads work,
	 * and MCP_IOCON.HAEN = 1, so we work with all chips.
	 */
        LOG_debug("Configure IOCON");
	status = mcp->ops->read(mcp, MCP_IOCON);
	if (status < 0)
		goto fail;
	if ((status & IOCON_SEQOP) || !(status & IOCON_HAEN)) {
		/* mcp23s17 has IOCON twice, make sure they are in sync */
		status &= ~(IOCON_SEQOP | (IOCON_SEQOP << 8));
		status |= IOCON_HAEN | (IOCON_HAEN << 8);
		status &= ~(IOCON_INTPOL | (IOCON_INTPOL << 8));
                status |= IOCON_ODR | (IOCON_ODR << 8);

		status = mcp->ops->write(mcp, MCP_IOCON, status);
		if (status < 0)
			goto fail;
	}

	/* configure ~100K pullups */
        LOG_debug("Configure 100k pullups");
	status = mcp->ops->write(mcp, MCP_GPPU, 0x00ff);
	if (status < 0)
		goto fail;


        /* Setup I/O directions, GPA is input, GPB is output */
        LOG_debug("Configure IODIR");
        status = mcp->ops->write(mcp, MCP_IODIR, 0x00ff);
        if (status < 0)
                goto fail;

	/* disable inverter on input */
        LOG_debug("Disabling IPOL on input");
        status = mcp->ops->write(mcp, MCP_IPOL, 0);
        if (status < 0)
                goto fail;


        /* turn off output */
        LOG_debug("Turning output off");
        status = mcp->ops->write(mcp, MCP_GPIO, 0xff00);
        if (status < 0)
                goto fail;

	/* enable irqs */
        if (!polling) {
                LOG_debug("Enabling IRQ");
                status = mcp->ops->write(mcp, MCP_GPINTEN, 0x00ff);
                if (status < 0)
                        goto fail;
        }

        /* Update registers cache */
        LOG_debug("Updating registers cache");
	status = mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
	if (status < 0)
		goto fail;

        /* adding GPIO chip */
        LOG_debug("Registering gpio chip %d address %d", (~0x40 & addr), addr);
	status = gpiochip_add(&mcp->chip);
fail:
	if (status < 0)
		dev_dbg(dev, "can't setup chip %d, --> %d\n",
			addr, status);
        
        LOG_debug("Returning %d", status);
	return status;
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_OF
#ifdef CONFIG_SPI_MASTER
static struct of_device_id mcp23s08_spi_of_match[] = {
	{
		.compatible = "mcp,mcp23s08", .data = (void *) MCP_TYPE_S08,
	},
	{
		.compatible = "mcp,mcp23s17", .data = (void *) MCP_TYPE_S17,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, mcp23s08_spi_of_match);
#endif

#if IS_ENABLED(CONFIG_I2C)
static struct of_device_id mcp23s08_i2c_of_match[] = {
	{
		.compatible = "mcp,mcp23008", .data = (void *) MCP_TYPE_008,
	},
	{
		.compatible = "mcp,mcp23017", .data = (void *) MCP_TYPE_017,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, mcp23s08_i2c_of_match);
#endif
#endif /* CONFIG_OF */


#if IS_ENABLED(CONFIG_I2C)

static int mcp230xx_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct mcp23s08_platform_data *pdata;
	struct mcp23s08 *mcp;
	int status, base, pullups;
	const struct of_device_id *match;

	match = of_match_device(of_match_ptr(mcp23s08_i2c_of_match),
					&client->dev);
	if (match) {
		base = -1;
		pullups = 0;
	} else {
		pdata = client->dev.platform_data;
		if (!pdata || !gpio_is_valid(pdata->base)) {
			dev_dbg(&client->dev,
					"invalid or missing platform data\n");
			return -EINVAL;
		}
		base = pdata->base;
		pullups = pdata->chip[0].pullups;
	}

	mcp = kzalloc(sizeof *mcp, GFP_KERNEL);
	if (!mcp)
		return -ENOMEM;

	status = mcp23s08_probe_one(mcp, &client->dev, client, client->addr,
				    id->driver_data, base, pullups);
	if (status)
		goto fail;

	i2c_set_clientdata(client, mcp);

	return 0;

fail:
	kfree(mcp);

	return status;
}

static int mcp230xx_remove(struct i2c_client *client)
{
	struct mcp23s08 *mcp = i2c_get_clientdata(client);

	gpiochip_remove(&mcp->chip);
	kfree(mcp);

	return 0;
}

static const struct i2c_device_id mcp230xx_id[] = {
	{ "mcp23008", MCP_TYPE_008 },
	{ "mcp23017", MCP_TYPE_017 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mcp230xx_id);

static struct i2c_driver mcp230xx_driver = {
	.driver = {
		.name	= "mcp230xx",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mcp23s08_i2c_of_match),
	},
	.probe		= mcp230xx_probe,
	.remove		= mcp230xx_remove,
	.id_table	= mcp230xx_id,
};

static int __init mcp23s08_i2c_init(void)
{
	return i2c_add_driver(&mcp230xx_driver);
}

static void mcp23s08_i2c_exit(void)
{
	i2c_del_driver(&mcp230xx_driver);
}

#else

static int __init mcp23s08_i2c_init(void) { return 0; }
static void mcp23s08_i2c_exit(void) { }

#endif /* CONFIG_I2C */

/*----------------------------------------------------------------------*/

#ifdef CONFIG_SPI_MASTER

static int mcp23s08_probe(struct spi_device *spi)
{
	struct mcp23s08_platform_data	*pdata;
	unsigned			addr;
	unsigned			chips = 0;
	struct mcp23s08_driver_data	*data;
	int				status, type;
	unsigned			base = -1,
					ngpio = 0,
					pullups[ARRAY_SIZE(pdata->chip)];
	const struct			of_device_id *match;
	u32				spi_present_mask = 0;

        LOG_debug("");
	match = of_match_device(of_match_ptr(mcp23s08_spi_of_match), &spi->dev);
	if (match) {
                LOG_debug("device tree detected");
		type = (int)match->data;
		status = of_property_read_u32(spi->dev.of_node,
				"mcp,spi-present-mask", &spi_present_mask);
		if (status) {
			dev_err(&spi->dev, "DT has no spi-present-mask\n");
			return -ENODEV;
		}
                LOG_debug("spi-present-mask %x", spi_present_mask);
		if ((spi_present_mask <= 0) || (spi_present_mask >= 256)) {
			dev_err(&spi->dev, "invalid spi-present-mask\n");
			return -ENODEV;
		}

		for (addr = 0; addr < ARRAY_SIZE(pdata->chip); addr++) {
			pullups[addr] = 0;
			if (spi_present_mask & (1 << addr))
				chips++;
                }
	} else {
                LOG_debug("no device tree detected");
		type = spi_get_device_id(spi)->driver_data;
		pdata = spi->dev.platform_data;
		if (!pdata || !gpio_is_valid(pdata->base)) {
			dev_dbg(&spi->dev,
					"invalid or missing platform data\n");
			return -EINVAL;
		}

		for (addr = 0; addr < ARRAY_SIZE(pdata->chip); addr++) {
			if (!pdata->chip[addr].is_present)
				continue;
			chips++;
			if ((type == MCP_TYPE_S08) && (addr > 3)) {
				dev_err(&spi->dev,
					"mcp23s08 only supports address 0..3\n");
				return -EINVAL;
			}
			spi_present_mask |= 1 << addr;
			pullups[addr] = pdata->chip[addr].pullups;
		}

		base = pdata->base;
	}

        LOG_debug("%d chips detected", chips);
        if (!chips)
                return -ENODEV;

        LOG_debug("allocating driver data");
	data = kzalloc(sizeof *data + chips * sizeof(struct mcp23s08),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	spi_set_drvdata(spi, data);

	for (addr = 0; addr < ARRAY_SIZE(pdata->chip); addr++) {
		if (!(spi_present_mask & (1 << addr)))
			continue;
		chips--;
		data->mcp[addr] = &data->chip[chips];
		status = mcp23s08_probe_one(data->mcp[addr], &spi->dev, spi,
					    0x40 | (addr << 1), type, base,
					    pullups[addr]);
		if (status < 0)
			goto fail;

		if (base != -1)
			base += (type == MCP_TYPE_S17) ? 16 : 8;
		ngpio += (type == MCP_TYPE_S17) ? 16 : 8;
	}
        /* optimization for CSI digital IO */
        LOG_debug("Setup CSI optimizations");
        mcp_global = data->mcp[0];
        LOG_debug("Allocating page for mcp_circ");
        mcp_circ.buf = (unsigned char *)get_zeroed_page(GFP_KERNEL); /* alloc circular buffer */
        if (!mcp_circ.buf)
                return -ENOMEM;
        mcp_circ.head = mcp_circ.tail = 0; /* initialize head & tail */
        spin_lock_init(&mcp_circ_prod_lock);
        mutex_init(&mcp_circ_cons_lock);
        /* enfof: optimization for CSI digital IO */
	data->ngpio = ngpio;

        if (polling) {
                LOG_debug("Starting polling thread");
                mcp_polling_taskp = kthread_run(mcp_poll, mcp_global, "mcp23s08-poll");
                if (!mcp_polling_taskp) {
                        LOG_err("Can't start polling thread");
                        goto fail;
                }
        } else {
                LOG_debug("Registering IRQ");
                status = request_threaded_irq(spi->irq, mcp_hard_irq, mcp_irq, IRQF_TRIGGER_FALLING, "MCP23sXX IRQ", mcp_global);
                if (status) {
                        dev_err(&spi->dev, "can't request IRQ");
                        goto fail;
                }
        }

        LOG_debug("Creating sysfs attributes");
        status = sysfs_create_group(&spi->dev.kobj, &mcp_attr_group);
        if (status) {
                dev_err(&spi->dev, "Can't create sysfs attributes");
                goto fail2;
        }

        LOG_debug("Returning success");
	return 0;

fail2:
        free_irq(spi->irq, mcp_global);
fail:
	for (addr = 0; addr < ARRAY_SIZE(data->mcp); addr++) {
		if (!data->mcp[addr])
			continue;
                LOG_debug("Removing gpio chip %d", addr);
		gpiochip_remove(&data->mcp[addr]->chip);
	}
        LOG_debug("Freeing driver data");
	kfree(data);
	return status;

}

static int mcp23s08_remove(struct spi_device *spi)
{
	struct mcp23s08_driver_data	*data = spi_get_drvdata(spi);
	unsigned			addr;

        if (polling) {
                LOG_debug("Stop polling");
                kthread_stop(mcp_polling_taskp);
        } else {
                LOG_debug("Disabling IRQ");
                mcp_global->ops->write(mcp_global, MCP_GPINTEN, 0x0000);
                LOG_debug("Freeing IRQ");
                free_irq(spi->irq, mcp_global);
        }

        LOG_debug("Removing sysfs attributes");
        sysfs_remove_group(&spi->dev.kobj, &mcp_attr_group);

        LOG_debug("Freeing circular buffer");
        free_page((unsigned long)mcp_circ.buf);

	for (addr = 0; addr < ARRAY_SIZE(data->mcp); addr++) {
		if (!data->mcp[addr])
			continue;

                LOG_debug("Removing chip %d", addr);
		gpiochip_remove(&data->mcp[addr]->chip);
		
	}
        
	LOG_debug("Freeing driver data");
	kfree(data);

	return 0;
}

static const struct spi_device_id mcp23s08_ids[] = {
	{ "mcp23s08", MCP_TYPE_S08 },
	{ "mcp23s17", MCP_TYPE_S17 },
	{ },
};
MODULE_DEVICE_TABLE(spi, mcp23s08_ids);

static struct spi_driver mcp23s08_driver = {
	.probe		= mcp23s08_probe,
	.remove		= mcp23s08_remove,
	.id_table	= mcp23s08_ids,
	.driver = {
		.name	= "mcp23s08",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mcp23s08_spi_of_match),
	},
};

static int __init mcp23s08_spi_init(void)
{
        LOG_debug("Registering spi driver");
	return spi_register_driver(&mcp23s08_driver);
}

static void mcp23s08_spi_exit(void)
{
        LOG_debug("Unregistering spi driver");
	spi_unregister_driver(&mcp23s08_driver);
}

#else

static int __init mcp23s08_spi_init(void) { return 0; }
static void mcp23s08_spi_exit(void) { }

#endif /* CONFIG_SPI_MASTER */

/*----------------------------------------------------------------------*/

static int __init mcp23s08_init(void)
{
	int ret;

        LOG_info(REVISION);

	ret = mcp23s08_spi_init();
	if (ret)
		goto spi_fail;

	ret = mcp23s08_i2c_init();
	if (ret)
		goto i2c_fail;

	return 0;

 i2c_fail:
	mcp23s08_spi_exit();
 spi_fail:
	return ret;
}
/* register after spi/i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(mcp23s08_init);

static void __exit mcp23s08_exit(void)
{
	mcp23s08_spi_exit();
	mcp23s08_i2c_exit();
}
module_exit(mcp23s08_exit);

MODULE_LICENSE("GPL");
