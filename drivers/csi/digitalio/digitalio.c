#include <asm/uaccess.h> 

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcp23s08.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/digitalio.h>
#include <linux/ioctl.h>

#define FIRST_INPUT        240
#define FIRST_OUTPUT       248
#define NUM_GPIO            12
#define FIRSTMIN             0
#define DIRECTION_INPUT      0
#define DIRECTION_OUTPUT     1
#define MODULE_NAME "digitalio"
#define REVISION "$Id: digitalio.c 15634 2015-12-02 20:13:06Z daniel.hilst $"
#define LOG_msg(pri, fmt, args...) \
        ({pr_ ## pri("digitalio [" #pri "] %s@%d: " fmt "\n", __func__, __LINE__, ##args);})
#ifdef  DEBUG
# define LOG_debug(fmt, args...) LOG_msg(debug, fmt, ##args)
#else
# define LOG_debug(fmt, args...)
#endif
#define LOG_info(fmt, args...)  LOG_msg(info,  fmt, ##args)
#define LOG_warn(fmt, args...)  LOG_msg(warn,  fmt, ##args)
#define LOG_err(fmt, args...)   LOG_msg(err,   fmt, ##args)

struct digitalio {
        dev_t devt;
        struct cdev *cdev;
        struct class *class;
};
static struct digitalio digitalio;

struct digitalio_private_data {
        int direction;
        int gpio;
};

static struct gpio input_gpio[] = {
        { FIRST_INPUT, GPIOF_DIR_IN, "digitalinput0" },
        { FIRST_INPUT + 1, GPIOF_DIR_IN, "digitalinput1" },
        { FIRST_INPUT + 2, GPIOF_DIR_IN, "digitalinput2" },
        { FIRST_INPUT + 3, GPIOF_DIR_IN, "digitalinput3" },
        { FIRST_INPUT + 4, GPIOF_DIR_IN, "digitalinput4" },
        { FIRST_INPUT + 5, GPIOF_DIR_IN, "digitalinput5" },
};

static struct gpio output_gpio[] = {
        { FIRST_OUTPUT, GPIOF_OUT_INIT_HIGH, "digitaloutput0" },
        { FIRST_OUTPUT + 1, GPIOF_OUT_INIT_HIGH, "digitaloutput1" },
        { FIRST_OUTPUT + 2, GPIOF_OUT_INIT_HIGH, "digitaloutput2" },
        { FIRST_OUTPUT + 3, GPIOF_OUT_INIT_HIGH, "digitaloutput3" },
        { FIRST_OUTPUT + 4, GPIOF_OUT_INIT_HIGH, "digitaloutput4" },
        { FIRST_OUTPUT + 5, GPIOF_OUT_INIT_HIGH, "digitaloutput5" },
};

/* Module parametes ------------------------------------------------------- */
static int turnoff_on_release = 0;
module_param(turnoff_on_release, int, 0600);
MODULE_PARM_DESC(turnoff_on_release, "Turn gpios OFF while realeasing /dev nodes");


/* File operations -------------------------------------------------------- */
static int digitalio_open(struct inode *ip, struct file *fp)
{
        struct digitalio_private_data *pdata;

        /* Two process opening the same node at same time is forbidden */
        if (fp->private_data)
                return -EBUSY;

        pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
                LOG_err("can't allocate private data while opening file, out of memory?");
                return -ENOMEM;
        }

        /* I use the minor number to determine if is input or
         * output. Input goes from 0 to (NUM_GPIO/2)-1 output go
         * from NUM_GPIO/2 to NUM_GPIO-1 
         */
        if (MINOR(ip->i_rdev) < (NUM_GPIO / 2)) {
                pdata->direction = DIRECTION_INPUT;
                pdata->gpio      = FIRST_INPUT + MINOR(ip->i_rdev);
                LOG_debug("Opening input gpio %d", pdata->gpio);
        } else {
                pdata->direction = DIRECTION_OUTPUT;
                pdata->gpio      = FIRST_OUTPUT + (MINOR(ip->i_rdev) - (NUM_GPIO / 2));
                LOG_debug("Opening output gpio %d", pdata->gpio);
        }

        if (pdata->gpio == FIRST_INPUT)
                mcp_flush_queue(); /* flush the gpio-mcp23s08 queue  */

        fp->private_data = pdata;
        return 0;
}

static int digitalio_release(struct inode *ip, struct file *fp)
{
        struct digitalio_private_data *pdata = fp->private_data;

        if (turnoff_on_release &&
            (pdata->direction == DIRECTION_OUTPUT))
                gpio_set_value_cansleep(pdata->gpio, 1);

        kfree(pdata);
        fp->private_data = NULL;
        return 0;
}

static ssize_t digitalio_write(struct file *fp,
                               const char __user *buf,
                               size_t len,
                               loff_t *off)
{
        unsigned value;
        struct digitalio_private_data *pdata = fp->private_data;

        LOG_debug("len(%d), off(%lld)", len, *off);

        if (pdata->direction == DIRECTION_INPUT)
                return -ENOSYS;

        get_user(value, buf);
        gpio_set_value_cansleep(pdata->gpio, !value); /* invert value */
        return 1;
}

static ssize_t digitalio_read(struct file *fp,
                        char __user *buf,
                        size_t len,
                        loff_t *off)
{
        char value;
        struct digitalio_private_data *pdata = fp->private_data;

        LOG_debug("len(%d), off(%lld)", len, *off);
        value = gpio_get_value_cansleep(pdata->gpio);
        put_user(!value, buf);
        return 1;
}

static long digitalio_ioctl (struct file *fp, unsigned int cmd, unsigned long arg)
{
        long status = -EIO;
        struct digitalio_private_data *pdata = fp->private_data;
        struct digitalio_waitforinputchange waitfi;
        unsigned long timeout;
        int mask;

        switch (cmd) {
        case DIGITALIO_WAITFORINPUTMASK:
                if (pdata->direction != DIRECTION_INPUT)
                        return -EINVAL;

                if (copy_from_user(&waitfi, (void *)arg, sizeof(waitfi)))
                        return -EIO;

                if (waitfi.mask == 0)
                        return -EINVAL;

                timeout = waitfi.timeout * HZ / 1000;

                status = mcp_wait_irq(timeout, waitfi.mask, &waitfi.intcap);
                if (status)       /* -ETIME or -ERESTARTSYS */
                        goto out; /* timedout */

                if (copy_to_user((void *)arg, &waitfi, sizeof(waitfi)))
                        return -EIO;

                break;
        case DIGITALIO_GETINPUTMASK:
                if (pdata->direction != DIRECTION_INPUT)
                        return -EINVAL;

                mask = mcp_getmask();
                if (mask < 0)
                        return -EIO;

                put_user(mask, (int *)arg);
                status = 0;     /* return value */
                break;
        case DIGITALIO_GETLASTINPUT:
                LOG_debug("ioctl: DIGITALIO_GETLASTINPUT");
                if (pdata->direction != DIRECTION_INPUT)
                        return -EINVAL;

                if (copy_from_user(&waitfi, (void *)arg, sizeof(waitfi)))
                        return -EIO;

                LOG_debug("timeout = %u", waitfi.timeout);
                status  = mcp_getlast(&waitfi.intcap, waitfi.timeout); /* get mask from mcp queue */
                if (status)     /* -ETIME or -ERESTARTSYS */
                        goto out;

                if (copy_to_user((void *)arg, &waitfi, sizeof(waitfi)))
                        return -EIO;

                break;
        default:
                return -ENOTTY;
        }

out:
        return status;
}

struct file_operations fops = {
        .owner          = THIS_MODULE,
        .read           = digitalio_read,
        .write          = digitalio_write,
        .open           = digitalio_open,
        .release        = digitalio_release,
        .unlocked_ioctl = digitalio_ioctl,

};

/* Init/Exit -------------------------------------------------------------- */

static int __init digitalio_init(void)
{
        int status;
        int i = 0;

        LOG_info(REVISION);

        /* request the gpios */
        status = gpio_request_array(input_gpio, ARRAY_SIZE(input_gpio));
        if (status) {
                LOG_err("can't request input GPIOs");
                goto fail1;
        }

        status = gpio_request_array(output_gpio, ARRAY_SIZE(output_gpio));
        if (status) {
                LOG_err("can't request output GPIOs");
                goto fail2;
        }

        /* create character devices */
        digitalio.class = class_create(THIS_MODULE, MODULE_NAME);
        if (IS_ERR(digitalio.class)) {
                LOG_err("can't create class %s", MODULE_NAME);
                goto fail3;
        }

        status = alloc_chrdev_region(&digitalio.devt, FIRSTMIN, NUM_GPIO, MODULE_NAME);
        if (status) {
                LOG_err("can't allocate chrdev region");
                goto fail4;
        }

        digitalio.cdev = cdev_alloc();
        if (!digitalio.cdev) {
                LOG_err("can't allocate cdev");
                goto fail5;
        }

        cdev_init(digitalio.cdev, &fops);

        status = cdev_add(digitalio.cdev, digitalio.devt, NUM_GPIO);
        if (status < 0) {
                LOG_err("can't add character device");
                goto fail6;
        }

        /* create device nodes */
        for (i = 0; i < NUM_GPIO; i++) {
                dev_t pair = MKDEV(MAJOR(digitalio.devt), MINOR(i + FIRSTMIN));
                struct device *devp;

                if (i < (NUM_GPIO / 2)) {
                        devp = device_create(digitalio.class, NULL, pair, NULL, "digitalinput%d", i);
                        if (IS_ERR(devp)) {
                                LOG_err("can't create device input%d", i);
                                goto fail7;
                        }
                } else {
                        devp = device_create(digitalio.class, NULL, pair, NULL, "digitaloutput%d", i - (NUM_GPIO / 2));
                        if (IS_ERR(devp)) {
                                LOG_err("can't create device output%d", i);
                                goto fail7;
                        }
                }
        }
        
        return 0;
fail7:
        while (i--)
                device_destroy(digitalio.class, MKDEV(MAJOR(digitalio.devt), MINOR(i + FIRSTMIN)));
fail6:
        cdev_del(digitalio.cdev);
fail5:
        unregister_chrdev_region(digitalio.devt, NUM_GPIO);
fail4:
        class_unregister(digitalio.class);
        class_destroy(digitalio.class);
fail3:
        gpio_free_array(output_gpio, ARRAY_SIZE(output_gpio));
fail2:
        gpio_free_array(input_gpio, ARRAY_SIZE(input_gpio));
fail1:
        return status;
}

static void __exit digitalio_exit(void)
{
        int i;

        for (i = 0; i < NUM_GPIO; i++)
                device_destroy(digitalio.class, MKDEV(MAJOR(digitalio.devt), MINOR(i + FIRSTMIN)));

        cdev_del(digitalio.cdev);
        unregister_chrdev_region(digitalio.devt, NUM_GPIO);
        class_unregister(digitalio.class);
        class_destroy(digitalio.class);
        gpio_free_array(output_gpio, ARRAY_SIZE(output_gpio));
        gpio_free_array(input_gpio, ARRAY_SIZE(input_gpio));
}

module_init(digitalio_init);
module_exit(digitalio_exit);
MODULE_LICENSE("GPL");

