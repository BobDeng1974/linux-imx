#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#define MODULE_NAME "powerfailure"
#define REVISION "CSI: $Id: powerfailure.c 2018-07-31 13:56:03  douglas.cogubum " 
#define _DEBUG(fmt, msg...)                                             \
        do {                                                            \
                pr_debug("%s:%s@%d: "                                   \
                         fmt, MODULE_NAME, __func__, __LINE__, ##msg);  \
        } while (0)

#define _ERROR(fmt, msg...)                                     \
        do {                                                    \
                pr_err("CSI: %s ERROR : "                       \
                         fmt, MODULE_NAME, ##msg);              \
        } while (0)

static int power_fail_pin;           /* check this gpio before power off the device */
static int power_off_pin;            /* raising this gpio power offs the device */

static void pf_power_off(void)
{
        printk(KERN_EMERG "Power failure: POWER OFF");
        gpio_direction_input(power_fail_pin);
        
        mdelay(300);
        if(gpio_get_value(power_fail_pin))
            emergency_restart();
                    
        mdelay(300);
        gpio_set_value(power_off_pin, 1);
        mdelay(300);
        WARN_ON(1);
}

static struct task_struct *powerfailure_id = NULL;
static int powerfailure_fn(void *data)
{
        int v;
        int trigger = 0;
        struct platform_device *dev = (struct platform_device *)data;
        int atpfail_gpio = of_get_named_gpio(dev->dev.of_node, "atpfail-gpio", 0);

        if (atpfail_gpio < 0) {
                _ERROR("of_get_named_gpio failed");
                return -1;
        }

        gpio_direction_input(atpfail_gpio);
        while (!kthread_should_stop()) {
                v = gpio_get_value(atpfail_gpio);
                
                if (v == 0)
                        trigger++;
                else
                        trigger = 0;

                if (trigger > 100) {
                        pr_info("Power failure triggered. Starting a clean shutdown");
                        orderly_poweroff(1);
                        return 0;
                }

                msleep(100);
        }
        return 0;
}

static int pf_probe(struct platform_device *dev)
{
        int error;
        int poff_gpio, atpfail_gpio;

        pr_info(REVISION);

        atpfail_gpio = of_get_named_gpio(dev->dev.of_node, "atpfail-gpio", 0);
        if (atpfail_gpio < 0) {
                _ERROR("atpfail-gpio not found in device tree");
                return -ENODEV;
        }

        poff_gpio = of_get_named_gpio(dev->dev.of_node, "poff-gpio", 0);
        if (poff_gpio < 0) {
                _ERROR("poff-gpio not found in device tree");
                return -ENODEV;
        }
        
        error = gpio_request(poff_gpio, "powerfailure power off");
        if (error) {
                _ERROR("Can't request gpio for power failure pin");
                return -ENODEV;
        }
        power_off_pin = poff_gpio;


        error = gpio_request(atpfail_gpio, "powerfailure trigger");
        if (error) {
                _ERROR("gpio_request failed");
                goto err2;
        }
        power_fail_pin = atpfail_gpio;
        
        pm_power_off = pf_power_off; /* setup power-off function */
        
        gpio_direction_output(power_off_pin, 0);
        mdelay(100);

        powerfailure_id = kthread_run(powerfailure_fn, dev, "powerfailure");
        if (IS_ERR(powerfailure_id)) {
                _ERROR("kthread_run failed");
                goto err1;
        }
        
        return 0;
err1:
        gpio_free(atpfail_gpio);
err2:
        gpio_free(poff_gpio);
        return -1;
}

static int pf_remove(struct platform_device *dev)
{
        int error;
        int atpfail_gpio;

        atpfail_gpio = of_get_named_gpio(dev->dev.of_node, "atpfail-gpio", 0);
        BUG_ON(atpfail_gpio < 0); /* since this same call works on pf_probe
                                   * failing here is considered a bug */
        gpio_free(atpfail_gpio);
        gpio_free(power_off_pin);
        gpio_free(power_fail_pin);

        error = kthread_stop(powerfailure_id);
        if (error) {
                _ERROR("kthread_stop fialed");
                return -1;
        }

        return 0;
}

static const struct of_device_id powerfailure_ids[] = {
        { .compatible = MODULE_NAME },
        { },
};
/* MODULE_DEVICE_TABLE(of, powerfailure_ids); */

struct platform_driver pf_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(powerfailure_ids),
        },
        .probe = pf_probe,
        .remove = pf_remove,
};

static int __init pf_init(void)
{
        _DEBUG("");
        return platform_driver_register(&pf_driver);
}


static void __exit pf_exit(void)
{
        _DEBUG("");
        platform_driver_unregister(&pf_driver);
}

module_init(pf_init);
module_exit(pf_exit);
MODULE_AUTHOR("Daniel Hilst daniel.hilst@csi.ind.br>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Power Failure module");
