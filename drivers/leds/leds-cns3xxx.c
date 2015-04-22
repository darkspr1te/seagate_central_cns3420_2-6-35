/* drivers/leds/leds-cns3xxx.c
 * CNS3XXX - LEDs GPIO driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/timer.h>

#include <asm/mach/irq.h>
#include <asm/uaccess.h>

#include <mach/gpio.h>
#include <mach/cns3xxx.h>

#define GPIO_PROC_NAME          "leds"
#define GREEN_LED               0
#define YELLOW_LED              1
#define BLINK_PERIOD_MS         500

enum {
    LS_OFF,
    LS_SOLID_GREEN,
    LS_SOLID_YELLOW,
    LS_SOLID_GREEN_AND_YELLOW,
    LS_BLINKING_GREEN,
    LS_BLINKING_YELLOW,
    LS_BLINKING_GREEN_AND_YELLOW,
    LS_NO
};

static struct proc_dir_entry *proc_cns3xxx_leds;
static struct timer_list cns3xx_leds_timer;
static char cns3xxx_leds_state_buffer[2];//1 byte + char(10) - new line
static int cns3xxx_leds_state = LS_OFF;
static char cns3xxx_blink_flag = 1;
static DEFINE_SPINLOCK(cns3xx_leds_lock);

static long cns3xx_panic_blink(long count)
{
    long delay = 0;
    static long last_blink;
    static char led;

    if (count - last_blink < 500)
        return 0;

    led ^= 1;
    cns3xxx_gpio_set(0, GREEN_LED, led);
    cns3xxx_gpio_set(0, YELLOW_LED, led);

    last_blink = count;

    return delay;
}

static inline void set_led(unsigned int led)
{
    cns3xxx_gpio_set(0, led, 1);
}

static inline void clear_led(unsigned int led)
{
    cns3xxx_gpio_set(0, led, 0);
}

static inline void start_leds_timer(unsigned int period_ms)
{
    //no need to check the returned value
    mod_timer(&cns3xx_leds_timer, jiffies + msecs_to_jiffies(period_ms));
}

static inline void stop_leds_timer(void)
{
    //no need to check the returned value
    del_timer(&cns3xx_leds_timer);
}

static void cns3xx_leds_timer_callback(unsigned long data)
{
    spin_lock(&cns3xx_leds_lock);

    if (LS_BLINKING_GREEN == cns3xxx_leds_state)
    {
        cns3xxx_blink_flag ^= 1;
        cns3xxx_gpio_set(0, GREEN_LED, cns3xxx_blink_flag);
        start_leds_timer(BLINK_PERIOD_MS);
    }
    else
        if (LS_BLINKING_YELLOW == cns3xxx_leds_state)
        {
            cns3xxx_blink_flag ^= 1;
            cns3xxx_gpio_set(0, YELLOW_LED, cns3xxx_blink_flag);
            start_leds_timer(BLINK_PERIOD_MS);
        }
        else
            if (LS_BLINKING_GREEN_AND_YELLOW == cns3xxx_leds_state)
            {
                cns3xxx_blink_flag ^= 1;
                cns3xxx_gpio_set(0, GREEN_LED, cns3xxx_blink_flag);
                cns3xxx_gpio_set(0, YELLOW_LED, cns3xxx_blink_flag);
                start_leds_timer(BLINK_PERIOD_MS);
            }

    spin_unlock(&cns3xx_leds_lock);
}

static void leds_state_manager(unsigned int new_state)
{
    spin_lock(&cns3xx_leds_lock);

    if (cns3xxx_leds_state == new_state)
    {
        spin_unlock(&cns3xx_leds_lock);
        return;
    }
    cns3xxx_leds_state = new_state;

    //printk(KERN_INFO"current leds state is: %d\n", cns3xxx_leds_state);

    switch(cns3xxx_leds_state)
    {
        case LS_OFF:
            stop_leds_timer( );
            clear_led(GREEN_LED);
            clear_led(YELLOW_LED);
            break;

        case LS_SOLID_GREEN:
            stop_leds_timer( );
            set_led(GREEN_LED);
            clear_led(YELLOW_LED);
            break;

        case LS_SOLID_YELLOW:
            stop_leds_timer( );
            clear_led(GREEN_LED);
            set_led(YELLOW_LED);
            break;

        case LS_SOLID_GREEN_AND_YELLOW:
            stop_leds_timer( );
            set_led(GREEN_LED);
            set_led(YELLOW_LED);
            break;

        case LS_BLINKING_GREEN:
            cns3xxx_blink_flag = 1;
            set_led(GREEN_LED);
            clear_led(YELLOW_LED);
            start_leds_timer(BLINK_PERIOD_MS);
            break;

        case LS_BLINKING_YELLOW:
            cns3xxx_blink_flag = 1;
            clear_led(GREEN_LED);
            set_led(YELLOW_LED);
            start_leds_timer(BLINK_PERIOD_MS);
            break;

        case LS_BLINKING_GREEN_AND_YELLOW:
            cns3xxx_blink_flag = 1;
            set_led(GREEN_LED);
            set_led(YELLOW_LED);
            start_leds_timer(BLINK_PERIOD_MS);
            break;
    }

    spin_unlock(&cns3xx_leds_lock);
}

static int cns3xxx_leds_read_proc(char *page, char **start,  off_t off, int count, int *eof, void *data)
{
    int num = 0;

    spin_lock(&cns3xx_leds_lock);

    num = sprintf(page, "current state: %d\n", cns3xxx_leds_state);
    num += sprintf(page + num, "0 - off\n");
    num += sprintf(page + num, "1 - solid green\n");
    num += sprintf(page + num, "2 - solid yellow\n");
    num += sprintf(page + num, "3 - solid green and yellow\n");
    num += sprintf(page + num, "4 - blink green\n");
    num += sprintf(page + num, "5 - blink yellow\n");
    num += sprintf(page + num, "6 - blink green and yellow\n");

    spin_unlock(&cns3xx_leds_lock);

    return num;
}

int cns3xxx_leds_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
    int tmp = 0;

    if (count > sizeof(cns3xxx_leds_state_buffer) ) {
        printk(KERN_ERR"failed to set leds state: invalid count: %lu - only values between \"%d\" - \"%d\" are accepted\n", count, LS_OFF, LS_NO - 1);
        return -EINVAL;
    }

    if (copy_from_user(cns3xxx_leds_state_buffer, buffer, count)) {
        printk(KERN_ERR"failed to set leds state: copy_from_user failed\n");
        return -EFAULT;
    }

    tmp = cns3xxx_leds_state_buffer[0] - '0';

    if (tmp < LS_OFF || tmp >= LS_NO)
    {
        printk(KERN_ERR"failed to set leds state: accepted values: \"%d\" - \"%d\" but \"%d\" was received\n", LS_OFF, LS_NO - 1, tmp);
        return -EINVAL;
    }

    leds_state_manager(tmp);

    return count;
}

static int __devinit cns3xxx_leds_probe(struct platform_device *pdev)
{
    if (cns3xxx_proc_dir) {
        proc_cns3xxx_leds = create_proc_entry(GPIO_PROC_NAME, S_IFREG | S_IRUGO, cns3xxx_proc_dir);

        if (proc_cns3xxx_leds) {
            proc_cns3xxx_leds->read_proc = cns3xxx_leds_read_proc;
            proc_cns3xxx_leds->write_proc = cns3xxx_leds_write_proc;
        }
    }

    return 0;
}

static struct platform_driver cns3xxx_leds_driver = {
    .probe      = cns3xxx_leds_probe,
    .driver     = {
        .owner  = THIS_MODULE,
        .name   = "cns3xxx_leds",
    },
};

int __init cns3xxx_leds_init(void)
{
    cns3xxx_gpio_direction_out(0, 0, 0);
    cns3xxx_gpio_direction_out(0, 1, 0);

    panic_blink = cns3xx_panic_blink;

    setup_timer(&cns3xx_leds_timer, cns3xx_leds_timer_callback, 0);

    leds_state_manager(LS_BLINKING_GREEN);

    return platform_driver_register(&cns3xxx_leds_driver);
}

void __exit cns3xxx_leds_exit(void)
{
    stop_leds_timer();

    if (proc_cns3xxx_leds)
        remove_proc_entry(GPIO_PROC_NAME, cns3xxx_proc_dir);
    platform_driver_unregister(&cns3xxx_leds_driver);

    panic_blink = NULL;
}

module_init(cns3xxx_leds_init);
module_exit(cns3xxx_leds_exit);

MODULE_DESCRIPTION("CNS3XXX LED driver");
MODULE_LICENSE("GPL");
