#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <mach/cns3xxx.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>

#define FAIL_MSG() printk("%s fail\n", __FUNCTION__)


#define __DEBUG__ 0

/* Pin Description */
#define RS   GPIOA(10)
#define RW   GPIOA(12)
#define E    GPIOA(11)
#define UP   GPIOA(22)
#define DOWN GPIOA(23)
#define DB0  GPIOB(4)
#define DB1  GPIOA(13)
#define DB2  GPIOA(14)
#define DB3  GPIOA(15)
#define DB4  GPIOB(25)
#define DB5  GPIOB(24)
#define DB6  GPIOA(20)
#define DB7  GPIOA(21)

#define DB_SHIFT(SHIFT) (1 << (SHIFT))

#define LCD_BUTTONS_MAX 		2

/* In ms */
#define LCD_BUTTON_HOLD_DURATION	1000
#define SAMPLING_INTERVAL		100

/* lcd button events */
#define EV_LCD_UP		1
#define EV_LCD_DOWN		2
#define EV_LCD_UP_HOLD		3
#define EV_LCD_DOWN_HOLD	4

/* ioctl commands */
#define IOC_GET_LCD_BUTTON_EVENT 1
#define IOC_SET_LCD_BACKUP_EVENT 3
#define IOC_GET_LCD_BACKUP_EVENT 4

DEFINE_MUTEX(lcd_button_ev_mutex);
DEFINE_MUTEX(lcd_backup_ev_mutex);

wait_queue_head_t lcd_button_ev_wait;
wait_queue_head_t lcd_backup_ev_wait;


extern void cns3xxx_gpio_set_edgeintr(void (*funcptr)(int), int trig_both, int trig_rising, int gpio_pin);

static void lcd_init(void);
static int lcd_gpio_init(void);
static irqreturn_t thermal_alarm_handler(int i, void *d);
static int st7066_request_gpio_array(void);
static void prepare_data(int data);

/* Instruction and function */
static void write_com(int rs, int rw, int data);
static void set_cgram(int data);
static void set_ddram(int line, int pos);
static void disp_clear(void);
static void cur_disp_shift(int sc, int rl);
static void read_busy_flag_addr(void);
static void print_screen(int data, int pos);
static void ret_home(void);
static void entry_mode_set(int id, int s);
static void disp(int d, int c, int b);
static void func_set(int dl, int n, int f);
static void write_data(int data);
static void read_data(int *data);

/* proc */
extern struct proc_dir_entry *cns3xxx_proc_dir;
static struct proc_dir_entry *proc_st7066;
static int st7066_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data);
static int st7066_read_proc(char *page, char **start,  off_t off, int count, int *eof, void *data);

static ssize_t readstring(char *buff, const char *buf, size_t count);

static struct gpio lcd_gpios[] = {
	{RS,   GPIOF_OUT_INIT_LOW, "RS"},
	{RW,   GPIOF_OUT_INIT_LOW, "RW"},
	{E,    GPIOF_OUT_INIT_LOW, "E"},
	{UP,   GPIOF_IN, "Up button"},
	{DOWN, GPIOF_IN, "Down button"},
	{DB0,  GPIOF_OUT_INIT_LOW, "Data Bus 0"},
	{DB1,  GPIOF_OUT_INIT_LOW, "Data Bus 1"},
	{DB2,  GPIOF_OUT_INIT_LOW, "Data Bus 2"},
	{DB3,  GPIOF_OUT_INIT_LOW, "Data Bus 3"},
	{DB4,  GPIOF_OUT_INIT_LOW, "Data Bus 4"},
	{DB5,  GPIOF_OUT_INIT_LOW, "Data Bus 5"},
	{DB6,  GPIOF_OUT_INIT_LOW, "Data Bus 6"},
	{DB7,  GPIOF_OUT_INIT_LOW, "Data Bus 7"}
};

/* Predefined character, use set_cgram and write_data to define new characters. */
static int cgram[7][8] = {
	{0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f}, /* all black */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}, /* vertical line */
	{0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa}, /* snow */
	{0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f}, /* horizontal line */
	{0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f}, /* rectangle */
	{0x04, 0x1f, 0x15, 0x1f, 0x15, 0x1f, 0x04, 0x04}  /* 申 */
};

static int cur_pos=0x0;
static int lcd_button_event = 0;
static int backup_event = 0;

struct lcd_button_control
{
	unsigned gpio;
	int timer_count;
	int hold_timer_count;
	bool processed;
};

struct lcd_button_data
{
	struct lcd_button_control control[ LCD_BUTTONS_MAX ];
	struct timer_list timer;
        struct work_struct work;
};


static struct lcd_button_data *lcd_bdata;

static int st7066_request_gpio_array(void)
{
	if (gpio_request_array(lcd_gpios, ARRAY_SIZE(lcd_gpios))) {
		gpio_free_array(lcd_gpios, ARRAY_SIZE(lcd_gpios));
		FAIL_MSG();
		return -1;
	} else
		return 0;
}

/*
 * Open lcd-control device
 */
int lcdctrl_open(struct inode *inode, struct file *file)
{
        return 0;
}

/*
 * Close lcd-control device
 */
int lcdctrl_release(struct inode *inode, struct file *file)
{
        return 0;
}

/*
 * ioctls to get lcd-control / lcd-button events
 */
int lcdctrl_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, unsigned long arg)
{
        int ret = 0;
	void __user *argp = (void __user *)arg;
			
        switch(cmd){
		case IOC_GET_LCD_BUTTON_EVENT:

			interruptible_sleep_on(&lcd_button_ev_wait);

                        mutex_lock(&lcd_button_ev_mutex);
			copy_to_user(argp, &lcd_button_event, sizeof(lcd_button_event));
                        lcd_button_event = 0;
                        mutex_unlock(&lcd_button_ev_mutex);
                break;
		
		case IOC_SET_LCD_BACKUP_EVENT:

			mutex_lock(&lcd_backup_ev_mutex);
			copy_from_user(&backup_event, argp, sizeof(backup_event));
                        mutex_unlock(&lcd_backup_ev_mutex);
			
			wake_up_interruptible(&lcd_backup_ev_wait);
		break;
		
		case IOC_GET_LCD_BACKUP_EVENT:

			interruptible_sleep_on(&lcd_backup_ev_wait);

                        mutex_lock(&lcd_backup_ev_mutex);
			copy_to_user(argp, &backup_event, sizeof(backup_event));
			backup_event = 0;	
                        mutex_unlock(&lcd_backup_ev_mutex);
		break;
	
                default:
                        ret = -EIO;
                break;
        }

        return ret;
}

static const struct file_operations lcdctrl_fops = {
        .owner          = THIS_MODULE,
        .open           = lcdctrl_open,
        .ioctl          = lcdctrl_ioctl,
        .release        = lcdctrl_release,
};

static struct miscdevice my_lcdctrl_dev = {
	.minor = MISC_DYNAMIC_MINOR, /* defined as 130 in include/linux/miscdevice.h*/
	.name  = "lcdctrl",
        .fops  = &lcdctrl_fops
};


static void prepare_data(int data)
{
	gpio_direction_output(DB0, DB_SHIFT(0)&data);
	gpio_direction_output(DB1, DB_SHIFT(1)&data);
	gpio_direction_output(DB2, DB_SHIFT(2)&data);
	gpio_direction_output(DB3, DB_SHIFT(3)&data);
	gpio_direction_output(DB4, DB_SHIFT(4)&data);
	gpio_direction_output(DB5, DB_SHIFT(5)&data);
	gpio_direction_output(DB6, DB_SHIFT(6)&data);
	gpio_direction_output(DB7, DB_SHIFT(7)&data);
}

static void write_com(int rs, int rw, int data)
{
	read_busy_flag_addr();
	gpio_direction_output(RS, rs);
	gpio_direction_output(RW, rw);
	gpio_direction_output(E, 1);

	prepare_data(data);

	udelay(20);
	gpio_direction_output(E, 0);
}

/* Set CGRAM Address 
 * 
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   1   AC5 AC4 AC3 AC2 AC1 AC0
 */
static void set_cgram(int data)
{
	write_com(0, 0, 0x40|data);
}

/* Set DDRAM Address 
 * 
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  1   AC6 AC5 AC4 AC3 AC2 AC1 AC0
 *
 */
static void set_ddram(int line, int pos)
{
	write_com(0, 0, 0x80 | line << 6 | pos);
}

/* Clear Display
 *
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   0   0   0   0   0   0   1
 */
static void disp_clear(void)
{
	write_com(0, 0, 0x1);
	cur_pos = 0;
}

/* Cursor or Display Shift
 *
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   0   0   1   S/C R/L x   x
 *
 * S/C   R/L   Description                      AC Value
 *  L     L    Shift cursor to left             AC = AC + 1
 *  L     H    Shift cursor to right            AC = AC - 1
 *  H     L    Shfit display to left.           AC = AC
 *             Cursor follow the display shift
 *  H     H    Shfit display to right.          AC = AC
 *             Cursor follow the display shift
 */
static void cur_disp_shift(int sc, int rl)
{
	write_com(0, 0, 0x10 | sc << 3 | rl << 2);
}

/* Read Busy Flag and Address 
 * 
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 1  0  D7  D6  D5  D4  D3  D2  D1  D0
 */
static void read_busy_flag_addr(void)
{
	mdelay(10);

	gpio_direction_output(DB7, 0);
	gpio_direction_output(RS, 0);
	gpio_direction_output(RW, 1);
	gpio_direction_output(E, 1);
	gpio_direction_output(E, 0);
}

/* Print something on screen
 *
 * pos = 0 ~ 31
 */
static void print_screen(int data, int pos)	
{
	if (pos < 16)
		set_ddram(0x0, pos);
	else /* change line */
		set_ddram(0x1, (pos-16));
	udelay(50);
//	write_com(1, 0, data);
	write_data(data);
}

/* Retrun Home
 *
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   0   0   0   0   0   1   x
 */
static void ret_home(void)
{
	write_com(0, 0, 0x02);
	cur_pos = 0;
}

/* Entry Mode Set
 *
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   0   0   0   0   1   I/D S
 *
 * I/D = high, cursor/blink moves to right and 
 *             DDRAM addr is increased by 1.
 *     =  low, cursor/blink moves to left and
 *             DDRAM addr is decreased by 1.
 */
static void entry_mode_set(int id, int s)
{
	write_com(0, 0, 0x4 | id << 1 | s);
}

/* Display ON / OFF
 *
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   0   0   0   1   D   C   B
 * 
 * D = high, entire display turn on
 *   =  low, display is turned off, but data is remained in DDRAM
 * C = high, cursor is turned on
 *   =  low, cursor is disappeared in current display, 
 *      but I/D register remains its data
 * B = high, cursor blink is on, that performs alternate between all
 *           the high data and display character at the cursor position.
 *   =  low, blink is off.
 */
static void disp(int d, int c, int b)
{
	write_com(0, 0, 0x8 | d << 2 | c << 1 | b);
}

/* RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0  0  0   0   1   DL  N   F   x   x
 * 
 * DL = high, it means 8-bit bus mode with MPU.
 *    =  low, it means 4-bit bus mode with MPU. So to speak,
 *       DL is a signal to select 8-bit or 4-bit bus mode.
 * N  = high, 2-lines display mode is set.
 *    =  low, it means 1-line display mode.
 * F  = high, 5x10 dots format display mode.
 *    =  low, it means 5x8 dots format display mode.
 */
static void func_set(int dl, int n, int f)
{
	write_com(0, 0, 0x20 | dl << 4 | n << 3 | f << 2);
}

/* Write Data 
 * 
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 1  0  D7  D6  D5  D4  D3  D2  D1  D0
 */
static void write_data(int data)
{
	write_com(1, 0, data);
}

/* Read binary 8-bit data from CGRAM/DDRAM 
 * 
 * RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 1  1  D7  D6  D5  D4  D3  D2  D1  D0
 */
static void read_data(int *data)
{
	write_com(1, 1, *data);
}

static ssize_t readstring(char *buff, const char *buf, size_t count)
{
	int i=0;
    if (count) {
		char c;

		for(i=0;i<count&&i<50;i++) {
			if (get_user(c, buf+i))
				return -EFAULT;
			buff[i] = c;
		}
	}
	return count;
}

static int st7066_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int i, j, new_cgram[8], cgram_addr;
	char read_buff[50]={'\0'}, buf_cmd[50]={'\0'}, buf_str[32]={'\0'};

	if (count > 50) {
		printk("%s: count = %lu, too many character.\n", __FUNCTION__, count);
		return count;
	}

	readstring((char *)read_buff, (const char *)buffer, count-1);

	sscanf(read_buff,"%s ",(char *)&buf_cmd);

	if (strcmp(buf_cmd, "off") == 0) {
		disp(0, 0, 0); /* DISPLAY OFF ,00001DCB ,D=1:Display on; 0:off */
	} else if (strcmp(buf_cmd, "on") == 0 || strcmp(buf_cmd, "bon") == 0) {
		disp(1, 1, 1); /* DISPLAY ON ,00001DCB ,D=1:Display on; 0:off */
	} else if (strcmp(buf_cmd, "scleft") == 0) {
		cur_disp_shift(0, 0);
	} else if (strcmp(buf_cmd, "scright") == 0) {
		cur_disp_shift(0, 1);
	} else if (strcmp(buf_cmd, "sdleft") == 0) {
		cur_disp_shift(1, 0);
	} else if (strcmp(buf_cmd, "sdright") == 0) {
		cur_disp_shift(1, 1);
	} else if (strcmp(buf_cmd, "home") == 0) {
		ret_home(); /* RETURN HOME */
	} else if (strcmp(buf_cmd, "boff") == 0) {
		disp(1, 0, 0);
	} else if (strcmp(buf_cmd, "clear") == 0) {
		disp_clear();
		mdelay(5);
	} else if (strcmp(buf_cmd, "print") == 0) {
		for (i=6, j=0; i<strlen(read_buff); i++, j++)
			buf_str[j] = read_buff[i];

		if (cur_pos > 32)
			cur_pos = 0;

		for (i=0; i<strlen(buf_str); i++) {
			print_screen((int)buf_str[i], cur_pos);
			cur_pos += 0x1;
		}
	} else if (strcmp(buf_cmd, "move") == 0) {
		sscanf(read_buff,"%s %d",(char *)&buf_cmd, &cur_pos);
		if (cur_pos < 16)
			set_ddram(0x0, cur_pos);
		else /* change line */
			set_ddram(0x1, (cur_pos-16));
	} else if (strcmp(buf_cmd, "set_cgram") == 0) {
		sscanf(read_buff,"%s %d %x %x %x %x %x %x %x %x",(char *)&buf_cmd, &cgram_addr,
				&new_cgram[0], &new_cgram[1], &new_cgram[2], &new_cgram[3],
				&new_cgram[4], &new_cgram[5], &new_cgram[6], &new_cgram[7]);
		set_cgram(cgram_addr);
		for (i=0; i<8; i++) {
			printk("%x ", new_cgram[i]);
			write_data(new_cgram[i]);
		}
		printk("\n");
	} else if (strcmp(buf_cmd, "print_cgram") == 0) {
		sscanf(read_buff,"%s %d",(char *)&buf_cmd, &i);
		print_screen(i, cur_pos);
	} else if (strcmp(buf_cmd, "all_black") == 0) {
		set_cgram(0x00);
		for (i=0; i<8; i++)
			write_data(cgram[0][i]);
		
		for (cur_pos=0; cur_pos<32; cur_pos++)
			print_screen(0x00, cur_pos);
	}

	return count;
}

static int st7066_read_proc(char *page, char **start,  off_t off, int count, int *eof, void *data)
{
	int num = 0;

	return num;
}


void report_button_event( int button, int status )
{
	if(lcd_bdata->control[button].timer_count >= lcd_bdata->control[button].hold_timer_count){
#if(__DEBUG__)
		printk(KERN_INFO "%s: button press and hold\n", (button ? "down":"up")); /* pressing longer */
#endif
		lcd_bdata->control[button].processed = 1;				
		mutex_lock(&lcd_button_ev_mutex);
		if(!button)
			lcd_button_event = EV_LCD_UP_HOLD;
		else
			lcd_button_event = EV_LCD_DOWN_HOLD;
		mutex_unlock(&lcd_button_ev_mutex);
		wake_up_interruptible(&lcd_button_ev_wait);
	}else if((lcd_bdata->control[button].timer_count >= 1) && (!status)){
#if(__DEBUG__)
		printk(KERN_INFO "%s: button press and release\n", (button ? "down":"up")); /* pressed and released */
#endif
		lcd_bdata->control[button].processed = 1;
		mutex_lock(&lcd_button_ev_mutex);
		if(!button)
			lcd_button_event = EV_LCD_UP;
		else
			lcd_button_event = EV_LCD_DOWN;
		mutex_unlock(&lcd_button_ev_mutex);
		wake_up_interruptible(&lcd_button_ev_wait);
	}
}

void lcd_button_work_func(struct work_struct *work)
{
	int ret;
	int i, irq, status, pressed = 0;
	
	for(i=0; i<LCD_BUTTONS_MAX; i++){
	
		status = (gpio_get_value(lcd_bdata->control[i].gpio) ? 1 : 0) ^ 1;
		
		if(status){
			lcd_bdata->control[i].timer_count++;
		}

                if(!(lcd_bdata->control[i].processed)){
	                report_button_event(i, status);
                }

		pressed |= status;
		status = 0;
	}
	
	/* If any button, still remain pressed, continue the polling */
	if(pressed){
		ret = mod_timer(&lcd_bdata->timer, jiffies + msecs_to_jiffies(SAMPLING_INTERVAL));
        	if(ret){
        	        printk(KERN_ERR "lcd_button_work: mod timer error \n");
        	}
	} /* re-enable irqs */
	else{
		for(i=0; i<LCD_BUTTONS_MAX; i++){
			lcd_bdata->control[i].timer_count = 0;
			lcd_bdata->control[i].processed = 0;
			irq = gpio_to_irq(lcd_bdata->control[i].gpio);
			enable_irq(irq);
		}
	}
			 
}


void lcd_button_timer(unsigned long _data)
{
	/* schedule work */
	schedule_work(&lcd_bdata->work);
}


static irqreturn_t lcd_button_gpio_intr_handler(int i, void *d)
{
        int irq;
        int ret;
#if(__DEBUG__)
        printk(KERN_INFO "up/down\n");
#endif

        ret = mod_timer(&lcd_bdata->timer, jiffies + msecs_to_jiffies(SAMPLING_INTERVAL));
        if(ret){
                printk(KERN_ERR "button_gpio_intr_handler: mod timer error \n");
        }
        else{
		/* disable the irqs */
		for(i=0; i<LCD_BUTTONS_MAX; i++){
                        irq = gpio_to_irq(lcd_bdata->control[i].gpio);
               		disable_irq_nosync(irq);
                }

        }

        return IRQ_HANDLED;
}

/* Buttons */
#define ALARM               GPIOB(3)

static irqreturn_t thermal_alarm_handler(int i, void *d)
{
	if (gpio_get_value(ALARM))
		printk("Over temperature alarm released!!!\n");
	else
		printk("Over temperature alarm detected!!!\n");

	return IRQ_HANDLED;
}

static int lcd_gpio_init(void)
{
	int i;
	
	/* Enable GPIOA 10~15 and 20~23 */
	MISC_GPIOA_PIN_ENABLE_REG &= ~((0x1 << 10) |
                                   (0x1 << 11) |
                                   (0x1 << 12) |
                                   (0x1 << 13) |
                                   (0x1 << 14) |
                                   (0x1 << 15) |
                                   (0x1 << 20) |
                                   (0x1 << 21) |
                                   (0x1 << 22) |
                                   (0x1 << 23));

	/* Enable GPIOB 4, 24 and 25 */
	MISC_GPIOB_PIN_ENABLE_REG &= ~((0x1 <<  4) |
                                   (0x1 << 24) |
                                   (0x1 << 25));

	lcd_bdata = kzalloc(sizeof(struct lcd_button_data),GFP_KERNEL);

	if(!lcd_bdata){
		printk(KERN_ERR "lcd_bdata allocation failed.\n");
		return - 1;
	}


	lcd_bdata->control[0].gpio = UP;
	lcd_bdata->control[1].gpio = DOWN;

	for(i=0; i<LCD_BUTTONS_MAX; i++)
	{
		lcd_bdata->control[i].timer_count = 0;
		lcd_bdata->control[i].hold_timer_count = LCD_BUTTON_HOLD_DURATION / SAMPLING_INTERVAL;
		lcd_bdata->control[i].processed = 0;		
	}

	setup_timer(&lcd_bdata->timer, lcd_button_timer, 0);
	INIT_WORK(&lcd_bdata->work, lcd_button_work_func);

	init_waitqueue_head(&lcd_button_ev_wait);
	init_waitqueue_head(&lcd_backup_ev_wait);


	/* Request GPIO */
	if (st7066_request_gpio_array()) {
		FAIL_MSG();
		return -1;
	}

	/* trig at falling edge */
	/* Thermal alarm low avtive */
	gpio_request(ALARM, "thermal_alarm");
	gpio_direction_input(ALARM);
	request_irq(gpio_to_irq(ALARM), thermal_alarm_handler, IRQF_SHARED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "thermal_alarm", lcd_gpios);
	request_irq(gpio_to_irq(UP), lcd_button_gpio_intr_handler, IRQF_SHARED | IRQF_TRIGGER_FALLING, "lcd_up", lcd_gpios);
	request_irq(gpio_to_irq(DOWN), lcd_button_gpio_intr_handler, IRQF_SHARED | IRQF_TRIGGER_FALLING, "lcd_down", lcd_gpios);

	return 0;
}

static void lcd_init(void)
{
	/* Initialize */
	func_set(1, 1, 0);
	mdelay(10);
	entry_mode_set(1, 0);
	udelay(50);
	ret_home();
	udelay(50);
	disp(1, 1, 1);
	udelay(50);
	disp_clear(); /* CLR DISPLAY */
	mdelay(5);


	/* Create proc file */
	proc_st7066 = create_proc_entry("st7066", S_IFREG | S_IRUGO, cns3xxx_proc_dir) ;
	proc_st7066->read_proc = st7066_read_proc;
	proc_st7066->write_proc = st7066_write_proc;
}

static int __init st7066_init(void)
{
	int result;

        /* Registering device */
	result = misc_register(&my_lcdctrl_dev);

        if (result < 0) {
                printk(KERN_ERR "lcdctrl: misc_register failed\n");
                return result;
        }

	if (lcd_gpio_init())
		goto fail;

	lcd_init();

	return 0;

fail:
	FAIL_MSG();
	return -1;
}

module_init(st7066_init);

static void __exit st7066_exit(void)
{
	printk("%s: free gpios...\n", __func__);
	del_timer_sync(&lcd_bdata->timer);
	cancel_work_sync(&lcd_bdata->work);
	kfree(lcd_bdata);
	gpio_free_array(lcd_gpios, ARRAY_SIZE(lcd_gpios));
	free_irq(gpio_to_irq(UP), lcd_gpios);
	free_irq(gpio_to_irq(DOWN), lcd_gpios);
	free_irq(gpio_to_irq(ALARM), lcd_gpios);
	gpio_free(ALARM);
	remove_proc_entry("st7066", cns3xxx_proc_dir);
	misc_deregister(&my_lcdctrl_dev);

}

module_exit(st7066_exit);

MODULE_DESCRIPTION("ST7066 LCM driver");
MODULE_LICENSE("GPL");

