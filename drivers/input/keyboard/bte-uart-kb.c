#include <linux/fs.h>
#include <linux/syscalls.h>	
#include <linux/ioctl.h>

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <asm/termios.h>
#include <asm/ioctls.h>
#include <linux/serial.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/keyboard.h>
//starwars
#if defined(GAME_STARWARS) || defined(GAME_OUTRUN)
#include <linux/cdev.h>
#include <linux/device.h>
#endif
//a40i_vol_proc
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/rt.h>
static struct input_dev *bte_dev = NULL;
#define BTE_UART_KB_TTY "/dev/ttyS1"
static struct tty_struct *local_tty = NULL;
struct file *bte_fp = NULL;
static struct task_struct *bte_tsk = NULL;
#define BTE_UART_KB_TTY2 "/dev/ttyS3"
static struct tty_struct *local_tty2 = NULL;
struct file *bte_fp2 = NULL;
struct input_dev* input_js = NULL;

u8 bte_timeout = 0;

u8 key_val = 0;
u8 key_val_org = 0;
u8 proc_flag = 1;
int sign_unlock;
//gametype
int bte_gametype = -1;

u8 bte_loop=0;
u8 bte_jsx = 0x80, bte_jsy = 0x80;

static int major = -1;
static struct cdev mycdev;
static struct class *myclass = NULL;
static struct mutex outrun_mutex;
static u8 mode_flag = 0;
static u8 get_data_once = 0;

struct VR_DATA {
	u8 vr_wheel;
	u8 vr_break;
	u8 vr_gas;
	s8 vr_gear;
};

struct CAL_DATA {
	u8 wheel_low;
	u8 wheel_mid_low;
	u8 wheel_mid_high;
	u8 wheel_high;
	u8 gas_low;
	u8 gas_high;
	u8 break_low;
	u8 break_high;
};

struct ADC_DATA {
	u8 wheel_data;
	u8 gas_data;
	u8 break_data;
	u8 adc_wheelh;
	u8 adc_wheell;
	u8 adc_gash;
	u8 adc_gasl;
	u8 adc_breakh;
	u8 adc_breakl;
};

struct VR_DATA vrdata; 
struct CAL_DATA caldata;
struct ADC_DATA adcdata;

u8 cal_wheel1=0, cal_wheel2=0, cal_wheel3=0, cal_wheel4=0;
u8 cal_gas1=0, cal_gas2=0;
u8 cal_break1=0, cal_break2=0;
s8 erase_data_type = -1;
s8 write_data_type = -1;
uint8_t bteWrite[17] = {0xA6, 0x0F, 0xF5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x10};

static u8 p1_start = 0;
static u8 p2_start = 0;
static u8 p1_a = 0;
static u8 p1_b = 0;
static u8 p1_x = 0;
static u8 p1_xl = 0;
static u8 p1_xr = 0;
static u8 p2_kp2 = 0;
static u8 testmode_bootup = 0;
static int g_power_off = 0;
static s16 wheel[256] = 
{
-32768, -32512, -32256, -32000, -31744, -31488, -31232, -30976,
-30720, -30464, -30208, -29952, -29696, -29440, -29184, -28928,
-28672, -28416, -28160, -27904, -27648, -27392, -27136, -26880,
-26624, -26368, -26112, -25856, -25600, -25344, -25088, -24832,
-24576, -24320, -24064, -23808, -23552, -23296, -23040, -22784,
-22528, -22272, -22016, -21760, -21504, -21248, -20992, -20736,
-20480, -20224, -19968, -19712, -19456, -19200, -18944, -18688,
-18432, -18176, -17920, -17664, -17408, -17152, -16896, -16640,
-16384, -16128, -15872, -15616, -15360, -15104, -14848, -14592,
-14336, -14080, -13824, -13568, -13312, -13056, -12800, -12544,
-12288, -12032, -11776, -11520, -11264, -11008, -10752, -10496,
-10240, -9984, -9728, -9472, -9216, -8960, -8704, -8448,
-8192, -7936, -7680, -7424, -7168, -6912, -6656, -6400,
-6144, -5888, -5632, -5376, -5120, -4864, -4608, -4352,
-4096, -3840, -3584, -3328, -3072, -2816, -2560, -2304,
-2048, -1792, -1536, -1280, -1024, -768, -512, -256,
0, 256, 512, 768, 1024, 1280, 1536, 1792,
2048, 2304, 2560, 2816, 3072, 3328, 3584, 3840,
4096, 4352, 4608, 4864, 5120, 5376, 5632, 5888,
6144, 6400, 6656, 6912, 7168, 7424, 7680, 7936,
8192, 8448, 8704, 8960, 9216, 9472, 9728, 9984,
10240, 10496, 10752, 11008, 11264, 11520, 11776, 12032,
12288, 12544, 12800, 13056, 13312, 13568, 13824, 14080,
14336, 14592, 14848, 15104, 15360, 15616, 15872, 16128,
16384, 16640, 16896, 17152, 17408, 17664, 17920, 18176,
18432, 18688, 18944, 19200, 19456, 19712, 19968, 20224,
20480, 20736, 20992, 21248, 21504, 21760, 22016, 22272,
22528, 22784, 23040, 23296, 23552, 23808, 24064, 24320,
24576, 24832, 25088, 25344, 25600, 25856, 26112, 26368,
26624, 26880, 27136, 27392, 27648, 27904, 28160, 28416,
28672, 28928, 29184, 29440, 29696, 29952, 30208, 30464,
30720, 30976, 31232, 31488, 31744, 32000, 32256, 32767
};

enum CMD_VR_IOCTRL
{
	CMD_NORMAL_MODE,
	CMD_START_TEST_MODE,
	CMD_START_FACTORY_CALIBRATION,
	CMD_SAVE_WHELL_DATA,
	CMD_SAVE_BREAK_DATA,
	CMD_SAVE_GAS_DATA,
	CMD_RECIEVED_12BIT_DATA,
	CMD_USER_FLASH_WRITE,
	CMD_START_USER_CALIBRATION,
	CMD_GET_VR_DATA,
	CMD_GET_CALBRIATION_DATA,
	CMD_GET_ADC_DATA,
	CMD_GET_12BIT_DATA,
	CMD_ERASE_DATA,
	CMD_WRITE_DATA,
};

enum MODE_CALIBRATION
{
	MODE_NORMAL,
	MODE_TEST,
	MODE_RETRIEV_12BITDATA,
	MODE_ERASE_DATA,
	MODE_WRITE_DATA,
};


struct timer_list btetimer;
static void bte_do(unsigned long arg){
	if(!bte_loop){
		printk("500ms timeout!\n");
	}
	bte_timeout = 1;
}


static void bte_timer_init(void){
    /* Timer 初始化 */
   init_timer(&btetimer);

   /* define timer 要執行之函式 */
  btetimer.function = bte_do;

  /* define timer 傳入函式之 Data */
  btetimer.data = ((unsigned long) 0);

  /* define timer Delay 10ms */
  btetimer.expires = jiffies + (HZ/2);//500ms

  /* 啟動 Timer*/
  add_timer(&btetimer);
}

/*a40i_vol_proc*/
static int a40i_vol_proc_show(struct seq_file *m, void *v) {
	proc_flag = 0;
	seq_printf(m, "%d\n", key_val);
	return 0;
}
static int a40i_vol_proc_open(struct inode *inode, struct  file *file) {
	return single_open(file, a40i_vol_proc_show, NULL);
}
static const struct file_operations a40i_vol_proc_fops = {
	.owner = THIS_MODULE,
	.open = a40i_vol_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#define TESTGAMETYPE 1
#if TESTGAMETYPE
ssize_t testgametype_write_proc(struct file *flip, const char* buf, size_t count, loff_t *offp){
	char msg[128];
	int ret_ignore = 0;
	memset(msg, '\0', sizeof(msg));
	ret_ignore = copy_from_user(msg, buf, count);
	bte_gametype = simple_strtol(msg, NULL, 10);
//	proc_flag = 1;
	return count;
}

struct file_operations testgametype_fops = {
	write: testgametype_write_proc
};
#endif


#define TEST 0
#if TEST
ssize_t test_write_proc(struct file *flip, const char* buf, size_t count, loff_t *offp){
	char msg[128];
	int ret_ignore = 0;
	memset(msg, '\0', sizeof(msg));
	ret_ignore = copy_from_user(msg, buf, count);
	key_val = simple_strtol(msg, NULL, 10);
	proc_flag = 1;
	return count;
}

struct file_operations test_fops = {
	write: test_write_proc
};
#endif

#define JSTEST 0
#if defined(GAME_STARWARS) && defined(JSTEST)
ssize_t jstest_write_proc(struct file *flip, const char* buf, size_t count, loff_t *offp){
	char msg[128];
	int num = 0, c = 0, x = 0, y = 0;
	memset(msg, '\0', sizeof(msg));
	copy_from_user(msg, buf, count);
	num = sscanf(msg, "%d %d", &x, &y);
	if(num != 2)
		return -EFAULT;
	c = strlen(msg);
	*offp = c;
	input_report_abs(input_js, ABS_X, x);
	input_report_abs(input_js, ABS_Y, y);
	input_sync(input_js);
	bte_jsx = x;
	bte_jsy = y;
	return c;
}

struct file_operations jstest_fops = {
	write: jstest_write_proc
};
#endif

#if defined(GAME_NBA)
ssize_t reboot_write_proc(struct file *flip, const char* buf, size_t count, loff_t *offp){
	char msg[128];
	int ret_ignore = 0;
	memset(msg, '\0', sizeof(msg));
	ret_ignore = copy_from_user(msg, buf, count);
	g_power_off = 2;
	return count;
}

struct file_operations reboot_fops = {
	write: reboot_write_proc
};
#endif
ssize_t unlock_write_proc(struct file *flip, const char* buf, size_t count, loff_t *offp){
	int ret;
	unsigned long long res;
	ret = kstrtoull_from_user(buf, count, 10, &res);
	if(ret){
		return ret;
	}else{
		printk("unlock got %llu\n", res);
		sign_unlock = res;
		*offp = count;
		return count;
	}
}

struct file_operations unlock_fops = {
	write: unlock_write_proc
};

#define TESTMODE 1
#if TESTMODE
static int a40i_testmode_proc_show(struct seq_file *m, void *v) {
	int tmp = 0;
	printk("[a40i_testmode_proc_show]: testmode_bootup=%d p1_start=%d p1_b=%d\n", testmode_bootup, p1_start, p1_b);
	if(testmode_bootup ==1 && p1_start == 1 && p1_b == 1){	
		tmp =1;
	} else if(testmode_bootup == 2 && p1_start == 1 && p1_x == 1){
		tmp = 2;
	}
	
	seq_printf(m, "%d\n", tmp);
	return 0;
}
static int a40i_testmode_proc_open(struct inode *inode, struct  file *file) {
	return single_open(file, a40i_testmode_proc_show, NULL);
}
static const struct file_operations a40i_testmode_proc_fops = {
	.owner = THIS_MODULE,
	.open = a40i_testmode_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif
//starwars ioctl
static DECLARE_WAIT_QUEUE_HEAD(wq);
static int wait_received_data = 0;
//static ssize_t drv_ioctl(struct file *filp, unsigned int cmd, unsigned int data[4]) {
static ssize_t drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	printk("outrun ioctl %d\n", cmd);
	unsigned char *data = (unsigned char*)arg;
	struct VR_DATA temp;
	struct CAL_DATA temp_cal;
	struct ADC_DATA temp_adc;

	switch(cmd){
		case CMD_NORMAL_MODE:
			printk("CMD_NORMAL_MODE\n");
			mutex_lock(&outrun_mutex);
			mode_flag = MODE_NORMAL;
			mutex_unlock(&outrun_mutex);
			break;

		case CMD_START_TEST_MODE:
			printk("CMD_START_TESTL_MODE\n");
			mutex_lock(&outrun_mutex);
			mode_flag = MODE_TEST;
			get_data_once = 1;
			mutex_unlock(&outrun_mutex);
			wait_received_data = 0;
			wait_event_interruptible(wq, wait_received_data == 1);
			wait_received_data = 0;
			temp_cal = caldata;
			copy_to_user((struct CAL_DATA *)arg, &temp_cal, sizeof(temp_cal));
			printk("CMD_GET_CALBIRATION_DATA: %02x %02x %02x %02x %02x %02x %02x %02x\n", caldata.wheel_low, caldata.wheel_mid_low, caldata.wheel_mid_high, caldata.wheel_high, caldata.gas_low, caldata.gas_high, caldata.break_low, caldata.break_high);
			break;

		case CMD_RECIEVED_12BIT_DATA:
			printk("CMD_RETRIEVE_128BITL_DATA\n");
			mutex_lock(&outrun_mutex);
			mode_flag = MODE_RETRIEV_12BITDATA;
			get_data_once = 1;
			mutex_unlock(&outrun_mutex);
			wait_received_data = 0;
			wait_event_interruptible(wq, wait_received_data == 1);
			wait_received_data = 0;
			temp_adc = adcdata;
			copy_to_user((struct ADC_DATA *)arg, &temp_adc, sizeof(temp_adc));
			printk("CMD_Retrevie data: %02x %02x %02x %02x %02x %02x\n", adcdata.adc_wheelh, adcdata.adc_wheell, adcdata.adc_gash, adcdata.adc_gasl, adcdata.adc_breakh, adcdata.adc_breakl);			
			break;
			
		case CMD_GET_VR_DATA:
			printk("CMD_GET_VR_DATA\n");
			temp.vr_wheel = vrdata.vr_wheel;
			temp.vr_break = vrdata.vr_break;
			temp.vr_gas = vrdata.vr_gas;
			temp.vr_gear = vrdata.vr_gear;
			copy_to_user((struct VR_DATA *)arg, &temp, sizeof(temp));
			break;

		case CMD_GET_CALBRIATION_DATA:
			//printk("CMD_GET_CALBIRATION_DATA: %02x %02x %02x %02x\n", caldata.wheel_low, caldata.wheel_mid_low, caldata.wheel_mid_high, caldata.wheel_high);
			mutex_lock(&outrun_mutex);
			temp_cal.wheel_low = caldata.wheel_low; temp_cal.wheel_mid_low = caldata.wheel_mid_low;
			temp_cal.wheel_mid_high = caldata.wheel_mid_high; temp_cal.wheel_high = caldata.wheel_high;
			temp_cal.gas_low = caldata.gas_low; temp_cal.gas_high = caldata.gas_high;
			temp_cal.break_low = caldata.break_low; temp_cal.break_high = caldata.break_high;
			mutex_unlock(&outrun_mutex);
			copy_to_user((struct CAL_DATA *)arg, &temp_cal, sizeof(temp_cal));
			break;
		case CMD_GET_ADC_DATA:
			//printk("CMD_GET_ADC_DATA: %02x %02x %02x %02x %02x %02x\n", adcdata.adc_wheelh, adcdata.adc_wheell, adcdata.adc_gash, adcdata.adc_gasl, adcdata.adc_breakh, adcdata.adc_breakl);
			mutex_lock(&outrun_mutex);
			temp_adc = adcdata;
			mutex_unlock(&outrun_mutex);
			copy_to_user((struct ADC_DATA *)arg, &temp_adc, sizeof(temp_adc));
			break;

		case CMD_ERASE_DATA:
			mutex_lock(&outrun_mutex);
			mode_flag = MODE_ERASE_DATA;
			erase_data_type = 0;
			get_data_once = 1;
			mutex_unlock(&outrun_mutex);
			wait_received_data = 0;
			wait_event_interruptible(wq, wait_received_data == 1);
			wait_received_data = 0;
			copy_to_user(data+12, &erase_data_type, sizeof(erase_data_type));			
			break;
		case CMD_WRITE_DATA:			
			memcpy(&bteWrite[3], data, 12);			
			mutex_lock(&outrun_mutex);
			mode_flag = MODE_WRITE_DATA;
			write_data_type = 0;
			get_data_once = 1;
			mutex_unlock(&outrun_mutex);
			wait_received_data = 0;
			wait_event_interruptible(wq, wait_received_data == 1);
			wait_received_data = 0;
			copy_to_user(data+12, &write_data_type, sizeof(write_data_type));			
			break;
		
	}

	return 0;
}
static const struct file_operations drv_fops = {
	.owner = THIS_MODULE,
	.compat_ioctl = drv_ioctl,
};
static void cleanup(int device_created)
{
    if (device_created) {
        device_destroy(myclass, major);
        cdev_del(&mycdev);
    }
    if (myclass)
        class_destroy(myclass);
    if (major != -1)
        unregister_chrdev_region(major, 1);
}
static int outrun_ioctl_init(void) {
	int device_created = 0;
	/* cat /proc/devices */
	if (alloc_chrdev_region(&major, 0, 1, "outrun_ioctl_proc") < 0)
		goto error;
	/* ls /sys/class */
	if ((myclass = class_create(THIS_MODULE, "outrun_ioctl_sys")) == NULL)
		goto error;
	/* ls /dev/ */
	if (device_create(myclass, NULL, major, NULL, "outrun_ioctl_dev") == NULL)
		goto error;
	device_created = 1;
	cdev_init(&mycdev, &drv_fops);
	if (cdev_add(&mycdev, major, 1) == -1)
		goto error;
	printk("outrun_ioctl_success!\n");
	return 0;
error:
	printk("outrun_ioctl_error!\n");
	cleanup(device_created);
	return -1;
}
static void outrun_ioctl_exit(void) {
	printk("outrun_ioctl_exit!\n");
}

static inline void bte_report_key(uint8_t value, int key){
	input_report_key(bte_dev, key, (value > 0) ? 1:0);
}

static inline void bte_report_key2A(uint8_t index, uint8_t value, uint8_t mask, int key){
        static uint8_t buf[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t temp = (value & mask);
        if((buf[index] & mask) != (temp)){
                input_report_key(bte_dev, key, (temp > 0) ? 1:0);
                if(temp){
                        buf[index] |= mask;
                }else{
                        buf[index] &= ~mask;
                }
        }
}

static inline void bte_report_key2B(uint8_t index, uint8_t value, uint8_t mask, int key){
        static uint8_t buf[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t temp = (value & mask);
        if((buf[index] & mask) != (temp)){
                input_report_key(bte_dev, key, (temp > 0) ? 1:0);
                if(temp){
                        buf[index] |= mask;
                }else{
                        buf[index] &= ~mask;
                }
        }
}

static inline void bte_report_key2D(uint8_t index, uint8_t value, uint8_t mask, int key) {
	static uint8_t buf[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t temp = (value & mask);
	if ((buf[index] & mask) != (temp)) {
		input_report_key(input_js, key, (temp > 0) ? 1 : 0);
		if (temp) {
			buf[index] |= mask;
		}
		else {
			buf[index] &= ~mask;
		}
	}
}

#include <linux/kmod.h>
static inline int run_shell(char* cmd){
	char* argv[6], *envp[6];
	int result = -1;
	argv[0] = "/bin/bash";
	argv[1] = "-c";
	argv[2] = cmd;
	argv[3] = NULL;
	envp[0] = "HOME=/";
	envp[1] = "TERM=linux";
	envp[2] = "PATH=/sbin:/usr/sbin:/bin:/usr/bin";
	envp[3] = "NULL";
	result = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC); //do not use UMH_KILLABLE
	return result;
}

static inline int run_power_off(void){
	int result=-1;
	char* argv1[] = { "/bin/echo", "1", ">", "/sys/class/gpio_sw/PH14/data", NULL};
	char* argv2[] = { "/bin/echo", "0", ">", "/sys/class/gpio_sw/PI5/data", NULL};
	char* argv3[] = { "/bin/echo", "0", ">", "/sys/class/gpio_sw/PB9/data", NULL};
	char* argv4[] = { "/usr/bin/killall", "-15", "ash", NULL};
	char* argv5[] = { "/usr/bin/killall", "-15", "OnlineHub", NULL};
	char* argv6[] = { "/usr/bin/killall", "-15", "nbajam", NULL};
	char* argv7[] = { "/usr/bin/killall", "-15", "menu", NULL};
	char* argv8[] = { "/bin/umount", "/moo/docs", NULL};
	char* argv9[] = { "/bin/sleep", "3", NULL};
	char* argv10[] = { "/usr/bin/fbv", "/pic/powering-off.jpg", "-s", "1", NULL};
	char* argv11[] = { "/bin/sleep", "3", NULL};
	char* argv12[] = { "/sbin/rmmod", "8188fu", NULL};
	char* argv13[] = { "/sbin/rmmod", "8188eu", NULL};
	char* envp[] = { "HOME=/", "TERM=linux", "PATH=/sbin:/usr/sbin:/bin:/usr/bin", NULL};
	result = call_usermodehelper(argv1[0], argv1, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv2[0], argv2, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv3[0], argv3, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv4[0], argv4, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv5[0], argv5, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv6[0], argv6, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv7[0], argv7, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv8[0], argv8, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv9[0], argv9, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv10[0], argv10, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv11[0], argv11, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv12[0], argv12, envp, UMH_WAIT_PROC);
	result = call_usermodehelper(argv13[0], argv13, envp, UMH_WAIT_PROC);
	return result;
}

extern struct task_struct *vsync_task;
static inline void stop_nba(void){
	struct siginfo info;
	info.si_signo = 15;
	send_sig_info(info.si_signo, &info, vsync_task);
}

extern int vsync_sleep;
static int bte_sleep_thread(void* arg){
#define SLEEP_TIME 5 * 60 * 1000
	long long int sleep_time = 0;
	printk("bte_sleep_thread start\n");
	while(!kthread_should_stop()){
		int result = 0;
		run_shell("/bin/sync");
		run_power_off();
		msleep(1000);
		run_shell("/bin/sync;/bin/sync");
		g_power_off = 1;
		return 0;
	}
	printk("bte_sleep_thread exit\n");
	//vsync_sleep = 0;
	return 0;
}

int deck2A[2][2][6] = {
#if defined(GAME_H2H) || defined(GAME_H2H_BLACK_SF2)
	{//player1
		{KEY_DOWN,   KEY_LEFT,   KEY_UP,     KEY_RIGHT,   KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_Z,      KEY_Q,      KEY_S,       KEY_X,            KEY_W},    //buf[5]
	},
	{//player2
		{KEY_C,      KEY_G,      KEY_D,      KEY_B,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP5.    KEY_KP7,    KEY_KP1,     KEY_KP2,          KEY_KP8},//buf[9]
	},
#elif defined(GAME_H2H_PACMAN) || defined(GAME_H2H_SAM) || defined(GAME_H2H_BLACK_PACMAN)  || defined(GAME_H2H_40TH_PACMAN10) || defined(GAME_H2H_MS_PACMAN8)
	{//player1
		{KEY_DOWN,   KEY_LEFT,   KEY_UP,     KEY_RIGHT,   KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_Z,      KEY_Q,      KEY_S,       KEY_X,            KEY_W},    //buf[5]
	},
	{//player2
		{KEY_C,      KEY_G,      KEY_D,      KEY_B,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP1,    KEY_KP7,    KEY_KP5,     KEY_KP2,          KEY_KP8},//buf[9]
	},
#elif defined(GAME_KC)
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_X,      KEY_Z,      KEY_S,      KEY_W,       KEY_Q,            KEY_A},    //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP2,    KEY_KP1,    KEY_KP5,    KEY_KP8,     KEY_KP7,          KEY_KP4},//buf[9]
	},
#elif defined(GAME_NBA) || defined(GAME_GOLDENAXE) || defined(GAME_PONG) 
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_Z,      KEY_Q,      KEY_X,       KEY_S,            KEY_F1},   //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP1,    KEY_KP7,    KEY_KP2,     KEY_KP5,          KEY_KP8},//buf[9]
	},
#elif defined(GAME_SF2) || defined(GAME_FF)
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_W,      KEY_S,      KEY_Z,      KEY_Q,       KEY_X,            KEY_A},    //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP8,    KEY_KP5,    KEY_KP1,    KEY_KP7,     KEY_KP2,          KEY_KP4},//buf[9]
	},
#elif defined(GAME_40TH_PACMAN)
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_Z,      KEY_Q,      KEY_S,       KEY_X,            KEY_W},    //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP5,    KEY_KP7,    KEY_KP1,     KEY_KP2,          KEY_KP8},//buf[9]
	},
#elif defined(GAME_FROGGER)
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_Z,      KEY_Q,      KEY_S,       KEY_X,            KEY_W},    //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP1,    KEY_KP7,    KEY_KP5,     KEY_KP2,          KEY_KP8},//buf[9]
	},
#elif defined(GAME_OUTRUN)
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_Z,      KEY_Q,      KEY_S,       KEY_X,            KEY_W},    //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP1,    KEY_KP7,    KEY_KP5,     KEY_KP2,          KEY_KP8},//buf[9]
	},
#else
	{//player1
		{KEY_RIGHT,  KEY_DOWN,   KEY_LEFT,   KEY_UP,      KEY_RIGHTSHIFT,   KEY_ENTER},//buf[4]
		{KEY_A,      KEY_S,      KEY_Q,      KEY_Z,       KEY_X,            KEY_W},    //buf[5]
	},
	{//player2
		{KEY_B,      KEY_C,      KEY_G,      KEY_D,       KEY_KP9,          KEY_KP6},//buf[8]
		{KEY_KP4,    KEY_KP5,    KEY_KP7,    KEY_KP1,     KEY_KP2,          KEY_KP8},//buf[9]
	},
#endif
};
#if defined(GAME_TMNT) || defined(GAME_NBA) || defined(GAME_GOLDENAXE) || defined(GAME_PONG) || defined(GAME_OUTRUN)
int deck2B[2][2][6] = {
	#if defined(GAME_NBA) || defined(GAME_GOLDENAXE) || defined(GAME_PONG) 
		{//player3
			{KEY_T,   KEY_R,   KEY_E,   KEY_KP0, KEY_2,   KEY_I}, //buf[4]: Right,Down,Left,Up,Coin,Start
			{KEY_3,   KEY_U,   KEY_5,   KEY_O,   KEY_4,   KEY_6}, //buf[5]: B,Y,X,A,C,Z
		},
		{//player4
			{KEY_J,   KEY_H,   KEY_F,   KEY_P,   KEY_7,   KEY_N}, //buf[8]: Right,Down,Left,Up,Coin,Start
			{KEY_8,   KEY_V,   KEY_0,   KEY_M,   KEY_9,   KEY_1}, //buf[9]: B,Y,X,A,C,Z
		},
	#elif defined(GAME_OUTRUN)
		{//player3
			{KEY_T,   KEY_R,   KEY_E,   KEY_KP0, KEY_2,   KEY_I}, //buf[4]: Right,Down,Left,Up,Coin,Start
			{BTN_START,   BTN_EAST,   KEY_5,   KEY_O,   KEY_4,   KEY_6}, //buf[5]: B,Y,X,A,C,Z
		},
		{//player4
			{KEY_J,   KEY_H,   KEY_F,   KEY_P,   KEY_7,   KEY_N}, //buf[8]: Right,Down,Left,Up,Coin,Start
			{KEY_8,   KEY_V,   KEY_0,   KEY_M,   KEY_9,   KEY_1}, //buf[9]: B,Y,X,A,C,Z
		},
	#else
		{//player3
			{KEY_T,   KEY_R,   KEY_E,   KEY_KP0, KEY_2,   KEY_I}, //buf[4]
			{KEY_3,   KEY_4,   KEY_5,   KEY_U,   KEY_O,   KEY_6}, //buf[5]
		},
		{//player4
			{KEY_J,   KEY_H,   KEY_F,   KEY_P,   KEY_7,   KEY_N}, //buf[8]
			{KEY_8,   KEY_9,   KEY_0,   KEY_V,   KEY_M,   KEY_1}, //buf[9]
		},
	#endif
};
#endif

static int bte_thread(void* arg){
	loff_t pos = 0, pos2 = 0;
	int len = 0;
	uint8_t buf[32];
	uint8_t buf2[72];
	uint8_t bte_buf_tx[3] = {0xA6, 0x01, 0x00};
	uint8_t bte_version_tx[5] = {0xA6, 0x02, 0xf1, 0x02, 0x00};
#if defined(GAME_STARWARS)
	uint8_t bte_buf_tx_testmode[4] = {0xA6, 0x02, 0xF2, 0x11}; //Start Test Mode
	uint8_t bteEnFactoryValue[4]   = {0xA6, 0x02, 0xF7, 0x01}; //Enable Factory  Value, Disable User Value
	uint8_t bteDisFactoryValue[4]  = {0xA6, 0x02, 0xF8, 0xEE}; //Disable Factory Value, Enable User Value
	uint8_t bteFlashErase[4]       = {0xA6, 0x02, 0xF4, 0xEE}; //User/Factory Flash Erase Calibration Value
	uint8_t bteUserFlashErase[4]   = {0xA6, 0x02, 0xF6, 0xEE}; //User Flash Erase Calibration Value
	int bValueOnlyFirst = 1;
#elif defined(GAME_NBA)
	uint8_t nba_sleep_pressed = 0;
	uint8_t btePowerOff[4]   = {0xA6, 0x02, 0xE0, 0x01}; //User Flash Erase Calibration Value
	uint8_t bteReboot[4]   = {0xA6, 0x02, 0xE0, 0x02}; //User Flash Erase Calibration Value
#endif
	uint8_t bteReadRaw[4] = {0xA6, 0x02, 0xF0, 0x00};
	uint8_t bteReadAdc[4] = {0xA6, 0x02, 0xF2, 0x11};
	uint8_t bteErase[4] = {0xA6, 0x02, 0xF6, 0xEE};

	int bFirstCmd = 0;
	//a40i-volume-control
	printk("bte_thread start.........\n");

	set_current_state(TASK_UNINTERRUPTIBLE);

	printk("bte timer start!\n");
	bte_timer_init();	

#if 0
	printk("bte_loop.....\n");
	while(!bte_loop){// 先判斷buf[6]是否有跟gametype match
		if(!bte_timeout){
			vfs_write(bte_fp, bte_buf_tx, 3, &pos);//跟mcu要資料
			memset(buf, 0, sizeof(buf));
			len = vfs_read(bte_fp, buf, sizeof(buf), &pos);
			if(len > 0 && buf[0]==0xA7 && buf[1]==0x10 && buf[6]==bte_buf_tx[2]){
				printk("gametype matched...\n");
				bte_loop = 1;
			}
		}
		else{
			printk("bte timer time out!\n");
			printk("gametype default 0x00!\n");
			bte_buf_tx[2]=0x00;
			bte_loop = 1;
		}
		msleep(10);
	}

	msleep(10);
#endif
	while(!kthread_should_stop()){
		int i = 0, n=0;
		struct timeval t1, t2;
		unsigned long val;

		if(bte_fp){
		vfs_write(bte_fp, bte_buf_tx, 3, &pos);//跟mcu要資料
		memset(buf, 0, sizeof(buf));
		len = vfs_read(bte_fp, buf, sizeof(buf), &pos);
		
		if(len > 0){
			if(buf[0]==0xA7 && buf[1]==0x10){
				//p1
				if(len != 18){
					printk("%x ", buf[0]);
					printk("%x ", buf[1]);
					printk("%x ", buf[2]);
					printk("%x ", buf[3]);
					printk("%x ", buf[4]);
					printk("%x ", buf[5]);
					printk("%x\n", buf[6]);
					continue;
				}
				if(buf[2] == 0x00){//Volume Control Mode
					key_val = 0;
				}else if(buf[2] == 0x01){
					key_val = 1;
				}else if(buf[2] == 0x02){
					key_val = 2;
				}

				if(key_val - key_val_org){
#if defined(VOL_BAR)
					if(key_val == 0){//no sound
						bte_report_key(0x0, KEY_VOLUMEUP);
						bte_report_key(0x1, KEY_VOLUMEDOWN);
						msleep(20);
					}else if(key_val == 1){//middle sound
						bte_report_key(0x0, KEY_VOLUMEUP);
						bte_report_key(0x0, KEY_VOLUMEDOWN);
						msleep(20);
					}else if(key_val==2){//large sound
						bte_report_key(0x0, KEY_VOLUMEDOWN);
						bte_report_key(0x1, KEY_VOLUMEUP);
						msleep(20);
					}
#endif
					key_val_org = key_val;
					proc_flag = 1;
				}

			if (buf[4] & 0x20) {
				input_report_key(input_js, KEY_S, 1);
				p1_start = 1;
			} else {
				input_report_key(input_js, KEY_S, 0);
				p1_start = 0;
			}
			
			if(buf[8] & 0x20) {
				input_report_key(input_js, KEY_D, 1);
				p1_x = 1;
			} else {
				input_report_key(input_js, KEY_D, 0);
				p1_x = 0;
			}

			//printk("UR1:KK[%02x],Y1[%02x],Z1[%02x],R1[%02x],Y2[%02x],Z2[%02x],R2[%02x]\n", buf[2],buf[4],buf[5],buf[6],buf[8],buf[9],buf[10]);

			
#if TESTMODE
			if(bFirstCmd == 0){//skip first time result
				bFirstCmd = 1;
			}else if(bFirstCmd == 1){
				//printk("key_val=%d, p1_start=%d, p2_start=%d, p1_a=%d, p2_kp2=%d\n", key_val, p1_start, p2_start, p1_a, p2_kp2);
				if(p1_start == 1 && p1_b == 1){
					testmode_bootup = 1;
					printk("[boot]: entry test mode\n");
				} else if(p1_start == 1 && p1_x == 1){
					testmode_bootup = 2;
				} else if(p1_x == 1 && p1_xl == 1 && p1_xr == 1){
					testmode_bootup = 3;
				}
				bFirstCmd = 2;
			}else{
				if(testmode_bootup == 1){
					if(p1_start != 1 || p1_b != 1){
						testmode_bootup = 0;
					}
				}
				if(testmode_bootup == 2){
					if(p1_start != 1 || p1_x != 1){
						testmode_bootup = 0;
					}
				}				
			}
#endif
			}
		}else{//len > 0
			msleep(10);
		}
		}

#if 0
		uint8_t i;
		if(bte_fp2){
			if(mode_flag == 0){
				vfs_write(bte_fp2, bte_buf_tx, 3, &pos);
				bValueOnlyFirst = 1;
				msleep(10);
			}else if(mode_flag == 1){//Test Mode:Read VR Data, mode_flag flow 1->2->3->1
				if(bValueOnlyFirst == 1){//stanley
					printk("[a40i] enable factory value mode=[%d]\n", mode_flag);
					vfs_write(bte_fp2, bteEnFactoryValue, 4, &pos);
					msleep(10);
					memset(buf, 0, sizeof(buf));
					len = vfs_read(bte_fp2, buf, sizeof(buf), &pos);
					printk("[a40i] len=%d, [%02x][%02x][%02x][%02x]\n", len, buf[0], buf[1], buf[2], buf[3]);
					bValueOnlyFirst = 0;
				}
				//printk("[a40i] enable factory mode\n");
				vfs_write(bte_fp2, bte_buf_tx_testmode, 4, &pos);
				msleep(10);
			}else if(mode_flag == 2){
				vfs_write(bte_fp2, bteFlashErase, 4, &pos);
				msleep(20);//mcu need time to do erase
			}else if(mode_flag == 3){//write factory and user value to MCU
				mutex_lock(&starwars_mutex);
				vfs_write(bte_fp2, bteFlashWrite, 17, &pos);
				mutex_unlock(&starwars_mutex);
				msleep(200);//mcu need time to do flash back
			}else if(mode_flag == 4){//Test Mode:Read VR Data, mode_flag flow 4->5->6->4
				if(bValueOnlyFirst == 1){//stanley
					printk("[a40i] enable factory value mode=[%d]\n", mode_flag);
					vfs_write(bte_fp2, bteEnFactoryValue, 4, &pos);
					msleep(10);
					memset(buf, 0, sizeof(buf));
					len = vfs_read(bte_fp2, buf, sizeof(buf), &pos);
					printk("[a40i] len=%d, [%02x][%02x][%02x][%02x]\n", len, buf[0], buf[1], buf[2], buf[3]);
					bValueOnlyFirst = 0;
				}
				vfs_write(bte_fp2, bte_buf_tx_testmode, 4, &pos);
				msleep(20);
			}else if(mode_flag == 5){
				vfs_write(bte_fp2, bteUserFlashErase, 4, &pos);
				msleep(20);//mcu need time to do erase
			}else if(mode_flag == 6){//write user value to MCU
				mutex_lock(&starwars_mutex);
				vfs_write(bte_fp2, bteUserFlashWrite, 17, &pos);
				mutex_unlock(&starwars_mutex);
				msleep(200);//mcu need time to do flash back
			}
			memset(buf, 0, sizeof(buf));
			len = vfs_read(bte_fp2, buf, sizeof(buf), &pos);
			if(len > 0){
				if((mode_flag == 0 || mode_flag == 1 || mode_flag == 4) &&
						buf[0] == 0xA7 && buf[1] == 0x10 && buf[3] == 0x05 && len == 18){
						if(bte_jsx != buf[4]){//0xAA Left/Right VR 8bit
							bte_jsx = buf[4];
							input_report_abs(input_js, ABS_X, bte_jsx);
						}
						if(bte_jsy != buf[5]){//0xBB Front/Back VR 8bit
							bte_jsy = buf[5];
							input_report_abs(input_js, ABS_Y, bte_jsy);
						}
						if(bte_print){
							printk("[mcu] ");
							for(i=0;i<18;i++){
								printk("0x%02x ", buf[i]);
							}
							printk("\n");
						}
						if(mode_flag == 1 || mode_flag == 4){//receive data
							highbit = buf[10];
							if(VR12bit_AdjData_x != ((highbit<<8) + buf[11])){
								VR12bit_AdjData_x = ((highbit<<8) + buf[11]);
							}
							highbit = buf[12];
							if(VR12bit_AdjData_y != ((highbit<<8) + buf[13])){
								VR12bit_AdjData_y = ((highbit<<8) + buf[13]);
							}
							highbit = buf[14];
							if(VR12bit_RawData_x != ((highbit<<8) + buf[15])){
								VR12bit_RawData_x = ((highbit<<8) + buf[15]);
							}
							highbit = buf[16];
							if(VR12bit_RawData_y != ((highbit<<8) + buf[17])){
								VR12bit_RawData_y = ((highbit<<8) + buf[17]);
							}
							if(mode_flag == 4){
								if(user_mode_skip == 0){
									if(VR12bit_RawData_x < user_left_x) user_left_x = VR12bit_RawData_x;
									if(VR12bit_RawData_y < user_top_y)  user_top_y  = VR12bit_RawData_y;
									if(VR12bit_RawData_x > user_right_x) user_right_x = VR12bit_RawData_x;
									if(VR12bit_RawData_y > user_bottom_y) user_bottom_y = VR12bit_RawData_y;
									//printk("[a40i] (0x%04x,0x%04x)-(0x%04x,0x%04x)\n", user_left_x, user_top_y, user_right_x, user_bottom_y);
								}else user_mode_skip= 0;
							}
							if(bte_print){
								printk("[a40i] AdjData=(0x%04x, 0x%04x), RawData=(0x%04x, 0x%04x)\n", VR12bit_AdjData_x, VR12bit_AdjData_y, VR12bit_RawData_x, VR12bit_RawData_y);
							}
						}
				}else if(mode_flag == 2 && buf[0] == 0xA7 && buf[1] == 0x0E && buf[2] == 0xF4){//erase success?
					printk("factory[mcu] ");
					for(i=0;i<16;i++){
						printk("0x%02x ", buf[i]);
					}
					printk("\n");
					for(i=0;i<13;i++){
						if(buf[i+3]!=0xFF){
							printk("[a40i] Flash Factory/User Erase Fail!\n");
							break;
						}
					}
					mutex_lock(&starwars_mutex);
					mode_flag=3;
					mutex_unlock(&starwars_mutex);
					printk("[a40i] flash factory/user erase success.\n");
				}else if(mode_flag == 3 && buf[0] == 0xA7 && buf[1] == 0x0E && buf[2] == 0xF3){//flash success?
					printk("factory[a40] ");
					for(i=0;i<17;i++){
						printk("0x%02x ", bteFlashWrite[i]);
					}
					printk("\n");
					printk("[mcu] ");
					for(i=0;i<16;i++){
						printk("0x%02x ", buf[i]);
					}
					printk("\n");

					for(i=0;i<13;i++){
						if(buf[i+3]!=bteFlashWrite[i+3]){
							printk("[a40] Flash Factory Write Fail!\n");
							break;
						}
					}
					mutex_lock(&starwars_mutex);
					mode_flag=1;
					mutex_unlock(&starwars_mutex);
					printk("[a40i] flash factory write success.\n");
					//Using User Calibration
					printk("[a40i] disable factory value mode[%d]\n", mode_flag);
					vfs_write(bte_fp2, bteDisFactoryValue, 4, &pos);
					msleep(200);
					memset(buf, 0, sizeof(buf));
					len = vfs_read(bte_fp2, buf, sizeof(buf), &pos);
					printk("[a40i] len=%d, [%02x][%02x][%02x][%02x]\n", len, buf[0], buf[1], buf[2], buf[3]);
				}else if(mode_flag == 5 && buf[0] == 0xA7 && buf[1] == 0x0E && buf[2] == 0xF6){//erase success?
					printk("user[mcu] ");
					for(i=0;i<16;i++){
						printk("0x%02x ", buf[i]);
					}
					printk("\n");
					for(i=0;i<13;i++){
						if(buf[i+3]!=0xFF){
							printk("[a40i] Flash User Erase Fail!\n");
							break;
						}
					}
					mutex_lock(&starwars_mutex);
					mode_flag=6;
					mutex_unlock(&starwars_mutex);
					printk("[a40i] flash user erase success.\n");
				}else if(mode_flag == 6 && buf[0] == 0xA7 && buf[1] == 0x0E && buf[2] == 0xF5){//flash success?
					printk("user[a40] ");
					for(i=0;i<17;i++){
						printk("0x%02x ", bteUserFlashWrite[i]);
					}
					printk("\n");
					printk("[mcu] ");
					for(i=0;i<16;i++){
						printk("0x%02x ", buf[i]);
					}
					printk("\n");

					for(i=0;i<13;i++){
						if(buf[i+3]!=bteUserFlashWrite[i+3]){
							printk("[a40] Flash User Write Fail!\n");
							break;
						}
					}
					mutex_lock(&starwars_mutex);
					mode_flag=4;
					mutex_unlock(&starwars_mutex);
					printk("[a40i] flash user write success.\n");
					//Using User Calibration
					printk("[a40i] disable factory value mode[%d]\n", mode_flag);
					vfs_write(bte_fp2, bteDisFactoryValue, 4, &pos);
					msleep(200);
					memset(buf, 0, sizeof(buf));
					len = vfs_read(bte_fp2, buf, sizeof(buf), &pos);
				}
			}
		}
#endif
	
		if(bte_fp2){			
			if(mode_flag == MODE_TEST){
				if(get_data_once > 0){
					printk("mode_test\n");
					vfs_write(bte_fp2, bteReadRaw, 4, &pos2);
					msleep(10);
					memset(buf2, 0, sizeof(buf2));
					len = vfs_read(bte_fp2, buf2, sizeof(buf2), &pos2);
					
					printk("calibration len(%d): %d\n", get_data_once, len);
					for(i = 0; i < len; i ++){
						printk("buf[%02d]=%02x ", i, buf2[i]);						
					}
					printk("\n");
					
					if(len >= 48){
						mutex_lock(&outrun_mutex);
						caldata.wheel_low = buf2[34]; caldata.wheel_mid_low = buf2[35];
						caldata.wheel_mid_high = buf2[36]; caldata.wheel_high = buf2[37];
						caldata.gas_low = buf2[38]; caldata.gas_high = buf2[41];
						caldata.break_low = buf2[42]; caldata.break_high = buf2[45];
						mutex_unlock(&outrun_mutex);
						get_data_once --;
						wait_received_data = 1;
						wake_up_all(&wq);
					} else {
						printk("retreive calibration data fail.\n");
					}
					msleep(10);
				}	
			} else if(mode_flag == MODE_RETRIEV_12BITDATA){
				if(get_data_once > 0) {
					vfs_write(bte_fp2, bteReadAdc, 4, &pos2);
					msleep(10);
					memset(buf2, 0, sizeof(buf2));
					len = vfs_read(bte_fp2, buf2, sizeof(buf2), &pos2);

					printk("ADC_DATA len(%d): %d\n", get_data_once, len);
					for(i = 0; i < len; i ++){
						printk("buf[%02d]=%02x ", i, buf2[i]);						
					}
					printk("\n");
					
					if(len >= 18 && buf2[3] == 0x05 && buf2[7] == 0x15){
						mutex_lock(&outrun_mutex);
						adcdata.wheel_data = buf2[4];
						adcdata.gas_data = buf2[5];
						adcdata.break_data = buf2[8];
						adcdata.adc_wheelh = buf2[12];
						adcdata.adc_wheell = buf2[13];
						adcdata.adc_gash = buf2[14];
						adcdata.adc_gasl = buf2[15];
						adcdata.adc_breakh = buf2[16];
						adcdata.adc_breakl = buf2[17];
						mutex_unlock(&outrun_mutex);
						get_data_once --;
						wait_received_data = 1;	
						wake_up_all(&wq);
					}else{
						printk("MODE_RETRIEV_12BITDATA error\n");
					}
					msleep(10);
				}		
			} else if(mode_flag == MODE_ERASE_DATA){
				if(get_data_once >0){					
					vfs_write(bte_fp2, bteErase, 4, &pos2);
					msleep(20);
					memset(buf2, 0, sizeof(buf2));
					len = vfs_read(bte_fp2, buf2, sizeof(buf2), &pos2);
					
					printk("Erase data len(%d): %d\n", get_data_once, len);
					for(i = 0; i < len; i ++){
						printk("buf[%02d]=%02x ", i, buf2[i]);						
					}
					printk("\n");
					
					if(len >= 16 && buf2[0] == 0xA7 && buf2[1] == 0x0E && buf2[2] == 0xF6 && buf2[3] == 0xFF){						
						erase_data_type = 1;
						get_data_once --;
						wait_received_data = 1;
						wake_up_all(&wq);
						printk("Erase Data success.\n");
					} else {
						printk("Erase Data fail.\n");
					}
					
				}
				msleep(10);
			} else if(mode_flag == MODE_WRITE_DATA){
				if(get_data_once > 0){
					vfs_write(bte_fp2, bteWrite, sizeof(bteWrite), &pos2);
					msleep(100);
					len = vfs_read(bte_fp2, buf2, sizeof(buf2), &pos2);
					printk("Wite Result data len(%d): %d\n", get_data_once, len);
					for(i = 0; i < len; i ++){
						printk("buf[%02d]=%02x ", i, buf2[i]);						
					}

					write_data_type = 1;
					wait_received_data = 1;
					wake_up_all(&wq);
					get_data_once --;
				}
				msleep(10);
			}

			
			vfs_write(bte_fp2, bte_buf_tx, 3, &pos2);//通知 mcu F104 可以開始送資料了
			msleep(10);
			memset(buf2, 0, sizeof(buf2));
			len = vfs_read(bte_fp2, buf2, sizeof(buf2), &pos2);
			if(len > 0 && buf2[3] == 0x05){
				//Todo:	
				if (buf2[3] == 0x05) {
					//printk("Wheel:[%02x], Accelerator:[%02x]\n", buf[4], buf[5]);
					vrdata.vr_wheel = buf2[4];
					vrdata.vr_gas = buf2[5] ^ 0xff;
					input_report_abs(input_js, ABS_X, wheel[vrdata.vr_wheel]);
					input_report_abs(input_js, ABS_RZ, vrdata.vr_gas);
				}

				if (buf2[7] == 0x15) {
					//printk("Breake:[%02x]\n", buf[8]);
					vrdata.vr_break = buf2[8] ^ 0xff;
					input_report_abs(input_js, ABS_Z, vrdata.vr_break);
				}
			}else{
				printk("wrong data: %02x\n", buf2[3]);
				msleep(10);
			}						
			
		}

		input_sync(bte_dev);
		input_sync(input_js);
		msleep(10);		
		//usleep_range(500, 1000);
	}//while

	printk("bte_thread stop\n");
	return 0;
}

long tty_ioctl(struct file *f, unsigned op, unsigned long param){
	if (f->f_op->unlocked_ioctl) {
		return f->f_op->unlocked_ioctl(f, op, param);
	}
	return 0;
}

int init_uart(char* dev, struct tty_struct **tty, struct file **fp){
	struct termios newtio;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	*fp = filp_open(dev, O_RDWR | O_NOCTTY, 0);
	if( IS_ERR(*fp) ){
		printk("Can not open serial port\n");
		return -1;
	}else printk("init_uart fp=[%p]\n", *fp);

	tty_ioctl(*fp, TCGETS, (unsigned long)&newtio);

	newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0; //inter-character timer unused
	newtio.c_cc[VMIN]  = 0; //0.5 seconds read timeout
	*tty = (struct tty_struct *)((struct file*)(*fp)->private_data);

	tty_ioctl(*fp, TCSETS, (unsigned long)&newtio);

	set_fs(oldfs);
	return 0;
}

int init_kb(void){
	int err = 0, i = 0;

	bte_dev = input_allocate_device();
	if(!bte_dev){
		printk("bte: not enought memory for input device\n");
		err = -ENOMEM;
		return err;
	}
	bte_dev->name = "bte-key";
	bte_dev->phys = "bte/input0";
	bte_dev->id.bustype = BUS_HOST;
	bte_dev->id.vendor = 0x0001;
	bte_dev->id.product = 0x0001;
	bte_dev->id.version = 0x0100;

	bte_dev->evbit[0] = (BIT_MASK(EV_KEY) | BIT_MASK(EV_MSC));
	bte_dev->mscbit[0] = (BIT_MASK(MSC_SCAN));

	for(i = 1; i <= 248; i++){ //you could check uapi/linux/input.h
		input_set_capability(bte_dev, EV_KEY, i);
	}

        err = input_register_device(bte_dev);
	if(err){
		input_free_device(bte_dev);
                printk("register bte-key failed\n");
                return -1;
        }

	return 0;
}

static int __init bte_uart_kb_init(void){
	int ret = 0;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
#if defined(GAME_OUTRUN)
	printk("define GAME_OUTRUN.........\n");
#endif		
	printk("bte_uart_kb_init %s.....\n", BTE_UART_KB_TTY);

	ret = init_uart(BTE_UART_KB_TTY, &local_tty, &bte_fp);
	if(ret < 0){
		printk("Can not initialize %s device\n", BTE_UART_KB_TTY);
		return 0;
	}else{
		printk("init [%s] uart successed[%p]\n", BTE_UART_KB_TTY, bte_fp);
	}
#if defined(GAME_STARWARS) || defined(GAME_TMNT) || defined(GAME_NBA) || defined(GAME_GOLDENAXE) || defined(GAME_PONG) || defined(GAME_OUTRUN)
	printk("bte_uart_kb_init %s\n", BTE_UART_KB_TTY2);
	ret = init_uart(BTE_UART_KB_TTY2, &local_tty2, &bte_fp2);
	if(ret < 0){
		printk("Can not initialize %s device\n", BTE_UART_KB_TTY2);
	}
	#if defined(GAME_TMNT) || defined(GAME_NBA) || defined(GAME_GOLDENAXE) || defined(GAME_PONG)
	else{
		printk("init [%s] uart successed[%p]\n", BTE_UART_KB_TTY2, bte_fp2);
	}
	#endif
	#if defined(GAME_OUTRUN)
	else {
		printk("init [%s] uart successed[%p]\n", BTE_UART_KB_TTY2, bte_fp2);
		input_js = input_allocate_device();
		input_js->name = "bte_controller";
		input_js->phys = "bte_controller/js0";
		input_js->id.bustype = BUS_HOST;
		input_js->id.vendor = 0x0001;
		input_js->id.product = 0x0001;
		input_js->id.version = 0x0001;

		input_js->evbit[0] = BIT(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		__set_bit(ABS_X, input_js->absbit);
		__set_bit(ABS_Y, input_js->absbit);
		__set_bit(ABS_Z, input_js->absbit);
		__set_bit(ABS_RX, input_js->absbit);
		__set_bit(ABS_RY, input_js->absbit);
		__set_bit(ABS_RZ, input_js->absbit);
		__set_bit(ABS_HAT0X, input_js->absbit);
		input_set_abs_params(input_js, ABS_X, -32768, 32767, 0, 0);
		input_set_abs_params(input_js, ABS_Y, -32768, 32767, 0, 0);
		input_set_abs_params(input_js, ABS_Z, 0, 255, 0, 0);
		input_set_abs_params(input_js, ABS_RX, -32768, 32767, 0, 0);
		input_set_abs_params(input_js, ABS_RY, -32768, 32767, 0, 0);
		input_set_abs_params(input_js, ABS_RZ, 0, 255, 0, 0);
		input_set_abs_params(input_js, ABS_HAT0X, -1, 1, 0, 0);
		input_set_abs_params(input_js, ABS_HAT0Y, -1, 1, 0, 0);
		
		input_set_capability(input_js, EV_KEY, BTN_A);
		input_set_capability(input_js, EV_KEY, BTN_B);
		input_set_capability(input_js, EV_KEY, BTN_X);
		input_set_capability(input_js, EV_KEY, BTN_Y);
		input_set_capability(input_js, EV_KEY, BTN_TL);
		input_set_capability(input_js, EV_KEY, BTN_TR);
		input_set_capability(input_js, EV_KEY, BTN_SELECT);
		input_set_capability(input_js, EV_KEY, BTN_START);
		input_set_capability(input_js, EV_KEY, BTN_MODE);
		input_set_capability(input_js, EV_KEY, BTN_THUMBL);
		input_set_capability(input_js, EV_KEY, BTN_THUMBR);
		input_register_device(input_js);
		mutex_init(&outrun_mutex);
		printk("outrun_ioctl_init...\n");
		outrun_ioctl_init();
	}
	#endif
	#if defined(GAME_STARWARS)
	else{
		printk("init [%s] uart successed[%p]\n", BTE_UART_KB_TTY2, bte_fp2);
		input_js = input_allocate_device();
		input_js->name = "bte_controller";
		input_js->phys = "bte_controller/js0";
		input_js->id.bustype = BUS_HOST;
		input_js->id.vendor = 0x0001;
		input_js->id.product = 0x0001;
		input_js->id.version = 0x0001;

		input_js->evbit[0] = BIT(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		input_set_capability(input_js, EV_KEY, BTN_0);
		input_set_capability(input_js, EV_KEY, BTN_1);
		input_set_capability(input_js, EV_KEY, BTN_2);
		input_set_capability(input_js, EV_KEY, BTN_3);
		__set_bit(ABS_X, input_js->absbit);
		__set_bit(ABS_Y, input_js->absbit);
		input_set_abs_params(input_js, ABS_X, 0, 255, 0, 128);
		input_set_abs_params(input_js, ABS_Y, 0, 255, 0, 128);
		input_register_device(input_js);
		//input_report_key(input_js, BTN_0, 0); input_sync(input_js);
		//input_report_abs(input_js, ABS_X, ++js_x); input_sync(input_js);
		mutex_init(&starwars_mutex);
		printk("starwars_ioctl_init...\n");
		starwars_ioctl_init();
	}
	#elif defined(GAME_NBA)
		mutex_init(&nba_mutex);
		nba_ioctl_init();
	#endif
#endif
	ret = init_kb();
	if(ret < 0){
		printk("Can not create bte keyboard\n");
		return 0;
	}else{
		printk("init bte keyboard successed\n");
	}

	bte_tsk = kthread_create(bte_thread, NULL, "bte_thread");
	sched_setscheduler(bte_tsk, SCHED_FIFO, &param);
	wake_up_process(bte_tsk);

	printk("[btejame]create a40i_vol_proc\n");
	proc_create("a40i_vol_proc", 0, NULL, &a40i_vol_proc_fops);/*a40i_vol_proc create*/

#if TESTGAMETYPE
	printk("[btejame]create a40i_gametype\n");
	proc_create("a40i_gametype", 0, NULL, &testgametype_fops);
#endif

#if TEST
	proc_create("a40i_vol_test", 0, NULL, &test_fops);
#endif

#if defined(GAME_STARWARS) && defined(JSTEST)
	proc_create("a40i_js_test", 0, NULL, &jstest_fops);
#endif

#if TESTMODE
	proc_create("a40i_testmode", 0, NULL, &a40i_testmode_proc_fops);
#endif

#if defined(GAME_NBA)
	proc_create("a40i_reboot", 0, NULL, &reboot_fops);
#endif
	proc_create("unlock", 0, NULL, &unlock_fops);
	return 0;
}

static void __exit bte_uart_kb_exit(void){
	kthread_stop(bte_tsk);

	if(bte_fp)
		filp_close(bte_fp, NULL);
#if defined(GAME_STARWARS)
	printk("starwars_ioctl_exit!\n");
	starwars_ioctl_exit();
	if(bte_fp2)
		filp_close(bte_fp2, NULL);
	if(input_js)
		input_free_device(input_js);
#elif defined(GAME_OUTRUN)
	printk("outrun_iotc_exit!\n");
	outrun_ioctl_exit();
	if(bte_fp2)
		filp_close(bte_fp2, NULL);
	if(input_js)
		input_free_device(input_js);
#elif defined(GAME_NBA)
	nba_ioctl_exit();
#endif

	input_unregister_device(bte_dev);
	input_free_device(bte_dev);
#if TESTGAMETYPE
	remove_proc_entry("a40i_gametype", NULL);
#endif
#if TEST
	remove_proc_entry("a40i_vol_test", NULL);
#endif
#if defined(GAME_STARWARS) && defined(JSTEST)
	remove_proc_entry("a40i_js_test", NULL);
#endif
#if TESTMODE
	remove_proc_entry("a40i_testmode", NULL);
#endif
#if defined(GAME_NBA)
	remove_proc_entry("a40i_reboot", NULL);
#endif
	remove_proc_entry("unlock", NULL);
	printk("bte_uart_kb_exit\n");
	return;
}

module_init(bte_uart_kb_init);
module_exit(bte_uart_kb_exit);
MODULE_AUTHOR("<stanley@bte.com.tw");
MODULE_DESCRIPTION("BTE UART Keyboard Driver");
MODULE_LICENSE("GPL");
