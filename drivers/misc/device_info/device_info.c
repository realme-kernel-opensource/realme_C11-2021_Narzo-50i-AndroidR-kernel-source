#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "../../../../fs/proc/internal.h"
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/device_info.h>

#define DEVINFO_NAME "devinfo"
#define INFO_BUF_LEN 64
/**for definfo log**/
#define log_fmt(fmt) "[line:%d][module:%s][%s] " fmt

#define DEVINFO_ERR(a,arg...) \
 do { \
		printk(KERN_NOTICE log_fmt(a),__LINE__,DEVINFO_NAME,__func__,##arg); \
  } while (0)

#define DEVINFO_MSG(a, arg...) \
 do { \
		printk(KERN_INFO log_fmt(a),__LINE__,DEVINFO_NAME,__func__,##arg); \
  } while (0)

/**definfo log end**/

int oplus_sdcard_gpio_value = -1;
int kboard_gpio_value = -1;

#if 0
extern pid_t fork_pid_child;
extern pid_t fork_pid_father;
extern int happend_times;
#endif
static char *devinfo_prj_name;
static char *devinfo_pcb_version;

static struct of_device_id devinfo_id[] = {
	{.compatible = "oplus, device_info",},
	{},
};

struct devinfo_data {
	struct platform_device *devinfo;
	struct pinctrl *pinctrl;
	int hw_id1_gpio;
	int hw_id2_gpio;
	int hw_id3_gpio;
	int sub_hw_id1;
	int sub_hw_id2;
	int main_hw_id5;
	int main_hw_id6;
	int sub_board_id;
	int sdcard_gpio;
	struct pinctrl_state *hw_sub_id_sleep;
	struct pinctrl_state *hw_sub_id_active;
	struct pinctrl_state *hw_main_id5_active;
	struct pinctrl_state *hw_main_id6_active;
	int ant_select_gpio;
	struct manufacture_info sub_mainboard_info;
};

static struct devinfo_data *dev_info;
static struct proc_dir_entry *parent = NULL;

static int __init op_prj_name_setup(char *str) {
	devinfo_prj_name = str;

	return 0;
}
 __setup("prj_name=", op_prj_name_setup);

static unsigned int atoi(const char *str)
{
	unsigned int value = 0;

	while (*str >= '0' && *str <= '9') {
		value *= 10;
		value += *str - '0';
		str++;
	}
	return value;
}

static int __init op_pcb_version_setup(char *str) {
	int temp_value;

	temp_value = atoi(str);
	switch (temp_value)
	{
		case 1:
			devinfo_pcb_version = "EVT0";
			break;
		case 2:
			devinfo_pcb_version = "EVT1";
			break;
		case 3:
			devinfo_pcb_version = "MP1";
			break;
		case 4:
			devinfo_pcb_version = "DVT1";
			break;
		case 5:
			devinfo_pcb_version = "DVT2";
			break;
		case 6:
			devinfo_pcb_version = "PVT";
			break;
		case 7:
			devinfo_pcb_version = "MP2";
			break;
		default:
			devinfo_pcb_version = "UNKOWN";
			return -1;
	}

	return 0;
}

 __setup("pcb_version=", op_pcb_version_setup);

static void *device_seq_start(struct seq_file *s, loff_t *pos)
{
	static unsigned long counter = 0;
	if ( *pos == 0 ) {
		return &counter;
	}else{
		*pos = 0;
		return NULL;
	}
}

static void *device_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void device_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static int device_seq_show(struct seq_file *s, void *v)
{
        struct proc_dir_entry *pde = s->private;
        struct manufacture_info *info = pde->data;
        if (info) {
                seq_printf(s, "Device version:\t\t%s\nDevice manufacture:\t\t%s\n",
                         info->version,        info->manufacture);
                if(info->fw_path)
                        seq_printf(s, "Device fw_path:\t\t%s\n",
                                info->fw_path);
		}
        return 0;
}

static struct seq_operations device_seq_ops = {
	.start = device_seq_start,
	.next = device_seq_next,
	.stop = device_seq_stop,
	.show = device_seq_show
};

static int device_proc_open(struct inode *inode,struct file *file)
{
	int ret = seq_open(file,&device_seq_ops);
	pr_err("%s is called\n",__func__);

	if(!ret){
		struct seq_file *sf = file->private_data;
		sf->private = PDE(inode);
	}

	return ret;
}
static const struct file_operations device_node_fops = {
	.read =  seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
	.open = device_proc_open,
	.owner = THIS_MODULE,
};

int register_device_proc(char *name, char *version, char *manufacture)
{
	struct proc_dir_entry *d_entry;
	struct manufacture_info *info;

	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	info = kzalloc(sizeof *info, GFP_KERNEL);
	info->version = version;
	info->manufacture = manufacture;
	d_entry = proc_create_data (name, S_IRUGO, parent, &device_node_fops, info);
	if(!d_entry) {
		DEVINFO_ERR("create %s proc failed.\n", name);
		kfree(info);
		return -ENOENT;
	}
	return 0;
}

int register_devinfo(char *name, struct manufacture_info *info)
{
	struct proc_dir_entry *d_entry;
	if(!parent) {
	parent =  proc_mkdir ("devinfo", NULL);
	if(!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	d_entry = proc_create_data (name, S_IRUGO, parent, &device_node_fops, info);
	if(!d_entry) {
		pr_err("create %s proc failed.\n", name);
		return -ENOENT;
	}
	return 0;
}

static int mainboard_init(void)
{
	return register_device_proc("mainboard", devinfo_pcb_version, devinfo_prj_name);
}


static ssize_t kboard_read_proc(struct file *file, char __user *buf,
                size_t count, loff_t *off)
{
	char page[256] = {0};
    int len = 0;
	int value = -1;
	value = gpio_get_value(kboard_gpio_value);
	if(value < 0)
		return 0;
	len = sprintf(page,"%d", value);
        if (len > *off) {
                len -= *off;
        }
        else{
                len = 0;
        }

        if (copy_to_user(buf, page, (len < count ? len : count))) {
                return -EFAULT;
        }
        *off += len < count ? len : count;
        return (len < count ? len : count);
}
struct file_operations kboard_proc_fops = {
        .read = kboard_read_proc,
};

static int subboard_init(struct devinfo_data *const devinfo_data) {
	int ret = 0;
	struct device_node *np = NULL;
	struct proc_dir_entry *pentry;
	np = devinfo_data->devinfo->dev.of_node;

	devinfo_data->sub_board_id = of_get_named_gpio(np, "sub_board-gpio", 0);
	if(devinfo_data->sub_board_id < 0 ) {
		DEVINFO_ERR("devinfo_data->sub_board_id not specified\n");
		return -1;
	}
	kboard_gpio_value = devinfo_data->sub_board_id;
	pentry = proc_create("kboard", 0664, NULL, &kboard_proc_fops);

	devinfo_data->sub_mainboard_info.version = kzalloc(INFO_BUF_LEN, GFP_KERNEL);
	if (devinfo_data->sub_mainboard_info.version == NULL) {
		return -1;
	}

	devinfo_data->sub_mainboard_info.manufacture = kzalloc(INFO_BUF_LEN, GFP_KERNEL);
	if (devinfo_data->sub_mainboard_info.manufacture == NULL) {
		kfree(devinfo_data->sub_mainboard_info.version);
		return -1;
	}

	snprintf(devinfo_data->sub_mainboard_info.version, INFO_BUF_LEN, "SPRD");
	ret = register_device_proc("audio_mainboard",
		devinfo_data->sub_mainboard_info.version,
		devinfo_data->sub_mainboard_info.manufacture);

	ret = register_device_proc("sub_mainboard",
		devinfo_data->sub_mainboard_info.version,
		devinfo_data->sub_mainboard_info.manufacture);
	return ret;
}

static int subboard_verify(struct devinfo_data *const devinfo_data)
{
	int ret = 0;
	int id = -1;

	DEVINFO_MSG("Enter\n");
	if(IS_ERR_OR_NULL(devinfo_data)){
		DEVINFO_ERR("devinfo_data is NULL\n");
		return -1;
	}

	if(devinfo_data->sub_board_id >= 0 ) {
			id = gpio_get_value(devinfo_data->sub_board_id);
	}
	DEVINFO_ERR("id= %d", id);
	if ((0 == id || 1 == id)) {
		snprintf(devinfo_data->sub_mainboard_info.manufacture, INFO_BUF_LEN, "sub-match");
	}
	else {
		snprintf(devinfo_data->sub_mainboard_info.manufacture, INFO_BUF_LEN, "sub-unmatch");
	}

	return ret;
}

static ssize_t sdcard_read_proc(struct file *file, char __user *buf,
                size_t count, loff_t *off)
{
	char page[256] = {0};
    int len = 0;
	int value = -1;
	value = gpio_get_value(oplus_sdcard_gpio_value);
	if(value < 0)
		return 0;
	len = sprintf(page,"%d", value);
        if (len > *off) {
                len -= *off;
        }
        else{
                len = 0;
        }

        if (copy_to_user(buf, page, (len < count ? len : count))) {
                return -EFAULT;
        }

        *off += len < count ? len : count;
        return (len < count ? len : count);
}

struct file_operations sdcard_proc_fops = {
        .read = sdcard_read_proc,
};
static int sdcard_init(struct devinfo_data *const devinfo_data) {
	struct proc_dir_entry *pentry;
	struct device_node *np = NULL;
	np = devinfo_data->devinfo->dev.of_node;
	devinfo_data->sdcard_gpio= of_get_named_gpio(np, "sdcard-gpio", 0);
	if(devinfo_data->sdcard_gpio < 0 ) {
		DEVINFO_ERR("devinfo_data->sdcard_gpio not specified\n");
		return -1;
	}

	oplus_sdcard_gpio_value = devinfo_data->sdcard_gpio;
	pentry = proc_create("sdcard_detect", 0664, NULL, &sdcard_proc_fops);
	if(!pentry) {
		pr_err("create sdcard_detect proc failed.\n");
		return -ENOENT;
	}
	return 0;
}

static ssize_t devinfo_modify_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos){
	char proc_devinfo_modify_data[28];
	DEVINFO_ERR("call\n");
	if (copy_from_user(&proc_devinfo_modify_data, buff, count)){
		DEVINFO_ERR("error.\n");
		return count;
	}
	if (strncmp(proc_devinfo_modify_data, "1", 1) == 0){
		DEVINFO_ERR("subboard need to check again.\n");
		subboard_verify(dev_info);
	} else {
		DEVINFO_ERR("not support \n");
		return count;
	}

	return count;
}

static const struct file_operations devinfo_modify_fops = {
	.write = devinfo_modify_write,
	.llseek = noop_llseek,
};

#if 0
static ssize_t fork_para_monitor_read_proc(struct file *file, char __user *buf,
                size_t count, loff_t *off)
{
        char page[256] = {0};
        int ret = 0;
        ret = snprintf(page, 255, " times:%d\n father pid:%d\n child pid:%d\n", happend_times, fork_pid_father, fork_pid_child);

        ret = simple_read_from_buffer(buf, count, off, page, strlen(page));
        return ret;
}

struct file_operations fork_para_monitor_proc_fops = {
        .read = fork_para_monitor_read_proc,
        .write = NULL,
};

static void recursive_fork_para_monitor(void)
{
		struct proc_dir_entry *pentry;

		pentry = proc_create("fork_monitor", S_IRUGO, parent, &fork_para_monitor_proc_fops);
        if (!pentry) {
                pr_err("create /devinfo/fork_monitor proc failed.\n");
        }
}
#endif
static void init_proc_devinfo_modify(void){
	struct proc_dir_entry *p = NULL;
	p = proc_create("devinfo_modify", S_IWUSR | S_IWGRP | S_IWOTH, NULL, &devinfo_modify_fops);
	if(!p)
		DEVINFO_ERR("proc_create devinfo_modify_fops fail!\n");
}

static int devinfo_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct devinfo_data * const devinfo_data = devm_kzalloc(&pdev->dev,sizeof(struct devinfo_data), GFP_KERNEL);

	if( IS_ERR_OR_NULL(devinfo_data) ) {
		DEVINFO_ERR("devinfo_data kzalloc failed\n");
		ret = -ENOMEM;
		return ret;
	}

	devinfo_data->devinfo = pdev;
	dev_info = devinfo_data;

	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			DEVINFO_ERR("can't create devinfo proc\n");
			ret = -ENOENT;
		}
	}

	ret = subboard_init(devinfo_data);
	if (ret < 0) {
		DEVINFO_ERR("register audio mainboard failed\n");
	} else {
		subboard_verify(devinfo_data);
	}

	ret = mainboard_init();
	if (ret < 0) {
		DEVINFO_ERR("register mainboard failed\n");
	}
#if 0
	recursive_fork_para_monitor();
#endif

	ret = sdcard_init(devinfo_data);
	if (ret < 0) {
		DEVINFO_ERR("register sdcard failed\n");
	}
	init_proc_devinfo_modify();
	return ret;
}

static int devinfo_remove(struct platform_device *dev)
{
	remove_proc_entry(DEVINFO_NAME, NULL);
	return 0;
}

static struct platform_driver devinfo_platform_driver = {
	.probe = devinfo_probe,
	.remove = devinfo_remove,
	.driver = {
		.name = DEVINFO_NAME,
		.of_match_table = devinfo_id,
	},
};

module_platform_driver(devinfo_platform_driver);

MODULE_DESCRIPTION("OPPO device info");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("lzt<linzhengtao@vanyol.com>");
