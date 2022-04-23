#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/genhd.h> 
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/delay.h>



extern char *saved_command_line;
extern int sec_schip_enabled (void);

static int board_id_proc_show(struct seq_file *m, void *v)
{
    char *arg_start = NULL;
    char board_id[25] = {0, };

    arg_start = strstr(saved_command_line, "androidboot.board_id=");
    if(NULL != arg_start) {
        arg_start += 21;  // skip "androidboot.board_id="
        strncpy(board_id, arg_start, 9);
        seq_printf(m, "%s\n", board_id);
    } else {
        seq_printf(m, "0x13100001");
    }

    return 0;
}







#define PROC_FOPS_RO(name)	\
	static int name##_proc_open(struct inode *inode, struct file *file)	\
	{									\
		return single_open(file, name##_proc_show, PDE_DATA(inode));	\
	}									\
	static const struct file_operations name##_proc_fops = {		\
		.owner          = THIS_MODULE,					\
		.open           = name##_proc_open,				\
		.read           = seq_read,					\
		.llseek         = seq_lseek,					\
		.release        = single_release,				\
	}

#define PROC_ENTRY(name) {__stringify(name), &name##_proc_fops}
PROC_FOPS_RO(board_id);

struct pentry {
	const char *name;
	const struct file_operations *fops;
};
const struct pentry lk_info_entries[] = {
	PROC_ENTRY(board_id),

};

static int __init proc_lk_info_init(void)
{
	struct proc_dir_entry *dir_entry = NULL;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(lk_info_entries); i++) {
		if (! proc_create(lk_info_entries[i].name, S_IRUGO, dir_entry, lk_info_entries[i].fops))
			pr_err("LK_INFO: Failed to create /proc/lk_info entry nodes\n");
	}

    return 0;
}
module_init(proc_lk_info_init);
