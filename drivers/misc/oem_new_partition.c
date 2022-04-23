#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/genhd.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/kdev_t.h>
#include <linux/string.h>
#include "oem_new_partition.h"

#define SECONDARD_SIZE_32 32
#define SECONDARD_SIZE_512 512
#define SECONDARD_SIZE_1024 1024
#define SECONDARD_SIZE_3072 (1024 * 3)
#define 	OEM_MIN(a,b)	((a) < (b) ? (a) : (b))
void oem_offset_get_new(wt_proinfo_type type, unsigned long long *offset1,unsigned long long *offset2)
{
	*offset2 = 0xffff;
        *offset1 = type & 0xffff;

	if(*offset1 == WT_PROINFO_custom_product_info || *offset1 == WT_PROINFO_build_type)
	{
		*offset2 = (type & (0xffff << CUSTOM_OFFSET))>>CUSTOM_OFFSET;
	}
	return;
}

static void mmc_get_secondary_patition_size(unsigned long long offset1, unsigned long long offset2,int *secondary_patition_size)
{

        if(offset1 == 8)
        {
                *secondary_patition_size = SECONDARD_SIZE_32;
        }
        else if(offset1 == 12)
        {
                if(offset2 == 0)
                        *secondary_patition_size = SECONDARD_SIZE_3072;
                else
                        *secondary_patition_size = SECONDARD_SIZE_1024;
        }
        else if(offset1 == 13)
        {
                *secondary_patition_size = SECONDARD_SIZE_32;
        }
        else if(offset1 == 14)
        {
                *secondary_patition_size = SECONDARD_SIZE_512;
        }
        else
        {
                *secondary_patition_size = SECONDARD_SIZE_1024;
        }

}


int emmc_partition_write_oem_version2(unsigned int *value, unsigned long long offset1, unsigned long long offset2, int len)
{
        static unsigned char tmp_buf[SIZE_4K] = {0};
        int secondary_patition_size;
        int ret = 0;

	if(offset2 == 0xffff)
	{
		return -1;
 	}
	
	ret = emmc_partition_rw("oem",  0, offset1 * SIZE_4K,(char *) tmp_buf, (unsigned long) SIZE_4K);
	mmc_get_secondary_patition_size(offset1, offset2,&secondary_patition_size);
	len = OEM_MIN(len, secondary_patition_size);
	if((offset2 + 1) * secondary_patition_size < SIZE_4K)
	{
		memcpy(&(tmp_buf[offset2 * secondary_patition_size]), value, len);
	}
	else
	{
		printk("ERROR: offset2 + 1 > SIZE_4K\n");
		return -1;
	}
	ret = emmc_partition_rw("oem", 1, offset1 * SIZE_4K, (char *) tmp_buf, (unsigned long) SIZE_4K);

	return ret;
}


int emmc_partition_read_oem_version2(unsigned int *value, unsigned long long offset1, unsigned long long offset2,int len)
{
	static unsigned char tmp_buf[SIZE_4K] = {0};
	int secondary_patition_size;
	int ret = 0;

	if(offset2 == 0xffff)
	{
		return -1;
 	}

	ret = emmc_partition_rw("oem",  0, offset1 * SIZE_4K,(char *) tmp_buf, (unsigned long) SIZE_4K);
	if (ret) {
		printk("ERROR: oem_repair_write_mmc\n");
		return ret;
	}

	mmc_get_secondary_patition_size(offset1, offset2, &secondary_patition_size);
	len = OEM_MIN(len, secondary_patition_size);
	if((offset2 + 1) * secondary_patition_size < SIZE_4K)
	{
		memcpy(value, &(tmp_buf[offset2 * secondary_patition_size]),len);
	}
	else
	{
		printk("ERROR: offset2 + 1 > SIZE_4K\n");
		return -1;
	}

	return ret;
}


