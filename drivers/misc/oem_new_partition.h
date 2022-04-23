#ifndef __OEM_PARTITION_H_
#define __OEM_PARTITION_H_

#define CUSTOM_PRODUCT_INFO 8
#define CUSTOM_OFFSET 16
#define CUSTOM_PRODUCT_INFO_NUM 7
#define CUSTOM_PRODUCT_INFO_LEN 32
#define CUSTOM_PRODUCT_INFO_ALL_LEN (CUSTOM_PRODUCT_INFO_NUM * CUSTOM_PRODUCT_INFO_LEN)
#define PRODUCT_SN 0
#define PRODUCT_PSN 1
#define PRODUCT_MEID 2
#define PRODUCT_IMEI0 3
#define PRODUCT_IMEI1 4
#define PRODUCT_BT 5
#define PRODUCT_WLAN 6
#define BUILD_TYPE_INFO 13
#define BUILD_TYPE 0

#define WT_PROINFO_STRING_LEN 30
#define WT_PROINFO_IMEI_LEN 8
#define SIZE_4K 4096

#define SIZE_3K 3072
#define WT_PROINFO_APN_STRING_LEN 100
#define WT_PROINFO_OMADM_STRING_LEN 1000
#define WT_PROINFO_BASEBAND_LEN 100
#define WT_PROINFO_ANTITHEF_LEN 100

//define type
typedef enum{
    WT_PROINFO_oem_clear = 1,
    WT_PROINFO_color_id = 6,
    WT_PROINFO_sku_id = 7,
    WT_PROINFO_factoryreset_date = 12, //change by shuya,old value = 8
  WT_PROINFO_vzw_omadm = 12,
  WT_PROINFO_vzw_apn = 14,
  WT_PROINFO_baseband = 14,
  WT_PROINFO_antithef_googlefactoryreset = 17,
  WT_PROINFO_antithef_screenlock = 18,
  WT_PROINFO_antithef_googleaccount = 19,
    WT_PROINFO_build_type = BUILD_TYPE_INFO|( BUILD_TYPE <<CUSTOM_OFFSET),
    WT_PROINFO_ta_code = 16, 
    WT_PROINFO_block_fastboot_mode = 20,
    WT_PROINFO_block_factory_reset = 21,
    WT_PROINFO_custom_product_info = CUSTOM_PRODUCT_INFO,
    WT_PROINFO_sn = CUSTOM_PRODUCT_INFO|( PRODUCT_SN <<CUSTOM_OFFSET),
    WT_PROINFO_psn = CUSTOM_PRODUCT_INFO|( PRODUCT_PSN <<CUSTOM_OFFSET),
    WT_PROINFO_MEID = CUSTOM_PRODUCT_INFO|( PRODUCT_MEID <<CUSTOM_OFFSET),
    WT_PROINFO_IMEI0 = CUSTOM_PRODUCT_INFO|( PRODUCT_IMEI0 <<CUSTOM_OFFSET),
    WT_PROINFO_IMEI1 = CUSTOM_PRODUCT_INFO|( PRODUCT_IMEI1 <<CUSTOM_OFFSET),
    WT_PROINFO_bt = CUSTOM_PRODUCT_INFO|( PRODUCT_BT <<CUSTOM_OFFSET),
    WT_PROINFO_wlan = CUSTOM_PRODUCT_INFO|( PRODUCT_WLAN <<CUSTOM_OFFSET),
}wt_proinfo_type;


extern int emmc_partition_rw(const char *part_name, int write, loff_t offset,void *buffer, size_t len);
void oem_offset_get_new(wt_proinfo_type type, unsigned long long *offset1,unsigned long long *offset2);
int emmc_partition_write_oem_version2(unsigned int *value, unsigned long long offset1, unsigned long long offset2, int len);
int emmc_partition_read_oem_version2(unsigned int *value, unsigned long long offset1, unsigned long long offset2, int len);

#endif
