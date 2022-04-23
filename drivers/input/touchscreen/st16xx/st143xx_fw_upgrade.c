#include "sitronix_i2c_touch.h"
#include "st143xx_fw_upgrade.h"
extern struct sitronix_ts_data sitronix_ts_gpts;
#ifdef ST_UPGRADE_FIRMWARE
static int st_i2c_read_direct(st_u8 *rxbuf, int len)
{
	
	int ret = 0;	
	ret = i2c_master_recv(sitronix_ts_gpts.client, rxbuf, len);

	if (ret < 0){
		sterr("read direct error (%d)\n", ret);
		return ret;
	}
	usleep_range(50,100);
	return len;
}
static int st_check_chipid(void)
{
	int ret = 0;
	unsigned char buffer[1];

	ret = st_i2c_read_bytes(0xF4, buffer, 1);
	if (ret < 0){
		sterr("read status reg error (%d)\n", ret);
		return ret;
	}else{
		stinf("ChipID = %d\n", buffer[0]);
	}

#ifdef ST_IC_A1802
	if(buffer[0] != 0xC)
	{
		sterr("This IC is not A1802 , cancel upgrade\n");
		return -1;
	}	
#endif
	return 0;
}

static int st_get_device_status(void)
{
	int ret = 0;
	unsigned char buffer[8];

	ret = st_i2c_read_bytes(1, buffer, 8);
	if (ret < 0){
		sterr("read status reg error (%d)\n", ret);
		return ret;
	}else{
		stinf("status reg = %d\n", buffer[0]);
	}


	return buffer[0]&0xF;
}

static int st_check_device_status(int ck1,int ck2,int delay)
{
	int maxTimes = 3;
	int isInStauts = 0;
	int status = -1;
	while(maxTimes-->0 && isInStauts==0)
	{
		status = st_get_device_status();
		if(status == ck1 || status == ck2)
			isInStauts=1;
		msleep(delay);
	}
	if(isInStauts==0)
		return -1;
	else
		return 0;
}

static int st_power_up(void)
{
	unsigned char reset[2];
	reset[0] = 2;
	reset[1] = 0;
	return st_i2c_write_bytes(reset,2);
}


static int st_isp_on(void)
{
	unsigned char IspKey[] = {0,'S',0,'T',0,'X',0,'_',0,'F',0,'W',0,'U',0,'P'};
	unsigned char i;
	int icStatus = st_get_device_status();
	
	stinf("ISP on\n");		
		
	if(icStatus <0)
		return -1;
	if(icStatus == 0x6)
		return 0;
	else if(icStatus == 0x5)
		st_power_up();	

	for(i=0;i<sizeof(IspKey); i+=2)
	{
		if(st_i2c_write_bytes(&IspKey[i],2) < 0)
		{
			sterr("Entering ISP fail.\n");
			return -1;
		}
	}
	msleep(150);	//This delay is very important for ISP mode changing.	
					//Do not remove this delay arbitrarily.
	return st_check_device_status(6,99,10);
}

static int st_compare_array(unsigned char *b1,unsigned char *b2,int len)
{
	int i=0;
	for(i=0;i<len;i++)
	{		
		if(b1[i] != b2[i])
			return -1;
	}
	return 0;
}



#ifdef ST_FIREWARE_FILE
static int st_load_fw_from_file(unsigned st_char *buf)
{
	int i,j;
	struct st_file  *fw_fp;
	mm_segment_t fs;
	int fileSize = 0;

	for(j=0;j<10;j++)
	{
 		st_msleep(1000);
 		fileSize = 0;
		fw_fp = st_filp_open(ST_FW_PATH, O_RDWR,0600);
	 	if(IS_ERR(fw_fp)){
	        	sterr("Test: filp_open error!!. %d\n",j);
			fileSize = 0;	        	
	        }else
	        {
	        	fileSize = 0;
			fs = get_fs();
			set_fs(get_ds());
			//f->f_op->read(f,buf,ROM_SIZE,&f->f_pos);
			fileSize = fw_fp->f_op->read(fw_fp,buf,ST_FW_LEN,&fw_fp->f_pos);
			set_fs(fs);
	 		sterr("fw file size:0x%X\n",fileSize);
			//for(i=0;i<0x10;i++)
			//	stmsg("Test: data is %X",buf[i]);			
	          	st_filp_close(fw_fp,NULL);
			break;
	    	}
	}
	return fileSize;
}

static int st_load_cfg_from_file(unsigned st_char *buf)
{
	int j;
	struct file  *cfg_fp;
	mm_segment_t fs;
	int fileSize = 0;
		
	cfg_fp = filp_open(ST_CFG_PATH, O_RDWR,0644);
	if(IS_ERR(cfg_fp)){
	       	sterr("Test: filp_open error!!. %d\n",j);				        	
	}else
	{
		fs = get_fs();
		set_fs(get_ds());
			
		fileSize = cfg_fp->f_op->read(cfg_fp,buf,ST_CFG_LEN,&cfg_fp->f_pos);
		set_fs(fs);
		filp_close(cfg_fp,NULL);
	}
	return fileSize;
}

unsigned char fw_buf[ST_FW_LEN];
unsigned char cfg_buf[ST_CFG_LEN];
#else
unsigned char fw_buf[] = SITRONIX_FW;
unsigned char cfg_buf[] = SITRONIX_CFG;
#endif //ST_FIREWARE_FILE


#ifdef ST_IC_A1802

static int st_get_fw_info_offset(int fwSize,unsigned char *data)
{
	int cksOffset;
	int i=0;
	cksOffset = data[0x84] * 0x100 + data[0x85];
	
	
	for(i=cksOffset-4;i>=4;i--)
	{
		if(	data[i]   == 0x54 &&
			data[i+1] == 0x46 &&
			data[i+2] == 0x49 &&
			data[i+3] == 0x33 )
		{
			stmsg("TOUCH_FW_INFO offset = 0x%X\n",i+4);
			return i+4;
		}		
	}	

	sterr("can't find TOUCH_FW_INFO offset\n");
	return -1;
}


void ChecksumCalculation(unsigned short *pChecksum,unsigned char *pInData,unsigned long Len)
{
    unsigned long i;
    unsigned char LowByteChecksum;
    for(i = 0; i < Len; i++)
    {
        *pChecksum += (unsigned short)pInData[i];
        LowByteChecksum = (unsigned char)(*pChecksum & 0xFF);
        LowByteChecksum = (LowByteChecksum) >> 7 | (LowByteChecksum) << 1;
        *pChecksum = (*pChecksum & 0xFF00) | LowByteChecksum;
    }
}

static int st_v2_format(st_u16 addr,st_u8 isRead,unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	int result;
	
	unsigned char isp_tbuf[ST_ISP_MAX_WRITE_LEN+2] = {0};
	//unsigned st_u8 isp_rbuf[ST_ISP_MAX_TRANS_LEN+2] = {0};
	
	isp_tbuf[0] = (addr>>8);
	isp_tbuf[1] = (addr&0xFF);
	if(isRead)
	{	
		result = st_i2c_write_bytes(isp_tbuf, 2);
		
		//stmsg("Read off = %x %x ,len = %d, result = %d \n",isp_tbuf[0],isp_tbuf[1] ,len, result);
		usleep_range(50,100);
		result = st_i2c_read_direct(rxbuf,len);		
		//stmsg("Read data = %x %x %x %x, result = %d \n",rxbuf[0],rxbuf[1] , rxbuf[2] , rxbuf[3] , result);
		
		if( result < 0)
			return result;
		else
			return len;
	}
	else
	{
		memcpy(isp_tbuf+2,txbuf,len);
		result = st_i2c_write_bytes(isp_tbuf,len+2);
		//stmsg("Write off = %x %x ,len = %d, data = %x %x %x %x %x %x %x %x, result = %d \n",isp_tbuf[0],isp_tbuf[1] ,len, isp_tbuf[2],isp_tbuf[3],isp_tbuf[4],isp_tbuf[5],isp_tbuf[6],isp_tbuf[7],isp_tbuf[8],isp_tbuf[9],result);
		if( result < 0)
			return result;
		else
			return len;
	}
	return 0;		
}

static int st_v2_len_check(st_u16 addr,st_u8 isRead,unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	int nowlen = len;
	int nowoff = 0;
	int ret = 0;
	
	
	if(isRead)
	{
		while(nowlen > 0 )
		{
			if(nowlen > ST_ISP_MAX_TRANS_LEN)
			{
				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, ST_ISP_MAX_TRANS_LEN);
			
				nowoff += ST_ISP_MAX_TRANS_LEN;
				nowlen -= ST_ISP_MAX_TRANS_LEN;
			}
			else
			{			
				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, nowlen);
				nowlen = 0;
			}
		}
	}
	else
	{
		while(nowlen > 0 )
		{
			if(nowlen > ST_ISP_MAX_WRITE_LEN)
			{			
			
				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, ST_ISP_MAX_WRITE_LEN);
				nowoff += ST_ISP_MAX_WRITE_LEN;
				nowlen -= ST_ISP_MAX_WRITE_LEN;
			}
			else
			{			
				ret += st_v2_format(addr+nowoff,isRead,txbuf+nowoff, rxbuf+nowoff, nowlen);
				nowlen = 0;
			}
		}
	}
	
	return ret;
}

static int st_v2_read_bytes(st_u16 addr,unsigned char *rxbuf, int len)
{
	unsigned char tmp_buf[len];
	return st_v2_len_check(addr,1,tmp_buf, rxbuf, len);		
}

static int st_v2_write_bytes(st_u16 addr,unsigned char *txbuf, int len)
{
	unsigned char tmp_buf[len];
	return st_v2_len_check(addr,0,txbuf, tmp_buf, len);	
}

int st_v2_set_2B_mode(void)
{
	unsigned char data[2];
	int rt = 0;	
	data[0] = 0xF1;	
	data[1] = 0x20;	
	rt += st_i2c_write_bytes(data,2);
	if(rt < 0)
	{
		sterr("Set 2 byte mode error\n");
		return -1;
	}
	st_msleep(1);
	
	return 0;	
}

int st_v2_isp_off(void)
{
	unsigned char data[1];
	int rt = 0;	
	data[0] = 1;
	rt = st_v2_write_bytes(0x2,data,1);	
	//rt += st_i2c_write_bytes(data,2);
	if(rt < 0)
	{
		sterr("ISP off error\n");
		return -1;
	}
	st_msleep(300);
	

	return st_check_device_status(0,4,10);
}

static int st_v2_cmd_write(unsigned char *buf,int len,int delay)
{
	unsigned short pCheckSum=0; 
	unsigned char pStatus[8]={0};
	int retryCount=0;
	int isSuccess=0;

	ChecksumCalculation(&pCheckSum,buf,len);
	buf[len] = pCheckSum&0xFF;	//CheckSum
	//buf[len+1] = 0;
	
	if(st_v2_write_bytes(0xD0,buf,len+1) != len+1)
	{
		sterr("ST_SPIW_1 Send 0xD0 fail.\n");
		return -1;
	}
	
	st_msleep(1);
	pStatus[0] = 1;
	if(st_v2_write_bytes(0xF8,pStatus,1) != 1)
	{
		sterr("ST_SPIW_2 Send 0xF8 fail.\n");
		return -1;
	}

	st_msleep(delay);
	
	retryCount=0;
	isSuccess=0;
		
	while(isSuccess==0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if(st_v2_read_bytes(0xF8,pStatus,1) != 1)
		{
			stmsg("ST_SPIW_3 Read 0xF8  fail. retry %d\n",retryCount);
			isSuccess=0;
		}
		else
			isSuccess = 1;
			
			
		if(isSuccess == 1)
		{
			if(pStatus[0] != 0)
			{
				sterr("ST_SPIW_4 status of 0xF8 error ,error:%x.\n",pStatus[0]);
				isSuccess = 0;
				st_msleep(10);
			}
			else
				isSuccess = 1;
		}		
	}
	return isSuccess-1;
	
	return 0;
}

static int st_v2_flash_read_offset(unsigned char *Buf,unsigned short off,unsigned short len)
{
	unsigned char PacketData[8];
	st_u32 offset;	
	st_u32 readLen;	
	st_u32 nowLen;	
	
	offset = off;
	nowLen = len;
	while(nowLen > 0)
	{
		if( offset%ST_ISP_BLOCK_SIZE !=0)
			readLen = ST_ISP_BLOCK_SIZE - offset%ST_ISP_BLOCK_SIZE;
		else
			readLen = ST_ISP_BLOCK_SIZE;
		
		if(offset + readLen > 	off + len)
			readLen = off + len - offset;
			
		//stmsg("offset %x , readLen %x \n",offset,readLen);
		//read
		memset(PacketData,0,8);
		PacketData[0] = 0x15;
		PacketData[1] = 6;
		
		PacketData[2] = (offset>>16)&0xFF;
		PacketData[3] = (offset>>8)&0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = (readLen>>8)&0xff;
		PacketData[6] = (readLen) & 0xFF;	
		
		if(st_v2_cmd_write(PacketData,7,5) <0)
			return -1;		
		
		if(st_v2_read_bytes(0x200,Buf+(offset-off),readLen) != readLen)	
		{
			sterr("ST_FR_2 Read Flash Data fail.\n");
			return -1;
		}
		nowLen -= readLen;
		offset += readLen;
	}
	
	return 0;		
}

static int st_v2_flash_read_page(unsigned char *Buf,unsigned short PageNumber)
{
	unsigned char PacketData[8];	
	int i;
	st_u32 offset;
	
	for(i=0;i<ST_FLASH_PAGE_SIZE/ST_ISP_BLOCK_SIZE;i++)
	{
		memset(PacketData,0,8);
		PacketData[0] = 0x15;
		PacketData[1] = 6;
		
		offset = PageNumber*ST_FLASH_PAGE_SIZE+i*ST_ISP_BLOCK_SIZE;
		PacketData[2] = (offset>>16)&0xFF;
		PacketData[3] = (offset>>8)&0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = 0x1;
		PacketData[6] = 0x0;	
		
		if(st_v2_cmd_write(PacketData,7,5) <0)
			return -1;		
		
		if(st_v2_read_bytes(0x200,Buf+(i*ST_ISP_BLOCK_SIZE),ST_ISP_BLOCK_SIZE) != ST_ISP_BLOCK_SIZE)	
		{
			sterr("ST_FR_2 Read Flash Data fail.\n");
			return -1;
		}
	}
	
	//stmsg("flash %x %x %x %x\n",Buf[0x100],Buf[0x101],Buf[0x102],Buf[0x103]);
	return 0;
}

static int st_v2_flash_unlock(void)
{
	unsigned char PacketData[12];
	
	int retryCount=0;
	int isSuccess=0;
	
	PacketData[0] = 0x10;		
	PacketData[1] = 9;
	PacketData[2] = 0x55;
	PacketData[3] = 0xAA;
	PacketData[4] = 0x55;
	PacketData[5] = 0x6E;
	PacketData[6] = 0x4C;
	PacketData[7] = 0x7F;
	PacketData[8] = 0x83;
	PacketData[9] = 0x9B;
	
	while(isSuccess==0 && retryCount++ < ST_ISP_RETRY_MAX)
	{						
		if(st_v2_cmd_write(PacketData,10,3) ==0)
			isSuccess = 1;
				
		if(isSuccess ==0)
		{
			stinf("Read ISP_Unlock_Ready packet fail retry : %d\n",retryCount);
			//MSLEEP(30);			
		}
	}

	if(isSuccess == 0)
	{
		stmsg("st_flash_unlock fail.\n");
		return -1;
	}
	
	return 0;
}

int st_v2_flash_erase_page(unsigned short PageNumber)
{
	unsigned char PacketData[8];
	st_u32 offset;
	int retryCount=0;
	int isSuccess=0;
	
	PacketData[0] = 0x13;
	PacketData[1] = 4;
	offset = PageNumber*ST_FLASH_PAGE_SIZE;
	PacketData[2] = (offset>>16)&0xFF;
	PacketData[3] = (offset>>8)&0xFF;
	PacketData[4] = (offset)&0xFF;	
	
	while(isSuccess==0 && retryCount++ < ST_ISP_RETRY_MAX)
	{						
		if(st_v2_cmd_write(PacketData,5,40) ==0)
			isSuccess = 1;
				
		if(isSuccess ==0)
		{
			sterr("Read ISP_Erase_Ready packet fail with page %d retry : %d\n",PageNumber,retryCount);
			//MSLEEP(30);			
		}
	}

	if(isSuccess == 0)
	{
		sterr("st_flash_erase_page fail.\n");
		return -1;
	}
	
	return 0;	
}

static int st_v2_flash_write_page(unsigned char *Buf,unsigned short PageNumber)
{
	unsigned char PacketData[10];
	unsigned char RetryCount;
	unsigned short DataCheckSum=0;
	int i;
	st_u32 offset;
	
	for(i=0;i<ST_FLASH_PAGE_SIZE/ST_ISP_BLOCK_SIZE;i++)
	{
		if(st_v2_flash_unlock()<0)
			return -1;
		
		RetryCount = 0;
		PacketData[0] = 0x14;
		PacketData[1] = 7;
		
		offset = PageNumber*ST_FLASH_PAGE_SIZE+i*ST_ISP_BLOCK_SIZE;
		PacketData[2] = (offset>>16)&0xFF;
		PacketData[3] = (offset>>8)&0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = 0x1;
		PacketData[6] = 0x0;
		
		DataCheckSum=0;
		ChecksumCalculation(&DataCheckSum,Buf+(i*ST_ISP_BLOCK_SIZE),ST_ISP_BLOCK_SIZE);
		
		PacketData[7] = DataCheckSum;	
		
		if(st_v2_write_bytes(0x200,Buf+(i*ST_ISP_BLOCK_SIZE),ST_ISP_BLOCK_SIZE) != ST_ISP_BLOCK_SIZE)
		{
			sterr("ST_FW_1 Write Flash Data fail.\n");
			return -1;
		}
		//st_msleep(10);
		
		if(st_v2_cmd_write(PacketData,8,5) <0)
			return -1;
	}

	return 0;
}

int st_v2_flash_write(unsigned char *Buf, int Offset, int NumByte)
{	
	unsigned short StartPage;
	unsigned short PageOffset;
	int WriteNumByte;
	short WriteLength;
	//unsigned char TempBuf[ST_FLASH_PAGE_SIZE];
	unsigned char *TempBuf;
	int retry = 0;
	int isSuccess = 0;	
	
	TempBuf = kzalloc(ST_FLASH_PAGE_SIZE, GFP_KERNEL);
	stinf("Write flash offset:0x%X , length:0x%X\n",Offset,NumByte);
	
	WriteNumByte = 0;
	if(NumByte == 0)
		return WriteNumByte;
	
	if((Offset + NumByte) > ST_FLASH_SIZE)
		NumByte = ST_FLASH_SIZE - Offset;
		
	StartPage = Offset / ST_FLASH_PAGE_SIZE;
	PageOffset = Offset % ST_FLASH_PAGE_SIZE;
	while(NumByte > 0)
	{
		if((PageOffset != 0) || (NumByte < ST_FLASH_PAGE_SIZE))
		{
			if(st_v2_flash_read_page(TempBuf,StartPage) < 0)
				return -1;
		}

		WriteLength = ST_FLASH_PAGE_SIZE - PageOffset;
		if(NumByte < WriteLength)
			WriteLength = NumByte;
		memcpy(&TempBuf[PageOffset],Buf,WriteLength);
				
		retry = 0;
		isSuccess = 0;
		while(retry++ <2 && isSuccess ==0)
		{
			if(st_v2_flash_unlock() >= 0 &&  st_v2_flash_erase_page(StartPage) >= 0)
			{	
				stinf("write page:%d\n",StartPage);			
				if(st_v2_flash_write_page(TempBuf,StartPage) >= 0)
					isSuccess =1;
			}
//			isSuccess =1;
			
			if(isSuccess==0)
				stinf("FIOCTL_IspPageWrite write page %d retry: %d\n",StartPage,retry);
		}
		if(isSuccess==0)
		{
			sterr("FIOCTL_IspPageWrite write page %d error\n",StartPage);
			return -1;
		}
		else
			StartPage++;
		
		NumByte -= WriteLength;
		Buf += WriteLength;
		WriteNumByte += WriteLength;
		PageOffset = 0;
	}
	kfree(TempBuf);
	return WriteNumByte;
}

#endif

unsigned char fw_check[ST_FLASH_PAGE_SIZE];
#ifdef ST_UPGRADE_BY_ID
unsigned char id_buf[] = SITRONIX_IDS;
st_u16 id_off[] = SITRONIX_ID_OFF;

unsigned char fw_buf0[] = SITRONIX_FW;
unsigned char cfg_buf0[] = SITRONIX_CFG;
unsigned char fw_buf1[] = SITRONIX_FW1;
unsigned char cfg_buf1[] = SITRONIX_CFG1;
unsigned char fw_buf2[] = SITRONIX_FW2;
unsigned char cfg_buf2[] = SITRONIX_CFG2;

static void st_replace_fw_by_id(int id)
{
	if(id==0)
	{
		stmsg("Found id by SITRONIX_FW and SITRONIX_CFG\n");
		memcpy(fw_buf,fw_buf0,sizeof(fw_buf0));
		memcpy(cfg_buf,cfg_buf0,sizeof(cfg_buf0));
	}
	else if(id==1)
	{
		stmsg("Found id by SITRONIX_FW1 and SITRONIX_CFG1\n");
		memcpy(fw_buf,fw_buf1,sizeof(fw_buf1));
		memcpy(cfg_buf,cfg_buf1,sizeof(cfg_buf1));
	}
	else if(id==2)
	{
		stmsg("Found id by SITRONIX_FW2 and SITRONIX_CFG2\n");
		memcpy(fw_buf,fw_buf2,sizeof(fw_buf2));
		memcpy(cfg_buf,cfg_buf2,sizeof(cfg_buf2));
	}	
}

static int st_select_fw_by_id(void)
{	
	int ret=0;
	unsigned char buf[8];
	unsigned char id[4];
	int i=0;
	int idlen = sizeof(id_buf) / 4;
	int isFindID = 0;	
	int status = st_get_device_status();
	if(status < 0)
	{		
		return -1;
	}
	else if(status != 0x6)
	{		
		st_i2c_read_bytes(0xC, buf, 4);
		st_i2c_read_bytes(0xF1, buf+4, 1);
		buf[6] = buf[4];
		buf[5] =  (buf[4] &0xFC) | 1;
		buf[4] = 0xF1;
		st_i2c_write_bytes(buf+4, 2);
		st_msleep(1);
		//
		st_i2c_read_bytes(0xF1, id, 1);
		stmsg("customer bank: %x \n",id[0]);
		//
		
		st_i2c_read_bytes(0xC, id, 4);
		buf[5] = buf[6];
		st_i2c_write_bytes(buf+4, 2);
		st_msleep(1);
		//stmsg("customer ids: %x %x %x %x ,buf %x %x %x %x \n",id[0],id[1],id[2],id[3],buf[0],buf[1],buf[2],buf[3]);
		if(	id[0] == buf[0]
		   &&	id[1] == buf[1]
		   &&	id[2] == buf[2]
		   &&	id[3] == buf[3])
		 {
		 	sterr("read customer id fail \n");
		 	return -1;
		 }
		 else
		 	stmsg("customer ids: %x %x %x %x \n",id[0],id[1],id[2],id[3]);

		 for(i=0;i<idlen;i++)
		 {
		 	if(	id[0] == id_buf[i*4]
		   	&&	id[1] == id_buf[i*4+1]
		   	&&	id[2] == id_buf[i*4+2]
		   	&&	id[3] == id_buf[i*4+3])
		   	{
		   		isFindID =1;		   		
		   		st_replace_fw_by_id(i);		   		
		   	}
		 }
		 if(0== isFindID)
		 	return -1;
	}
	else
	{
		stinf("IC's status : boot code \n");
		// if bootcode
		
		if(0==st_v2_isp_off())
		{
			//could ispoff
			sterr("IC's could go normat status \n");
			return st_select_fw_by_id();
		}
		else
		{
			stinf("IC really bootcode mode \n");
			ret = st_v2_flash_read_offset(fw_check,ST_CFG_OFFSET,ST_CFG_LEN);			
			if(ret < 0 )
			{
				sterr("read CFG fail! (%x)\n",ret);
				return -1;
			}

			for(i=0;i<idlen;i++)
			{								
				stinf("customer ids: %x %x %x %x \n",fw_check[0+id_off[i]],fw_check[1+id_off[i]],fw_check[2+id_off[i]],fw_check[3+id_off[i]]);
				if(	fw_check[0+id_off[i]] == id_buf[i*4]
			   	&&	fw_check[1+id_off[i]] == id_buf[i*4+1]
			   	&&	fw_check[2+id_off[i]] == id_buf[i*4+2]
			   	&&	fw_check[3+id_off[i]] == id_buf[i*4+3])
			   	{
			   		isFindID =1;		   		
		   			st_replace_fw_by_id(i);	
			   	}
			}
			
			if(0== isFindID)
			{
				//read from page 0
				sterr("Find ID in CFG fail , try to find in page 0\n");
				
				ret = st_v2_flash_read_offset(fw_check,0,ST_CFG_LEN);
				
				
				if(ret < 0 )
				{
					sterr("read flash fail! (%x)\n",ret);
					return -1;
				}
				
				for(i=0;i<idlen;i++)
				{
					stinf("customer ids: %x %x %x %x \n",fw_check[0+id_off[i]],fw_check[1+id_off[i]],fw_check[2+id_off[i]],fw_check[3+id_off[i]]);
					if(	fw_check[0+id_off[i]] == id_buf[i*4]
				   	&&	fw_check[1+id_off[i]] == id_buf[i*4+1]
				   	&&	fw_check[2+id_off[i]] == id_buf[i*4+2]
				   	&&	fw_check[3+id_off[i]] == id_buf[i*4+3])
			   		{
				   		stinf("Find ID in page 0 ,customer ids: %x %x %x %x \n",id_buf[i*4],id_buf[i*4+1],id_buf[i*4+2],id_buf[i*4+3]);
				   		isFindID =1;		   		
			   			st_replace_fw_by_id(i);	
			   			
			   			return 1;
			   		}
				}
				if(0== isFindID)
					return -1;
			}
		}
	}
	return 0;
}
#endif //ST_UPGRADE_BY_ID

int st_upgrade_fw(void)
{
	int rt=0;
	int fwSize =0;
	int cfgSize =0;	
	int fwInfoOff = 0;
	int fwInfoLen = ST_FW_INFO_LEN;
	int powerfulWrite = 0;
#ifdef ST_UPGRADE_BY_ID
	int idStatus = 0;
#endif	
	int checkOff = 0;
#ifdef ST_FIREWARE_FILE	
	fwSize = st_load_fw_from_file(fw_buf);
	
	cfgSize = st_load_cfg_from_file(cfg_buf);	
	
#else

#ifdef ST_UPGRADE_BY_ID
	
	idStatus = st_select_fw_by_id();
	if(idStatus <0 )
	{
		sterr("find id fail , cancel upgrade\n");
		return -1;
	}	
#endif

	fwSize = sizeof(fw_buf);
	cfgSize = sizeof(cfg_buf);
	stinf("fwSize 0x%X,cfgsize 0x%X\n",fwSize,cfgSize);
#endif //ST_FIREWARE_FILE
		
	if(fwSize != 0)
	{
		fwInfoOff = st_get_fw_info_offset(fwSize,fw_buf);
		if(fwInfoOff <0)
			fwSize = 0;		
#ifdef ST_IC_A1802
//		fwInfoLen = fw_buf[fwInfoOff]*0x100 + fw_buf[fwInfoOff+1] +2+3;	
        fwInfoLen = *(fw_buf+fwInfoOff)*0x100 + fw_buf[fwInfoOff+1] +2+3;	
		if(fwInfoOff == -1)
		{
			sterr("check fwInfoOff Len error (%x), cancel upgrade\n",fwInfoOff);
			return -1;
		}
#endif
		stinf("fwInfoOff 0x%X , fwInfoLen 0x%x\n",fwInfoOff,fwInfoLen);
	}	
	
	cfgSize = min(cfgSize,ST_CFG_LEN);
	if(cfgSize != 0)
	{
#if defined(ST_IC_A1802)
		if(cfg_buf[0] != 0x43 || cfg_buf[1] != 0x46 || cfg_buf[2] != 0x54 || cfg_buf[3] != 0x31 )
#endif		
		{
			sterr("cfg_buf error (invalid CFG)\n");
			return -1;
		}
	}
	
	if(fwSize ==0 && cfgSize ==0)
	{
		sterr("can't find FW or CFG , cancel upgrade\n");
		return -1;
	}
	
	if(st_get_device_status() == 0x6)
    		powerfulWrite = 1;    	
    	
    	st_irq_off();
    	
    	rt = st_isp_on();
    	if(rt !=0)
    	{
    		sterr("ISP on fail\n");
    		goto ST_IRQ_ON;
    	}
    	
    	if(st_check_chipid() < 0)
    	{
    		sterr("Check ChipId fail\n");
    		rt = -1;
    		goto ST_ISP_OFF;
    	}
#if defined(ST_IC_A1802)	
    	st_v2_set_2B_mode();
#endif    	
	
    	if(powerfulWrite ==0 &&(fwSize !=0 || cfgSize!=0))
    	{
    		//check fw and cfg
    		checkOff = (fwInfoOff / ST_FLASH_PAGE_SIZE) *ST_FLASH_PAGE_SIZE;
#if defined(ST_IC_A1802)
		
		if(st_v2_flash_read_offset(fw_check,fwInfoOff,fwInfoLen) < 0)
		//if(st_v2_flash_read_page(fw_check,fwInfoOff / ST_FLASH_PAGE_SIZE)< 0 )
#endif
    		{
    			sterr("read flash fail , cancel upgrade\n");
    			rt = -1;
    			goto ST_ISP_OFF;
    		}
    		
    		if(fwSize !=0)
    		{    			
    			if(0 == st_compare_array(fw_check,fw_buf+fwInfoOff,fwInfoLen) )
    			{
    				stinf("fw compare :same\n");
    				fwSize = 0;
    			}
    			else
    			{
    				stinf("fw compare :different\n");
    			}
    			
    		}
    		
    		if(cfgSize !=0)
    		{
			st_v2_flash_read_offset(fw_check,ST_CFG_OFFSET,ST_CFG_LEN);
			
				if(0 == st_compare_array(fw_check,cfg_buf,cfgSize))
    			{
    				stinf("cfg compare :same\n");
    				cfgSize = 0;
    			}
    			else
    			{
    				stinf("cfg compare : different\n");		
#ifdef ST_UPGRADE_BY_ID
				
#endif    				
    			}
    			
    		}
    		
    	}


#if defined(ST_IC_A1802)

#ifdef ST_UPGRADE_BY_ID	
		
	if(cfgSize != 0 || fwSize !=0)
	{
		if(idStatus ==1 )
			st_v2_flash_write(cfg_buf,ST_CFG_OFFSET,cfgSize);
			
		//backup CFG in page 0
		memcpy(fw_check,fw_buf,ST_FLASH_PAGE_SIZE);
		memcpy(fw_check,cfg_buf,cfgSize);		
		st_v2_flash_write(fw_check,0,ST_FLASH_PAGE_SIZE);
	}
#endif	//ST_UPGRADE_BY_ID



	if(cfgSize !=0)
		st_v2_flash_write(cfg_buf,ST_CFG_OFFSET,cfgSize);

	if(fwSize !=0)
	{
		//write page 1 ~ N
		st_v2_flash_write(fw_buf+ST_FLASH_PAGE_SIZE,ST_FLASH_PAGE_SIZE,fwSize-ST_FLASH_PAGE_SIZE);				   
		//write page 0
		st_v2_flash_write(fw_buf,0,ST_FLASH_PAGE_SIZE);
	}

#ifdef ST_UPGRADE_BY_ID
	//restore FW in page 0
	if(cfgSize != 0 && fwSize ==0)
		st_v2_flash_write(fw_buf,0,ST_FLASH_PAGE_SIZE);
#endif	//ST_UPGRADE_BY_ID		
		
		
#endif
		
    	
ST_ISP_OFF:
#if defined(ST_IC_A1802)
    	rt = st_v2_isp_off();
#endif
ST_IRQ_ON:    	    		
    	st_irq_on();
    	
	if(cfgSize != 0 || fwSize != 0)
    		return 1;
    	else
    		return rt;
	
}

#endif //ST_UPGRADE_FIRMWARE
