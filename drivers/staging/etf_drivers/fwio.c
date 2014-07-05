/*
 * Firmware I/O code for mac80211 ST-Ericsson CW1200 drivers
 *
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * Based on:
 * ST-Ericsson UMAC CW1200 driver which is
 * Copyright (c) 2010, ST-Ericsson
 * Author: Ajitpal Singh <ajitpal.singh@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include "cw1200_common.h"

#include "debug.h"
//#include "cw1200.h"
#include "fwio.h"
#include "hwio.h"
#include "sbus.h"
//#include "bh.h"

int cw1200_load_firmware(struct CW1200_priv *priv);

static int cw1200_get_hw_type(u32 config_reg_val, int *major_revision)
{
	int hw_type = -1;
	u32 config_value = config_reg_val;
	u32 silicon_type = (config_reg_val >> 24) & 0x3;
	u32 silicon_vers = (config_reg_val >> 31) & 0x1;


	/* Check if we have CW1200 or STLC9000 */
	if ((silicon_type == 0x1) || (silicon_type == 0x2)) {
		*major_revision = silicon_type;
		if (silicon_vers)
			hw_type = HIF_8601_VERSATILE;
		else
			hw_type = HIF_8601_SILICON;

	} else if (((config_value >> 24) & 0x4) == 0x4) {
		*major_revision = 0x4;
		if (silicon_vers)
			hw_type = HIF_8601_VERSATILE;
		else
			hw_type = HIF_8601_SILICON;

	} else {
		*major_revision = 1;
		hw_type = HIF_9000_SILICON_VERSTAILE;
	}

	return hw_type;
}

static int config_reg_read_stlc9000(struct CW1200_priv *priv,
				    u16 reg, u32 *val)
{
	u16 val16;
	int ret = cw1200_reg_read_16(priv, reg, &val16);
	if (ret < 0)
		return ret;
	*val = val16;
	return 0;
}

static int config_reg_write_stlc9000(struct CW1200_priv *priv,
				     u16 reg, u32 val)
{
	return cw1200_reg_write_16(priv, reg, (u16)val);
}

static int cw1200_load_firmware_cw1200(struct CW1200_priv *priv);


static int cw1200_load_firmware_generic(struct CW1200_priv *priv, const struct firmware *fw)
{

	int ret = ERROR_SDIO;
	int block, num_blocks;
	unsigned i;
	u32 val32;
	u32 put = 0, get = 0;
	u8 *buf = NULL;
	uint32_t reg_value = 0;

	/* Macroses are local. */
#define APB_WRITE(reg, val) \
	do { \
		ret = cw1200_apb_write_32(priv, CW12000_APB(reg), (val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, \
				"%s: can't write %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
#define APB_READ(reg, val) \
	do { \
		ret = cw1200_apb_read_32(priv, CW12000_APB(reg), &(val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, \
				"%s: can't read %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
#define REG_WRITE(reg, val) \
	do { \
		ret = cw1200_reg_write_32(priv, (reg), (val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, \
				"%s: can't write %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
#define REG_READ(reg, val) \
	do { \
		ret = cw1200_reg_read_32(priv, (reg), &(val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, \
				"%s: can't read %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
	/* Initialize common registers */
	//APB_WRITE(DOWNLOAD_IMAGE_SIZE_REG, DOWNLOAD_ARE_YOU_HERE);
	reg_value = DOWNLOAD_ARE_YOU_HERE;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON +
					DOWNLOAD_IMAGE_SIZE_REG,
					(uint8_t *)&reg_value, BIT_32_REG)) {
		DEBUG(DBG_ERROR, "%s:SBUS_SramWrite_APB()returned error \n",
			__func__);
		goto error;
	}

	//APB_WRITE(DOWNLOAD_PUT_REG, 0);
	reg_value = 0;
		if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_PUT_REG,
					(uint8_t *)&reg_value, sizeof(uint32_t))) {
			DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Put failed \n",
					__func__);
			goto error;
		}

	//APB_WRITE(DOWNLOAD_GET_REG, 0);
	reg_value = 0;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_GET_REG, (uint8_t *)&reg_value,
				sizeof(uint32_t))) {
		DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Get failed \n",
				__func__);
		goto error;
	}

	//APB_WRITE(DOWNLOAD_STATUS_REG, DOWNLOAD_PENDING);
	reg_value = DOWNLOAD_PENDING;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_STATUS_REG,
				(uint8_t *)&reg_value, sizeof(uint32_t))) {
		DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Status Reg"
				" failed \n", __func__);
		goto error;
	}

	//APB_WRITE(DOWNLOAD_FLAGS_REG, 1);
	reg_value = 1;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_FLAGS_REG,
				(uint8_t *)&reg_value, sizeof(uint32_t))) {
		DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Flag Reg failed \n",
				__func__);
		goto error;
	}

	
	buf = kmalloc(DOWNLOAD_BLOCK_SIZE, GFP_KERNEL | GFP_DMA);
	if (!buf) {
		DEBUG(DBG_ERROR, "%s: can't allocate bootloader buffer.\n", __func__);
		ret = -ENOMEM;
		goto error;
	}
	/* Check if the FPGA bootloader is ready */
	for (i = 0; i < 200; i += 1 + i / 2) {
		//APB_READ(DOWNLOAD_IMAGE_SIZE_REG, val32);
		if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_IMAGE_SIZE_REG,
					(uint8_t *)&val32, BIT_32_REG))
			goto error;
		if (val32 == DOWNLOAD_I_AM_HERE)
			break;
		//mdelay(i);
		mdelay(10);
	} /* End of for loop */

	if (val32 != DOWNLOAD_I_AM_HERE) {
		DEBUG(DBG_ERROR, "%s: bootloader is not ready.\n", __func__);
		ret = -ETIMEDOUT;
		goto error;
	}
	/* Calculcate number of download blocks */
	num_blocks = (fw->size - 1) / DOWNLOAD_BLOCK_SIZE + 1;

	/* Updating the length in Download Ctrl Area */
	val32 = fw->size; /* Explicit cast from size_t to u32 */

	//APB_WRITE(DOWNLOAD_IMAGE_SIZE_REG, val32);
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON +
					DOWNLOAD_IMAGE_SIZE_REG,
					(uint8_t *)&val32, BIT_32_REG)) {
			DEBUG(DBG_ERROR, "%s:SBUS_SramWrite_APB()returned error \n",
				__func__);
			goto error;
		}

	/* Bootloader downloading loop */
	printk(KERN_ERR "%s: NUM_BLOCKS = %d\n",__func__,num_blocks);
	for (block = 0; block < num_blocks ; block++) {
		size_t tx_size;
		size_t block_size;

		/* check the download status */
		//APB_READ(DOWNLOAD_STATUS_REG, val32);
		if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
						+ DOWNLOAD_STATUS_REG,
						(uint8_t *)&val32, BIT_32_REG)) {
			DEBUG(DBG_ERROR,
			"%s:SBUS_SramRead_APB()returned error \n",
					__func__);
			goto error;
		}
		if (val32 != DOWNLOAD_PENDING) {
			DEBUG(DBG_ERROR, "%s: bootloader reported error %d.\n",
				__func__, val32);
			ret = -EIO;
			goto error;
		}

		/* loop until put - get <= 24K */
		for (i = 0; i < 100; i++) {
			//APB_READ(DOWNLOAD_GET_REG, get);
			if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
						+ DOWNLOAD_GET_REG,
						(uint8_t *)&get,
						BIT_32_REG)) {
				DEBUG(DBG_ERROR, "Sram Read get failed \n");
				goto error;
			}

			if ((put - get) <=
			    (DOWNLOAD_FIFO_SIZE - DOWNLOAD_BLOCK_SIZE))
				break;
			//mdelay(i);
			mdelay(150);
		}

		if ((put - get) > (DOWNLOAD_FIFO_SIZE - DOWNLOAD_BLOCK_SIZE)) {
			DEBUG(DBG_ERROR, "%s: Timeout waiting for FIFO.\n",
				__func__);
		printk(KERN_ERR "%s:LOOP COUNT = %d\n",__func__,block);
			return -ETIMEDOUT;
		}

		/* calculate the block size */
		tx_size = block_size = min((size_t)(fw->size - put),
			(size_t)DOWNLOAD_BLOCK_SIZE);

		memcpy(buf, &fw->data[put], block_size);
		if (block_size < DOWNLOAD_BLOCK_SIZE) {
			memset(&buf[block_size],
				0, DOWNLOAD_BLOCK_SIZE - block_size);
			tx_size = DOWNLOAD_BLOCK_SIZE;
		}

		/* send the block to sram */
		/*ret = cw1200_apb_write(priv,
			CW12000_APB(DOWNLOAD_FIFO_OFFSET +
				(put & (DOWNLOAD_FIFO_SIZE - 1))),
			buf, tx_size);*/

		if (SBUS_SramWrite_APB(priv,
					PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_FIFO_OFFSET
					+ (put & (DOWNLOAD_FIFO_SIZE - 1)),
					buf, tx_size)) {
			DEBUG(DBG_ERROR, "%s:SRAM Write Error \n", __func__);
			goto error;
		}
		if (ret < 0) {
			DEBUG(DBG_ERROR, "%s: can't write block at line %d.\n",
				__func__, __LINE__);
			printk(KERN_ERR "%s:LOOP COUNT = %d\n",__func__,block);
			goto error;
		}

		/* update the put register */
		put += block_size;
		//APB_WRITE(DOWNLOAD_PUT_REG, put);
		if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_PUT_REG,
					(uint8_t *)&put, BIT_32_REG)) {
			DEBUG(DBG_ERROR, "%s: Sram Write update put failed\n",
					__func__);
			goto error;
		}
	} /* End of bootloader download loop */
	
	/* Wait for the download completion */
	for (i = 0; i < 600; i += 1 + i / 2) {
		//APB_READ(DOWNLOAD_STATUS_REG, val32);
		if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_STATUS_REG,
				(uint8_t *)&val32, BIT_32_REG)) {
		DEBUG(DBG_ERROR, "%s: Sram Read failed \n", __func__);
		goto error;
	}
		if (val32 != DOWNLOAD_PENDING)
			break;
		mdelay(i);
	}
	if (val32 != DOWNLOAD_SUCCESS) {
		DEBUG(DBG_ERROR, "%s: wait for download completion failed. " \
			"Read: 0x%.8X\n", __func__, val32);
		ret = -ETIMEDOUT;
		goto error;
	} else {
		DEBUG(DBG_ERROR, "Bin file download completed.\n");
		ret = 0;
		goto error;
	}
error:
	kfree(buf);	
	return ret;
#undef APB_WRITE
#undef APB_READ
#undef REG_WRITE
#undef REG_READ
}

int cw1200_load_firmware_cw1260_fpga(struct CW1200_priv *priv)
{

	int ret, ret_1;
	u32 val32_1;
	const char *fw_path;
	const char *bl_path;
	const struct firmware *firmware = NULL;
	const struct firmware *bootloader = NULL;
	//const struct firmware *bootloader_readback = NULL;
	//u32 *bootloader_readback_data = NULL;



	fw_path = FIRMWARE_1260_CUT10;
	bl_path = BOOTLOADER_FILE_1260;
	

      ret_1 = cw1200_reg_read_32(priv, ST90TDS_CONFIG_REG_ID, &val32_1);
       if (ret_1 < 0) {
		   DEBUG(DBG_ERROR, "%s: cant read BIT 15 of Config Reg.\n", __func__);
               //goto out;
        }
        else  {
               ret_1 = cw1200_reg_write_16(priv, ST90TDS_CONFIG_REG_ID,
                               val32_1 | 1<<15);
               if (ret_1 < 0) {
				   DEBUG(DBG_ERROR, "%s: Can't set valure of BIT 15 of Config Reg\n", __func__);
               //goto unsubscribe;
               }
    	}

	/*Load Bootloader File*/
	printk(KERN_ERR "%s: BL FILE = %s\n",__func__,bl_path);
	
	ret = request_firmware(&bootloader, bl_path, priv->pdev);
	if (ret) {
		DEBUG(DBG_ERROR, "%s: can't load bootloader file %s.\n",
			__func__, bl_path);
		goto error;
	}
	BUG_ON(!bootloader->data);
	ret = cw1200_load_firmware_generic(priv, bootloader);
	
	if(ret)
		goto error;
	printk(KERN_ERR "%s: BOOTLOADER DOWNLOAD SUCCESS\n",__func__);

	printk(KERN_ERR "%s:FW FILE = %s\n",__func__,fw_path);
	ret = request_firmware(&firmware, fw_path, priv->pdev);
	if (ret) {
		DEBUG(DBG_ERROR, "%s: can't load firmware file %s.\n",
			__func__, fw_path);
		goto error;
	}
	BUG_ON(!firmware->data);
	ret = cw1200_load_firmware_generic(priv, firmware);
	if(ret)
		goto error;
	printk(KERN_ERR "%s: FIRMWARE DOWNLOAD SUCCESS\n",__func__);
	
error:
//	kfree(buf);
	if(bootloader)
		release_firmware(bootloader);
	if (firmware)
		release_firmware(firmware);
	return ret;
}


int cw1200_load_firmware_cw1260(struct CW1200_priv *priv)
{
	int ret = ERROR_SDIO;
	const char *bl_path = NULL;
	u32 val32;
	const struct firmware *bootloader = NULL;
	//uint32_t reg_value = 0;
	int i;
	u32 addr = 0x08000000;
	u32 *data;

	bl_path = BOOTLOADER_FILE_1260;

#define AHB_WRITE(reg, val) \
	do { \
		ret = cw1200_ahb_write_32(priv, reg, (val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, "%s: can't write %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)

        /* Macroses are local. */
#define APB_WRITE(reg, val) \
        do { \
                ret = cw1200_apb_write_32(priv, CW12000_APB(reg), (val)); \
                if (ret < 0) { \
					DEBUG(DBG_ERROR, "%s: can't write %s at line %d.\n", \
                                __func__, #reg, __LINE__); \
                        goto error; \
                } \
        } while (0)
#define APB_READ(reg, val) \
        do { \
                ret = cw1200_apb_read_32(priv, CW12000_APB(reg), &(val)); \
                if (ret < 0) { \
					DEBUG(DBG_ERROR, "%s: can't read %s at line %d.\n", \
                                __func__, #reg, __LINE__); \
                        goto error; \
                } \
        } while (0)
#define REG_WRITE(reg, val) \
        do { \
                ret = cw1200_reg_write_32(priv, (reg), (val)); \
                if (ret < 0) { \
					DEBUG(DBG_ERROR, "%s: can't write %s at line %d.\n", \
                                __func__, #reg, __LINE__); \
                        goto error; \
                } \
        } while (0)
#define REG_READ(reg, val) \
        do { \
                ret = cw1200_reg_read_32(priv, (reg), &(val)); \
                if (ret < 0) { \
				DEBUG(DBG_ERROR, "%s: can't read %s at line %d.\n", \
                                __func__, #reg, __LINE__); \
                        goto error; \
                } \
        } while (0)

	/* Enable Clock */
	//REG_READ(ST90TDS_CONFIG_REG_ID, val32);
        //val32 &= ~ST90TDS_CONFIG_CPU_CLK_DIS_BIT;
        //REG_WRITE(ST90TDS_CONFIG_REG_ID, val32);

        /* Load a bootloader file */
        ret = request_firmware(&bootloader, bl_path, priv->pdev);
        if (ret) {
		DEBUG(DBG_ERROR, "%s: can't load bootloader file %s.\n",
                        __func__, bl_path);
                goto error;
        }
        BUG_ON(!bootloader->data);

	data = (u32 *)bootloader->data;

	for(i = 0; i < (bootloader->size)/4; i++) {
		REG_WRITE(ST90TDS_SRAM_BASE_ADDR_REG_ID, addr);
		/*if (SBUS_SDIO_RW_Reg(priv, ST90TDS_SRAM_BASE_ADDR_REG_ID,
			(uint8_t *)&addr, BIT_32_REG, SDIO_WRITE)) {
		DEBUG(DBG_ERROR, "%s:ERROR:SRAM Base Address Reg Failed \n",
			__func__);
		goto error;
	}
		if (SBUS_SDIO_RW_Reg(priv, ST90TDS_AHB_DPORT_REG_ID,
			(uint8_t *)&data[i], BIT_32_REG, SDIO_WRITE)) {
		DEBUG(DBG_ERROR, "%s:ERROR:NOP Write Failed \n", __func__);
		goto error;
	}*/
		REG_WRITE(ST90TDS_AHB_DPORT_REG_ID,data[i]);
		
		//if(i == 100 || i == 200 || i == 300 || i == 400 || i == 500 || i == 600 ) 
			
			REG_READ(ST90TDS_AHB_DPORT_REG_ID,val32);
			
		addr += 4;
	}

	printk(KERN_ERR "%s:WRITE COMPLETE\n",__func__);

	ret = cw1200_load_firmware_cw1200(priv);
error:	
	return ret;
	
}


static int cw1200_load_firmware_cw1200(struct CW1200_priv *priv)
{
	int ret = ERROR_SDIO, block, num_blocks;
	unsigned i;
	u32 val32;
	u32 put = 0, get = 0;
	u8 *buf = NULL;
	const char *fw_path;
	const char *bl_path = NULL;
	const struct firmware *firmware = NULL;
	fw_path = FIRMWARE_1260_CUT10;
	bl_path = BOOTLOADER_FILE_1260;
	uint32_t reg_value = 0;

	/* Macroses are local. */
#define APB_WRITE(reg, val) \
	do { \
		ret = cw1200_apb_write_32(priv, CW12000_APB(reg), (val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, "%s: can't write %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
#define APB_READ(reg, val) \
	do { \
		ret = cw1200_apb_read_32(priv, CW12000_APB(reg), &(val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, "%s: can't read %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
#define REG_WRITE(reg, val) \
	do { \
		ret = cw1200_reg_write_32(priv, (reg), (val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, "%s: can't write %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)
#define REG_READ(reg, val) \
	do { \
		ret = cw1200_reg_read_32(priv, (reg), &(val)); \
		if (ret < 0) { \
			DEBUG(DBG_ERROR, "%s: can't read %s at line %d.\n", \
				__func__, #reg, __LINE__); \
			goto error; \
		} \
	} while (0)

	
	/* Initialize common registers */
	//APB_WRITE(DOWNLOAD_IMAGE_SIZE_REG, DOWNLOAD_ARE_YOU_HERE);
	//APB_WRITE(DOWNLOAD_PUT_REG, 0);
	//APB_WRITE(DOWNLOAD_GET_REG, 0);
	//APB_WRITE(DOWNLOAD_STATUS_REG, DOWNLOAD_PENDING);
	//APB_WRITE(DOWNLOAD_FLAGS_REG, 0);
	/* Initialize common registers */
	//APB_WRITE(DOWNLOAD_IMAGE_SIZE_REG, DOWNLOAD_ARE_YOU_HERE);
	reg_value = DOWNLOAD_ARE_YOU_HERE;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON +
					DOWNLOAD_IMAGE_SIZE_REG,
					(uint8_t *)&reg_value, BIT_32_REG)) {
		DEBUG(DBG_ERROR, "%s:SBUS_SramWrite_APB()returned error \n",
			__func__);
		goto error;
	}
	
	//APB_WRITE(DOWNLOAD_PUT_REG, 0);
	reg_value = 0;
		if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_PUT_REG,
					(uint8_t *)&reg_value, sizeof(uint32_t))) {
			DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Put failed \n",
					__func__);
			goto error;
		}
	
	//APB_WRITE(DOWNLOAD_GET_REG, 0);
	reg_value = 0;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_GET_REG, (uint8_t *)&reg_value,
				sizeof(uint32_t))) {
		DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Get failed \n",
				__func__);
		goto error;
	}
	
	//APB_WRITE(DOWNLOAD_STATUS_REG, DOWNLOAD_PENDING);
	reg_value = DOWNLOAD_PENDING;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_STATUS_REG,
				(uint8_t *)&reg_value, sizeof(uint32_t))) {
		DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Status Reg"
				" failed \n", __func__);
		goto error;
	}
	
	//APB_WRITE(DOWNLOAD_FLAGS_REG, 1);
	reg_value = 0;
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_FLAGS_REG,
				(uint8_t *)&reg_value, sizeof(uint32_t))) {
		DEBUG(DBG_ERROR, "%s:ERROR: Download Crtl Flag Reg failed \n",
				__func__);
		goto error;
	}

	/* Write the NOP Instruction */
	//REG_WRITE(ST90TDS_SRAM_BASE_ADDR_REG_ID, 0xFFF20000);
	//REG_WRITE(ST90TDS_AHB_DPORT_REG_ID, 0xEAFFFFFE);

#if 0
	/* Write the NOP Instruction */
	reg_value = 0xFFF20000;
	if (SBUS_SDIO_RW_Reg(priv, ST90TDS_SRAM_BASE_ADDR_REG_ID,
			(uint8_t *)&reg_value, BIT_32_REG, SDIO_WRITE)) {
		DEBUG(DBG_ERROR, "%s:ERROR:SRAM Base Address Reg Failed \n",
			__func__);
		goto error;
	}

	reg_value = 0xEAFFFFFE;
	if (SBUS_SDIO_RW_Reg(priv, ST90TDS_AHB_DPORT_REG_ID,
			(uint8_t *)&reg_value, BIT_32_REG, SDIO_WRITE)) {
		DEBUG(DBG_ERROR, "%s:ERROR:NOP Write Failed \n", __func__);
		goto error;
	}
#endif
	/* Release CPU from RESET */
	REG_READ(ST90TDS_CONFIG_REG_ID, val32);
	val32 &= ~ST90TDS_CONFIG_CPU_RESET_BIT;
	REG_WRITE(ST90TDS_CONFIG_REG_ID, val32);

	/* Enable Clock */
	val32 &= ~ST90TDS_CONFIG_CPU_CLK_DIS_BIT;
	REG_WRITE(ST90TDS_CONFIG_REG_ID, val32);

 /*  
	if (SBUS_SDIO_RW_Reg(priv, ST90TDS_CONFIG_REG_ID,
			(char *)&val32, BIT_32_REG, SDIO_READ)) {
		DEBUG(DBG_ERROR, "%s:ERROR:Read Config Reg Failed \n",
			__func__);
		goto error;
	}
	val32 &= ~ST90TDS_CONFIG_CPU_RESET_BIT;
	if (SBUS_SDIO_RW_Reg(priv, ST90TDS_CONFIG_REG_ID,
			(char *)&val32, BIT_32_REG, SDIO_WRITE)) {
		DEBUG(DBG_ERROR, "%s:ERROR:Write Config Reg Failed \n",
			__func__);
		goto error;
	}

	val32 &= ~ST90TDS_CONFIG_CPU_CLK_DIS_BIT;
	if (SBUS_SDIO_RW_Reg(priv, ST90TDS_CONFIG_REG_ID,
			(char *)&val32, BIT_32_REG, SDIO_WRITE)) {
		DEBUG(DBG_ERROR, "%s:ERROR:Write Config Reg Failed \n",
			__func__);
		goto error;
	}
*/


	/* Load a firmware file */
	ret = request_firmware(&firmware, fw_path, priv->pdev);
	if (ret) {
		DEBUG(DBG_ERROR, "%s: can't load firmware file %s.\n",
			__func__, fw_path);
		ret = ERROR_SDIO;
		goto error;
	}
	BUG_ON(!firmware->data);

	buf = kmalloc(DOWNLOAD_BLOCK_SIZE, GFP_KERNEL | GFP_DMA);
	if (!buf) {
		DEBUG(DBG_ERROR, 
			"%s: can't allocate firmware buffer.\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	/* Check if the bootloader is ready */
	for (i = 0; i < 100; i += 1 + i / 2) {
		//APB_READ(DOWNLOAD_IMAGE_SIZE_REG, val32);
		if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_IMAGE_SIZE_REG,
				(uint8_t *)&val32, BIT_32_REG)){
		ret = ERROR_SDIO;
		goto error;
		}

		if (val32 == DOWNLOAD_I_AM_HERE)
			break;
		//mdelay(i);
		mdelay(10);
		if (val32 == DOWNLOAD_I_AM_HERE)
			break;
		mdelay(i);
	} /* End of for loop */

	if (val32 != DOWNLOAD_I_AM_HERE) {
		DEBUG(DBG_ERROR, 
			"%s: bootloader is not ready.\n", __func__);
		ret = -ETIMEDOUT;
		goto error;
	}

	/* Calculcate number of download blocks */
	num_blocks = (firmware->size - 1) / DOWNLOAD_BLOCK_SIZE + 1;

	/* Updating the length in Download Ctrl Area */
	val32 = firmware->size; /* Explicit cast from size_t to u32 */
	//APB_WRITE(DOWNLOAD_IMAGE_SIZE_REG, val32);
	if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON +
						DOWNLOAD_IMAGE_SIZE_REG,
						(uint8_t *)&val32, BIT_32_REG)) {
		DEBUG(DBG_ERROR, "%s:SBUS_SramWrite_APB()returned error \n",
				__func__);
		ret = ERROR_SDIO;
		goto error;
	}

	/* Firmware downloading loop */
	for (block = 0; block < num_blocks ; block++) {
		size_t tx_size;
		size_t block_size;

		/* check the download status */
		//APB_READ(DOWNLOAD_STATUS_REG, val32);
		if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
						+ DOWNLOAD_STATUS_REG,
						(uint8_t *)&val32, BIT_32_REG)) {
			DEBUG(DBG_ERROR,
			"%s:SBUS_SramRead_APB()returned error \n",
					__func__);
			ret = ERROR_SDIO;
			goto error;
		}
		if (val32 != DOWNLOAD_PENDING) {
			DEBUG(DBG_ERROR, 
				"%s: bootloader reported error %d.\n",
				__func__, val32);
			ret = -EIO;
			goto error;
		}

		/* loop until put - get <= 24K */
		for (i = 0; i < 100; i++) {
			//APB_READ(DOWNLOAD_GET_REG, get);
			if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
							+ DOWNLOAD_GET_REG,
						(uint8_t *)&get,BIT_32_REG)) {
				DEBUG(DBG_ERROR, "Sram Read get failed \n");
				ret = ERROR_SDIO;
				goto error;
			}
			if ((put - get) <=
			    (DOWNLOAD_FIFO_SIZE - DOWNLOAD_BLOCK_SIZE))
				break;
			mdelay(i);
		}

		if ((put - get) > (DOWNLOAD_FIFO_SIZE - DOWNLOAD_BLOCK_SIZE)) {
			DEBUG(DBG_ERROR, 
				"%s: Timeout waiting for FIFO.\n",
				__func__);
			return -ETIMEDOUT;
		}

		/* calculate the block size */
		tx_size = block_size = min((size_t)(firmware->size - put),
			(size_t)DOWNLOAD_BLOCK_SIZE);

		memcpy(buf, &firmware->data[put], block_size);
		if (block_size < DOWNLOAD_BLOCK_SIZE) {
			memset(&buf[block_size],
				0, DOWNLOAD_BLOCK_SIZE - block_size);
			tx_size = DOWNLOAD_BLOCK_SIZE;
		}

		/* send the block to sram */
		/*ret = cw1200_apb_write(priv,
			CW12000_APB(DOWNLOAD_FIFO_OFFSET +
				(put & (DOWNLOAD_FIFO_SIZE - 1))),
			buf, tx_size);*/
		if (SBUS_SramWrite_APB(priv,
					PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_FIFO_OFFSET
					+ (put & (DOWNLOAD_FIFO_SIZE - 1)),
					buf, tx_size)) {
			DEBUG(DBG_ERROR, "%s:SRAM Write Error \n", __func__);
			ret = ERROR_SDIO;
			goto error;
		}
		/* update the put register */
		put += block_size;
		//APB_WRITE(DOWNLOAD_PUT_REG, put);
		if (SBUS_SramWrite_APB(priv, PAC_SHARED_MEMORY_SILICON
					+ DOWNLOAD_PUT_REG,
					(uint8_t *)&put, BIT_32_REG)) {
			DEBUG(DBG_ERROR, "%s: Sram Write update put failed\n",
					__func__);
			ret = ERROR_SDIO;
			goto error;
		}
	} /* End of firmware download loop */

	/* Wait for the download completion */
	for (i = 0; i < 300; i += 1 + i / 2) {
		//APB_READ(DOWNLOAD_STATUS_REG, val32);
		if (SBUS_SramRead_APB(priv, PAC_SHARED_MEMORY_SILICON
				+ DOWNLOAD_STATUS_REG,
				(uint8_t *)&val32, BIT_32_REG)) {
		DEBUG(DBG_ERROR, "%s: Sram Read failed \n", __func__);
		ret = ERROR_SDIO;
		goto error;
		}
		if (val32 != DOWNLOAD_PENDING)
			break;
		mdelay(i);
	}
	if (val32 != DOWNLOAD_SUCCESS) {
		DEBUG(DBG_ERROR, 
			"%s: wait for download completion failed. " \
			"Read: 0x%.8X\n", __func__, val32);
		ret = -ETIMEDOUT;
		goto error;
	} else {
		DEBUG(DBG_ERROR, 
			"Firmware download completed.\n");
		ret = 0;
	}

error:
	kfree(buf);
	if (firmware)
		release_firmware(firmware);
	return ret;

#undef APB_WRITE
#undef APB_READ
#undef REG_WRITE
#undef REG_READ
}
