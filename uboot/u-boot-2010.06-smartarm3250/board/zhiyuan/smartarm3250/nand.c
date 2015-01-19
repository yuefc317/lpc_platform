/*
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian.pop@leadtechdesign.com>
 * Lead Tech Design <www.leadtechdesign.com>
 *
 * (C) Copyright 2006 ATMEL Rousset, Lacressonniere Nicolas
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include "lpc3250.h"
#include <nand.h>
#include <asm/errno.h>

#define	NAND_ALE_OFFS   4
#define	NAND_CLE_OFFS   8

#define ABING 0
//declarat the fuction
static void smartarm3250_write_byte(struct mtd_info *mtd, u_char byte);

static void smartarm3250_nand_init(void)
{
	    /* Enable clocks to the SLC NAND controller */
	    CLKPWR->clkpwr_nand_clk_ctrl = 0x05; // TBD use manifest values instead

        /* Reset SLC NAND controller & clear ECC */
        SLCNAND->slc_ctrl = (SLCCTRL_SW_RESET | SLCCTRL_ECC_CLEAR);

        /* 8-bit bus, no DMA, CE normal */
        SLCNAND->slc_cfg = 0;

        /* Interrupts disabled and cleared */
        SLCNAND->slc_ien = 0;
        SLCNAND->slc_icr = (SLCSTAT_INT_TC | SLCSTAT_INT_RDY_EN);

	    // TBD fix me with optimal timings later
        /* Start with slowest timings possible */
        SLCNAND->slc_tac = (SLCTAC_WDR(15) |
                    SLCTAC_WWIDTH(15) |
                    SLCTAC_WHOLD(15) |
                    SLCTAC_WSETUP(15) |
                    SLCTAC_RDR(15) |
                    SLCTAC_RWIDTH(15) |
                    SLCTAC_RHOLD(15) |
                    SLCTAC_RSETUP(15));
}

/*
static void smartarm3250_nand_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct nand_chip *this = mtd->priv;
	ulong IO_ADDR_W = (ulong) this->IO_ADDR_W;

	IO_ADDR_W &= ~0xFF;
	switch (cmd) {
	case NAND_CTL_SETCLE:
		IO_ADDR_W += NAND_CLE_OFFS;
		break;
	case NAND_CTL_SETALE:
		IO_ADDR_W += NAND_ALE_OFFS;
		break;
	case NAND_CTL_CLRNCE:
		SLCNAND->slc_cfg &= ~SLCCFG_CE_LOW;
		break;
	case NAND_CTL_SETNCE:
		SLCNAND->slc_cfg |= SLCCFG_CE_LOW;
		break;
	}
	this->IO_ADDR_W = (void *) IO_ADDR_W;
}
*/
//this function is changed for the memeber function pointer cmd_ctl of struct nand_chip by yuanlulu
static void smartarm3250_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	ulong IO_ADDR_W = (ulong) this->IO_ADDR_W;

	//IO_ADDR_W &= ~0xFF;
	if (ctrl & NAND_CTRL_CHANGE) {
		//if ( (ctrl&NAND_CLE) || (ctrl&NAND_ALE) )
		//	IO_ADDR_W &= ~0xFF;
		IO_ADDR_W &= ~(NAND_CLE_OFFS | NAND_ALE_OFFS);
		if(ctrl & NAND_CLE)
			IO_ADDR_W |= NAND_CLE_OFFS;//write command
		if(ctrl & NAND_ALE)
			IO_ADDR_W |= NAND_ALE_OFFS;//write address
		this->IO_ADDR_W = (void *) IO_ADDR_W;
		
		if(!(ctrl & NAND_NCE))
			SLCNAND->slc_cfg &= ~SLCCFG_CE_LOW;//disnable nand flash
		else 
			SLCNAND->slc_cfg |= SLCCFG_CE_LOW;//enable nand flash
	}
	//this->IO_ADDR_W = (void *) IO_ADDR_W;
	if(cmd != NAND_CMD_NONE)
		smartarm3250_write_byte(mtd, (u_char)cmd);
}

static int smartarm3250_nand_ready(struct mtd_info *mtd)
{
	/* Check the SLC NAND controller status */
	return (SLCNAND->slc_stat & SLCSTAT_NAND_READY);
}

static u_char smartarm3250_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_R;
	volatile unsigned long tmp32;
	tmp32 = *pReg;
	return (u_char) tmp32;
}

static void smartarm3250_write_byte(struct mtd_info *mtd, u_char byte)
{
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_W;
    //printf("ABING in %s\n",__FUNCTION__);
	*pReg = (unsigned long) byte;
}

/**
 * smartarm3250_write_buf - [DEFAULT] write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * Default write function for 8bit buswith
 */
static void smartarm3250_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_W;

	for (i=0; i<len; i++)
		*pReg = (unsigned long) buf[i];
}

/**
 * smartarm3250_read_buf - [DEFAULT] read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 8bit buswith
 */
static void smartarm3250_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_R;
	volatile unsigned long tmp32;

	for (i=0; i<len; i++)
	{
		tmp32 = *pReg;
		buf[i] = (u_char) tmp32;
	}
}

/**
 * nand_verify_buf - [DEFAULT] Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 * Default verify function for 8bit buswith
 */
static int smartarm3250_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;
	unsigned long *pReg = (unsigned long *) this->IO_ADDR_R;
	volatile unsigned long tmp32;

	for (i=0; i<len; i++)
	{
		tmp32 = *pReg;
		if (buf[i] != (u_char) tmp32)
			return -EFAULT;
	}

	return 0;
}

int board_nand_init(struct nand_chip *nand)
{
    //ABING
    //nand->IO_ADDR_W = (void __iomem *)CFG_NAND_BASE;
    //nand->IO_ADDR_R = (void __iomem *)CFG_NAND_BASE;
    //ABING


	/* Initial NAND interface */
	smartarm3250_nand_init();

	nand->options = NAND_SAMSUNG_LP_OPTIONS | NAND_NO_AUTOINCR;//0;//Abing
//	nand->eccmode = NAND_ECC_SOFT;
	nand->ecc.mode = NAND_ECC_SOFT;
	//nand->ecc.mode = NAND_ECC_NONE;
//	nand->hwcontrol = smartarm3250_nand_hwcontrol;
	nand->cmd_ctrl = smartarm3250_nand_hwcontrol;
	nand->dev_ready = smartarm3250_nand_ready;
	nand->chip_delay = 20;

#if 1 //ABING
//	nand->write_byte = smartarm3250_write_byte;
	nand->read_byte = smartarm3250_read_byte;
	nand->write_buf = smartarm3250_write_buf;
	nand->read_buf = smartarm3250_read_buf;
	nand->verify_buf = smartarm3250_verify_buf;
#endif /* if ABING */

	return 0;
}
