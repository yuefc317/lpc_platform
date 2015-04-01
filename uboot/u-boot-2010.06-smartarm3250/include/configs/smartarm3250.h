/*
 * Copyright (C) 2008 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Phytec 3250 board configuration file
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Phytec 3250 board configuation data
 */

#ifndef __smartarm3250_H__
#define __smartarm3250_H__

/*
 * There are 2 boot options for u-boot on the Phytec 3250 board. Option 1
 * or option 2. In either cases, u-boot does not need to be relocated.
 *
 * Option 1 - define CFG_BOOT_USES1L
 * With this option, the S1L loader present in the board initializes the
 * system (including SDRAM, MMUs, some MUX states, etc.). U-boot is loaded
 * into an already initialized system in SDRAM at address 0x83FC0000 (the
 * end of SDRAM in a 64M system). Because most of the system is already
 * initialized, system init is not performed again.
 *
 * Option 2 - undefine CFG_BOOT_USES1L
 * With this option, u-boot is the primary boot loader that is loaded and
 * started from the Phytec kickstart loader (see documentation with the
 * Phytec board for the kickstart loader). In this configuration, u-boot
 * loads and runs from RAM at address 0x00000000 and requires complete
 * system initialization. The kickstart loader will copy the u-boot image
 * from FLASH starting at block 1 into IRAM and start it at address 0x0.
 */

#define CONFIG_BOOT_USES1L

#ifdef CONFIG_BOOT_USES1L
/*
 * Skip low level init of MMU, SDRAM, muxing, etc. if u-boot is loaded
 * and executed from S1L
 */
#define CONFIG_SKIP_LOWLEVEL_INIT
#endif

/*
 * Linux machine type
 */
#define MACH_TYPE_UBOOTSYS MACH_TYPE_LPC3XXX

/*
 * Interrupts are not supported in this boot loader
 */
#undef CONFIG_USE_IRQ

/*
 * Use verbose help
 */
#define CONFIG_SYS_LONGHELP

/*
 * System UART selection, valid selections include UART3, UART4,
 * UART5, and UART6
 */
#define CONFIG_SYS_UART_SEL     UART5

/*
 * Default baud rate and baud rate table
 */
#define CONFIG_BAUDRATE		        115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * Default boot delay is 3 seconds
 */
#define CONFIG_BOOTDELAY            1

/*
 * SDRAM physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS    1
#define PHYS_SDRAM_1			0x80000000 /* SDRAM Bank #1 */

/*
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE (32*1024) /* 32K stack */
//#define CONFIG_STACKSIZE (512*1024) /* 512K stack */

/*
 * NOR FLASH not supported
 */
#define CONFIG_NO_FLASH
#define CONFIG_SYS_NO_FLASH

//#define CONFIG_SYS_MAX_FLASH_SECT 128 /*max nunber of sections on one chip*/

/*
 * Support for NAND FLASH, environment store in NAND at block 100
 */
#define CONFIG_CMD_NAND
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_SYS_MAX_FLASH_BANKS  1
#define CONFIG_SYS_MAX_NAND_DEVICE  1
#define CONFIG_SYS_NAND_BASE        0x20020000 /* SLC NAND controller */
#define NAND_MAX_CHIPS              1

/* Pick only 1 based on 32M or 64M FLASH boards */
#define CONFIG_SYS_MAX_FLASH_SECT (32*4096)//64M

/*
 * ATAG support
 */
#define CONFIG_CMDLINE_TAG		    1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		    1

/*
 * Environment support
 */
#define CONFIG_CMD_ENV

/*
 * 1KHz clock tick
 */
#define CONFIG_SYS_HZ   1000

/*
 * Support for various commands
 */
#define CONFIG_CMD_ECHO     /* ECHO command */
#define CONFIG_CMD_CACHE    /* Cache support */
#define CONFIG_CMD_RUN
#define CONFIG_CMD_LOADB
#define CONFIG_CMD_LOADS
#define CONFIG_CMD_SAVES
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_PING
#define CONFIG_CMD_NET
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_BDI
#define CONFIG_CMDLINE_EDITING  1

/*
 * Default range for the memory tests
 */
#define CONFIG_SYS_MEMTEST_START    0x80010000
#define CONFIG_SYS_MEMTEST_END      0x81000000

/*
 * Support for various capabilities
 */
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_LOADS_BAUD_CHANGE
#define CONFIG_ZERO_BOOTDELAY_CHECK	    /* check for keypress on bootdelay==0 */

/*
 * Ethernet buffer support in uncached IRAM and buffer size
 */
#define USE_IRAM_FOR_ETH_BUFFERS
#define IRAM_ETH_BUFF_BASE          0x08010000 /* Uncached IRAM */
#define IRAM_ETH_BUFF_SIZE          0x00010000

/*
 * Address and size of Environment Data
 */
#define CONFIG_ENV_ADDR		    0x80000100 /* Passed to kernel here */
#define CONFIG_ENV_SIZE		    0x40000 /* 2 block , 256K*/
#define CONFIG_ENV_OFFSET		0x00180000 /* Block 12 */
#define CONFIG_ENV_OVERWRITE 
#define CONFIG_CMD_SAVEENV

/*
 * Various network related parameters
 */
#define CONFIG_NETMASK          255.255.255.0
#define CONFIG_ETHADDR          00:04:08:0c:10:14
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_HOSTNAME       smartarm3250
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTCOMMAND "run yboot"//"tftpboot;bootm"
#define CONFIG_BOOTARGS "ubi.mtd=4 root=ubi0:rootfs console=ttyS0,115200 mem=32M rootfstype=ubifs rw"

/*
 * Area and size for malloc
 */
#define CONFIG_SYS_MALLOC_LEN       (1024*1024)
#define CONFIG_SYS_GBL_DATA_SIZE    128

/*
 * Prompt, command buffer
 */
#define	CONFIG_SYS_CBSIZE	256		            /* Console I/O Buffer Size	*/
#define	CONFIG_SYS_PROMPT	"U-Boot$"	        /* Monitor Command Prompt	*/
#define	CONFIG_SYS_PBSIZE   (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */
#define	CONFIG_SYS_MAXARGS	16		            /* max number of command args	*/
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE	/* Boot Argument Buffer Size	*/

/*
 * Default load address for programs
 */
#define	CONFIG_SYS_LOAD_ADDR		0x80100000	/* default load address	*/

#define MTDIDS_DEFAULT "nand0=nandflash0"
//default parts must be identical with your flash
#define MTDPARTS_DEFAULT "mtdparts=nandflash0:1536k(bootloder),"\
						"512k(params),"\
						"4m(kernel),"\
						"16m(safefs),"\
						"-(rootfs)"

#define CONFIG_MTD_DEVICE       1 
#define CONFIG_MTD_PARTITIONS   1 
#define CONFIG_CMD_MTDPARTS 
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_UBI
#define CONFIG_LZO              1
#define CONFIG_RBTREE           1 

/*
 * Other preset environment variables
 */
#define CONFIG_EXTRA_ENV_SETTINGS \
			"serverip="         "192.168.1.138\0"               \
			"gatewayip="        "192.168.1.254\0"               \
			"ipaddr="           "192.168.1.136\0"               \
                                                                \
			"bootfile="         "uImage\0"                      \
            "loadaddr="         "0x80008000\0"                  \
            "bootmaddr="        "0x81000000\0"                  \
            "kerneladdr="       "0x00200000\0"                  \
                                                                \
            "safefs="           "safefs.ubi\0"                  \
            "rootfs="           "rootfs.ubi\0"                  \
                                                                \
            "monitor="          "0\0"                           \
            "opt="              " \0"                           \
			                                                                                \
			"upkernel="         "tftp $(loadaddr) $(bootfile);"                             \
                    			"nand erase clean $(kerneladdr) $(filesize);"               \
                    			"nand write.jffs2 $(loadaddr) $(kerneladdr) $(filesize);"   \
                    			"setenv kernelsize $(filesize); saveenv\0"                  \
                                                                                            \
			"upsafefs="         "mtdparts default;"                                         \
                                "nand erase safefs;"                                        \
                                "ubi part safefs;"                                          \
                                "ubi create  safefs;"                                       \
                                "tftp $(loadaddr) $(safefs);"                               \
			                    "ubi write $(loadaddr) safefs $(filesize)\0"                \
			                                                                                \
			"uprootfs="         "mtdparts default;"                                         \
                                "nand erase rootfs;"                                        \
                                "ubi part rootfs;"                                          \
                                "ubi create  rootfs;"                                       \
                                "tftp $(loadaddr) $(rootfs);"                               \
			                    "ubi write $(loadaddr) rootfs $(filesize)\0"                \
			                                                                                \
			"yboot="            "nand read.jffs2 $(bootmaddr) $(kerneladdr) $(kernelsize);" \
			                    "bootm $(bootmaddr)\0"                                      \
			                                                                                \
			"safemode="         "setenv bootargs ubi.mtd=3 root=ubi0:safefs "               \
                                "console=ttyS0,115200 mem=32M rootfstype=ubifs rw;"         \
                                "run yboot\0"                                               \
			                                                                                \
			"zhiyuan="          "run upsafefs; run upkernel; run set\0"                     \
                                                                                            \
            "set="              "setenv bootargs ubi.mtd=4 root=ubi0:rootfs rootfstype=ubifs "\
                                "console=ttyS0,115200 mem=32M, monitor=$(monitor) ethaddr=$(ethaddr); "\
                                "saveenv\0"



			

#endif  /* __smartarm3250_H__*/

