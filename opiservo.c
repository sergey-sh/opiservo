/*
 * opiservo.c Multiple Servo Driver for the Orange PI one
 * Copyright (c) 2018 Sergey Shkuliov <sergey.sckuliov@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * If you want the device node created automatically create these two
 * files, and make /lib/udev/opiservo executable (chmod +x):
 *
 * ============= /etc/udev/rules.d/20-opiservo.rules =============
 * SUBSYSTEM=="module", DEVPATH=="/module/opiservo", RUN+="/lib/udev/opiservo"
 * ===================================================================
 *
 * ===================== /lib/udev/opiservo ======================
 * #!/bin/bash
 *
 * if [ "$ACTION" = "remove" ]; then
 *         rm -f /dev/opiservo
 * elif [ "$ACTION" = "add" ]; then
 *          major=$( sed -n 's/ opiservo//p' /proc/devices )
 *        [ "$major" ] && mknod -m 0666 /dev/opiservo c $major 0
 * fi
 *
 * exit 0
 * ===================================================================
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/scatterlist.h>
#include <linux/dma/sunxi-dma.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>

#define DEBUGSERVO
#ifdef DEBUGSERVO
/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')

#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8              PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16             PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BINARY_PATTERN_INT64    \
    PRINTF_BINARY_PATTERN_INT32             PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */

#endif

#define NUM_PINS 64
#define RESERVED_MEM_PIN 10
#define NUM_GPIO_REGISTER 9

#define MAX_BUF_VALUE 21

// This struct is used to store all temporary data associated with a given
// open() of /dev/opiservo
struct private_data {
	int rd_query;	
	int rd_len;
	char rd_data[NUM_PINS*RESERVED_MEM_PIN];
	int wr_len;
	char wr_data[NUM_PINS*RESERVED_MEM_PIN];
};

struct full_info_dma_lli {
	u32		cfg;		/* DMA configuration */
	// for check 
	dma_addr_t	src;		/* Source address */
	dma_addr_t	dst;		/* Destination address */
};

/* lli: linked list ltem, the DMA block descriptor */
struct sunxi_dma_lli {
	u32		cfg;		/* DMA configuration */
	dma_addr_t	src;		/* Source address */
	dma_addr_t	dst;		/* Destination address */
	u32		len;		/* Length of buffers */
	u32		para;		/* Parameter register */
	dma_addr_t	p_lln;		/* Next lli physical address */
	struct sunxi_dma_lli *v_lln;	/* Next lli virtual address (only for cpu) */
#ifdef DEBUG
	dma_addr_t	this_phy;	/* Physical address of this lli */
	#define set_this_phy(li, addr)	\
		((li)->this_phy = (addr))
#else
	#define set_this_phy(li, addr)
#endif
}__attribute__((packed));

struct virt_dma_desc {
	struct dma_async_tx_descriptor tx;
	/* protected by vc.lock */
	struct list_head node;
};

struct virt_dma_chan {
	struct dma_chan	chan;
	struct tasklet_struct task;
	void (*desc_free)(struct virt_dma_desc *);

	spinlock_t lock;

	/* protected by vc.lock */
	struct list_head desc_submitted;
	struct list_head desc_issued;
	struct list_head desc_completed;

	struct virt_dma_desc *cyclic;
};

struct sunxi_desc {
	struct virt_dma_desc	vd;
	dma_addr_t		lli_phys;	/* physical start for llis */
	struct sunxi_dma_lli	*lli_virt;	/* virtual start for lli */
};

struct sunxi_chan {
	struct virt_dma_chan	vc;

	struct list_head	node;		/* queue it to pending list */
	struct dma_slave_config	cfg;
	bool	cyclic;

	struct sunxi_desc	*desc;
	u32	irq_type;
};


static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static long dev_ioctl(struct file *, unsigned int, unsigned long);

static struct file_operations fops = {
	.open = dev_open,
	.read = dev_read,
	.write = dev_write,
	.release = dev_close,
	.unlocked_ioctl = dev_ioctl,
	.compat_ioctl = dev_ioctl,
};

static dev_t devno;
static struct cdev my_cdev;
static int my_major;
static struct dma_chan *chan = 0;
static dma_cookie_t cookie;
static struct dma_async_tx_descriptor *tx = 0;
bool terminated = false;

#define OPISERVO_CDEVNAME "opiservo"
#define NSEC_PER_MSEC   1000000L
#define MS_TO_NS(x) (x * NSEC_PER_MSEC)

// GPIO
static volatile uint32_t *gpio; 
static volatile uint32_t *serial; 

static int BP_PIN_MASK[NUM_GPIO_REGISTER][32] =  //[BANK]  [INDEX]
{
 { 0, 1, 2, 3,-1,-1, 6, 7, 8, 9,10,11,12,13,14,-1,-1,-1,18,19,20,21,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
 { 0, 1, 2, 3, 4,-1,-1, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,14,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
 {-1,-1,-1,-1,-1,-1, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
};

static bool UseServoRegister[NUM_GPIO_REGISTER] =  //[BANK]
{
 false,//PA
 false,//PB
 false,//PC
 false,//PD
 false,//PE
 false,//PF
 false,//PG
 false,//PH
 false,//PI
};

static int UseServoRegister_count =  0;

static int pinToGpio_BP [NUM_PINS] =
{
  1,110,    // 0, 1
   0,  3,    // 2, 3
  68, 71,    // 4  5
   2,  6,    // 6, 7
  12, 11,    // 8, 9
  67, 21,    //10,11
  64, 65,    //12,13
  66, 13,    //14,15
  14, -1,    //16,17
  -1, -1,    //18,19
  -1,  7,    //20,21
   8,  9,    //22,23
  10, 20,    //24,25
 200,201,    //26,27
 198,199,    //28,29
  19, 18,    //30,31

 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,// ... 63
} ;

#define	MAX_PIN_NUM		(0x40)  //64
#define SUNXI_PORT_BASE		(0x01C20000)

#define SUNXI_GPIO_OFFSET		(0x0800)
#define	SUNXI_GPIO_LEN		(4*1024)

#define	NONE_MODE			 -1
#define	INPUT_MODE			 0
#define	OUTPUT_MODE			 1
#define	SERVO_MODE			 2

#define	PUD_OFF			 0
// sunxi_pud (not compatible wiring PI 0,1,2)
#define	PUD_DOWN		 2
#define	PUD_UP			 1

#define	LOW			 0
#define	HIGH			 1
#define BUS_CLK_GATING_REG3 (0x006C>>2)
#define BUS_SOFT_RST_REG4 (0x02D8>>2)

#define UART_BASE (0x01C28C00)
#define UART3_GATING (BIT(19))
#define UART3_RESET (BIT(19))
#define SW_UART_HALT_LCRUP     (BIT(2))
#define SW_UART_HALT_FORCECFG  (BIT(1))
#define SW_UART_LCR_DLAB        (BIT(7))

#define UART_RBR (0x00>>2)
#define UART_THR (0x00>>2)
#define UART_DLL (0x00>>2)
#define UART_DLH (0x04>>2)
#define UART_IER (0x04>>2)
#define UART_IIR (0x08>>2)
#define UART_FCR (0x08>>2)
#define UART_LCR (0x0C>>2)
#define UART_MCR (0x10>>2)
#define UART_LSR (0x14>>2)
#define UART_MSR (0x18>>2)
#define UART_SCH (0x1C>>2) 
#define UART_USR (0x7C>>2) 
#define UART_TFL (0x80>>2) 
#define UART_RFL (0x84>>2) 
#define UART_HALT (0xA4>>2) 

#ifdef CONFIG_ARCH_SUN9I
#define LINK_END	0x1FFFF800		/* lastest link must be 0x1ffff800 */
#else
#define LINK_END	0xFFFFF800		/* lastest link must be 0xfffff800 */
#endif


static int pinGpio_Mode [NUM_PINS] = {
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
	NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE, NONE_MODE,
};

static int pinGpio_Set [NUM_PINS] = {
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
	LOW, LOW, LOW, LOW, LOW, LOW, LOW,LOW,
};

// must division 100000 for usec
//static int senddma_uarttime1byte = 156457;
static int senddma_uarttime1byte = 142234;
//static int senddma_uarttime1byte = 200000;
//static const int senddma_gpiotime4byte = 5796;

// usec
static int senddma_sequence_periode = 20000;
static int senddma_uartsyncbytes;
static int senddma_gpiosyncbytes;

static bool need_reinit = false;
static bool need_reupdate = false;
static bool is_start_dma_send = false;

static char *senddma_uartbuf = NULL;
static uint32_t *senddma_gpiobuf = NULL;
static uint32_t index_gpiobuf = 0;
static uint32_t size_gpiobuf = 0;

static int sg_max_len = 2*(NUM_PINS+1)+3;
static struct scatterlist sgl[2*(NUM_PINS+1)+3];
static int sg_real_len = 0;

static int pins[NUM_PINS];
static int c_pins = 0,c_regs = 0;
static int regs[NUM_GPIO_REGISTER];

struct gpio_send_plan {
	uint32_t value;
	int bank;
	int sleep;
	dma_addr_t phys;
};

static int first_odd_gpio_send_bytes = 0;
static int count_send_plan = 0;
static struct gpio_send_plan send_plan[2*NUM_PINS];

static void init_uart_port(void);
static void release_uart_port(void);
static void init_dma_properties(void);
static void release_dma_properties(void);

static inline struct sunxi_desc *to_sunxi_desc(struct dma_async_tx_descriptor *tx) {
	return container_of(tx, struct sunxi_desc, vd.tx);
}

#ifdef DEBUGSERVO
static bool is_dumped_sunxi_sg_list = false;

static void dump_sunxi_sg_list(struct dma_async_tx_descriptor *tx,int maxitem) {
	struct sunxi_desc *txd = 0;
	struct sunxi_dma_lli *lli;
	int i;
	
	txd = to_sunxi_desc(tx);
	i=0;
	lli = txd->lli_virt;
	while(lli && (maxitem==0 || i<maxitem)) {
		printk("cfg: %x src: %x dst: %x len: %x para: %x next: %x\n",
			lli->cfg,
			lli->src,
			lli->dst,
			lli->len,
			lli->para,
			lli->p_lln
		);
		lli = lli->v_lln;
		i++;
	}
}

static void dumpSendPlan(void) {
	int i;
	printk("Bank\tSleep\tValue\n");
	for(i=0;i<count_send_plan;i++) {
		printk("%d\t%d\t"PRINTF_BINARY_PATTERN_INT32"\n",
			send_plan[i].bank,
			send_plan[i].sleep,
			PRINTF_BYTE_TO_BINARY_INT32(send_plan[i].value)
		);
	}
}
static void dumpWeightPins(void) {
	int i;
	printk("Pin\tValue\n");
	for(i=0;i<c_pins;i++) {
		printk("%d\t%d\n",
			pins[i],
			pinGpio_Set[pins[i]]
		);
	}
}

#endif

static inline struct sunxi_chan *to_sunxi_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sunxi_chan, vc.chan);
}

static uint32_t readlI(uint32_t addr) {
	return *(gpio + (addr >> 2));
}

static void writelI(uint32_t val, uint32_t addr) {
	#ifdef DEBUGSERVO
	void *x = (void *)(gpio + (addr >> 2));
	printk("Addr: %x Val: %x Bin: "PRINTF_BINARY_PATTERN_INT32" Dec: %d\n",(u32)x,val,PRINTF_BYTE_TO_BINARY_INT32(val),val);
	#endif
	*(gpio + (addr >> 2)) = val;
}

static char* charn2str(char *data,int len,char *buf,int buf_len) {
	if(len>=0 && len<buf_len-1) {
		memcpy(buf, data, len);
		buf[len]=0x0;
		return buf;
	}
	return 0;
}

static int get_bank_reg(int pin) {
	if ((pin>=0 && pin<MAX_PIN_NUM)) {
		pin = pinToGpio_BP [pin] ;
		return (pin >> 5);
	} 
	return -1;
}

static uint32_t get_bank_reg_phys(int pin) {
	return SUNXI_PORT_BASE+SUNXI_GPIO_OFFSET + (get_bank_reg(pin) * 36) + 0x10; // +0x10 -> data reg
}

static int get_bitoffset_reg(int pin) {
	int bank;
	if ((pin>=0 && pin<MAX_PIN_NUM)) {
		pin = pinToGpio_BP [pin] ;
		bank = pin >> 5;
		return pin - (bank << 5);
	} 
	return -1;
}

static void sunxi_pullUpDnControl (int pin, int pud)
{
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	int sub = index >> 4;
	int sub_index = index - 16*sub;
	uint32_t phyaddr = SUNXI_GPIO_OFFSET + (bank * 36) + 0x1c + sub*4; // +0x10 -> pullUpDn reg
#ifdef DEBUGSERVO
	printk("func:%s pin:%d,bank:%d index:%d sub:%d phyaddr:0x%x\n",__func__, pin,bank,index,sub,phyaddr); 
#endif
	if(BP_PIN_MASK[bank][index] != -1) {  //PI13~PI21 need check again
		regval = readlI(phyaddr);
#ifdef DEBUGSERVO
		printk("pullUpDn reg:0x%x, pud:0x%x sub_index:%d\n", regval, pud, sub_index);
#endif
		regval &= ~(3 << (sub_index << 1));
		regval |= (pud << (sub_index << 1));
#ifdef DEBUGSERVO
		printk("pullUpDn val ready to set:0x%x\n", regval);
#endif
		writelI(regval, phyaddr);
		regval = readlI(phyaddr);
#ifdef DEBUGSERVO
		printk("pullUpDn reg after set:0x%x  addr:0x%x\n", regval, phyaddr);
#endif
	 } else {
		printk("pin number error\n");
	 } 
	return ;
}

static void sunxi_set_gpio_mode(int pin,int mode) {
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	int offset = ((index - ((index >> 3) << 3)) << 2);
	uint32_t phyaddr = SUNXI_GPIO_OFFSET + (bank * 36) + ((index >> 3) << 2);
	if(BP_PIN_MASK[bank][index] != -1) {
		regval = readlI(phyaddr);
		regval &= ~(7 << offset);
		regval |=  ((mode & 7) << offset);
		writelI(regval, phyaddr);
	} else {
		printk("pin number error\n");
	}
}

static void pinMode (int pin, int mode) {
	if ((pin>=0 && pin<MAX_PIN_NUM)) {
		pin = pinToGpio_BP [pin] ;
		/*VCC or GND return directly*/
		if (-1 != pin) {
			if (mode == INPUT_MODE || mode == OUTPUT_MODE) {
				sunxi_set_gpio_mode(pin,mode);
			} else if (mode == SERVO_MODE) {
				sunxi_set_gpio_mode(pin,OUTPUT_MODE);
			} else {
				sunxi_set_gpio_mode(pin,INPUT_MODE);
			}
		}
	}
}

static void pullUpDnControl(int pin, int mode) {
	if ((pin>=0 && pin<MAX_PIN_NUM)) {
		pin = pinToGpio_BP [pin] ;
		/*VCC or GND return directly*/
		if (-1 != pin) {
			sunxi_pullUpDnControl(pin, mode);
		}
	}
}

static void sunxi_digitalWrite(int pin, int value) { 
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_OFFSET + (bank * 36) + 0x10; // +0x10 -> data reg
	if(BP_PIN_MASK[bank][index] != -1) {
		regval = readlI(phyaddr);
		if(0 == value) {
			regval &= ~(1 << index);
			writelI(regval, phyaddr);
		} else {
			regval |= (1 << index);
			writelI(regval, phyaddr);
		}
	} else {
		printk("pin number error\n");
	}
}

static void digitalWrite (int pin, int value) {
	if ((pin>=0 && pin<MAX_PIN_NUM)) {
		pin = pinToGpio_BP [pin] ;
		if(-1 == pin) {
			//printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
			return ;
		}
		sunxi_digitalWrite(pin, value);		
	}
}

static int sunxi_digitalRead(int pin) {
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_OFFSET + (bank * 36) + 0x10; // +0x10 -> data reg
	if(BP_PIN_MASK[bank][index] != -1) {
		regval = readlI(phyaddr);
		regval = regval >> index;
		regval &= 1;
		return regval;
	} else {
		printk("Sunxi_digitalRead() pin - number error\n");
		return regval;
	}
}

static int digitalRead(int pin) {
	if ((pin>=0 && pin<MAX_PIN_NUM)) {
		pin = pinToGpio_BP [pin] ;
		return sunxi_digitalRead(pin);
	}
	return 0;
}


static int __init opiservo_kernel_init(void) {
	int res;
	
	senddma_uartsyncbytes = (senddma_sequence_periode*100000)/senddma_uarttime1byte+1;
	size_gpiobuf = (NUM_PINS+2);
	index_gpiobuf = 0;
	senddma_gpiosyncbytes = size_gpiobuf*sizeof(*senddma_gpiobuf);
	
	printk(KERN_INFO "Hello, opiservo dma! uartsync: %d gpiosync: %d\n",senddma_uartsyncbytes,senddma_gpiosyncbytes);
	
	// senddma_sequence for all update servo data
	senddma_gpiobuf = kmalloc(senddma_gpiosyncbytes, GFP_ATOMIC|GFP_DMA);
	if(senddma_gpiobuf == 0) {
		printk(KERN_WARNING "OPIServo: Failed to allocate gpiobuf\n");
		return -1;
	}
	memset(senddma_gpiobuf,0,senddma_gpiosyncbytes);
	
	senddma_uartbuf = kmalloc(senddma_uartsyncbytes, GFP_ATOMIC|GFP_DMA);
	if(senddma_uartbuf == 0) {
		printk(KERN_WARNING "OPIServo: Failed to allocate uartbuf\n");
		return -1;
	}
	memset(senddma_uartbuf,0,senddma_uartsyncbytes);
	sg_init_table(sgl, sg_max_len);

	gpio = (uint32_t *)ioremap(SUNXI_PORT_BASE, SUNXI_GPIO_LEN);
	serial = (uint32_t *)ioremap(UART_BASE, SUNXI_GPIO_LEN);

	res = alloc_chrdev_region(&devno, 0, 1, OPISERVO_CDEVNAME);
	if (res < 0) {
		printk(KERN_WARNING "OPIServo: Can't allocated device number\n");
		return res;
	}
	my_major = MAJOR(devno);
	cdev_init(&my_cdev, &fops);
	my_cdev.owner = THIS_MODULE;
	my_cdev.ops = &fops;
	res = cdev_add(&my_cdev, MKDEV(my_major, 0), 1);
	if (res) {
		printk(KERN_WARNING "OPIServo: Error %d adding device\n", res);
		unregister_chrdev_region(devno, 1);
		return res;
	}
	init_uart_port();
	init_dma_properties();

	return 0; 
}

static void __exit opiservo_kernel_exit(void) {
	release_dma_properties();
	release_uart_port();		
	
	cdev_del(&my_cdev);
	unregister_chrdev_region(devno, 1);
	
	iounmap(gpio);
	iounmap(serial);

	kfree(senddma_gpiobuf);
	kfree(senddma_uartbuf);
	
	printk(KERN_INFO "Goodbye, opiservo dma!\n");
}

static void init_uart_port(void) {
	gpio[BUS_CLK_GATING_REG3]|=UART3_GATING;
	gpio[BUS_SOFT_RST_REG4]|=UART3_RESET;

	serial[UART_HALT] = SW_UART_HALT_FORCECFG;
	serial[UART_LCR] |= SW_UART_LCR_DLAB;
	serial[UART_DLL] = 1;
	serial[UART_DLH] = 0;

	serial[UART_LCR] &= ~SW_UART_LCR_DLAB;
	serial[UART_HALT] |= SW_UART_HALT_LCRUP;
}

static void release_uart_port(void) {
	gpio[BUS_CLK_GATING_REG3]&=~UART3_GATING;
	gpio[BUS_SOFT_RST_REG4]&=~UART3_RESET;
}

static bool is_use_servo_port(int pin) {
	int bank = get_bank_reg(pin);
	if(bank>=0 && bank<NUM_GPIO_REGISTER) {
		if(UseServoRegister[bank]) {
			return true;
		}
	}
	return false;
}
/* not use

static bool dma_sync_wait_residue_timeout(struct dma_chan *chan,dma_cookie_t cookie,int residue,int timeout) {
	enum dma_status status;
	unsigned long dma_sync_wait_timeout;
	struct dma_tx_state state;
	dma_sync_wait_timeout = jiffies + msecs_to_jiffies(timeout);
	do {
		status = dmaengine_tx_status(chan, cookie, &state);
		if(state.residue<residue) {
			break;
		}
		if (time_after_eq(jiffies, dma_sync_wait_timeout)) {
			printk(KERN_ERR "dma_sync_wait_timeout!\n");
			return false;
		}
	} while (status == DMA_IN_PROGRESS);
	return true;
}
*/

// release all mapped memory release dma
static void release_dma_properties() {
	if(chan) {
		printk("DMA Stop: %s\n",dma_chan_name(chan));

		dmaengine_terminate_all(chan);
		if(sg_real_len) {
			dma_unmap_sg(chan->device->dev, sgl, sg_real_len, DMA_TO_DEVICE);
			sg_real_len = 0;
		}
		dma_release_channel(chan);
		chan = NULL;
	}
}

// perform sg list and perfrom full hacked list with dst & cfg register
static struct dma_async_tx_descriptor * performDMAsg(void) {
	struct dma_async_tx_descriptor *tx;
	unsigned long flags;
	struct dma_slave_config slave_config;
	int ret;
	struct scatterlist *sg;
	unsigned int sg_len,l_uart;
	unsigned int i;
	int nents = 0;

	sg_len = 2*count_send_plan;
	sg_init_table(sgl, sg_len);
	
	for(i = 0, sg_real_len = 0, sg = (sgl); i<count_send_plan; i++, sg = sg_next(sg), sg_real_len++) {
		senddma_gpiobuf[i] = send_plan[i].value;
		sg_set_buf(sg, &senddma_gpiobuf[i], sizeof(uint32_t));
		// must check for sleep 0, if use different registers
		l_uart = (send_plan[i].sleep*100000)/senddma_uarttime1byte;
		sg = sg_next(sg);
		sg_set_buf(sg, senddma_uartbuf, l_uart);
		sg_real_len++;
	}
	flags = DMA_CTRL_ACK | DMA_COMPL_SKIP_SRC_UNMAP | DMA_COMPL_SKIP_DEST_UNMAP;
		slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config.dst_maxburst = 4;
		slave_config.src_maxburst = 4;
		slave_config.slave_id = sunxi_slave_id(DRQDST_SRAM, DRQSRC_SRAM);//DRQDST_SRAM

	slave_config.dst_addr = get_bank_reg_phys(0);
	slave_config.src_addr = 0;
	
	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		printk("dma slave config failed with err %d\n", ret);
		return 0;
	}
	
	nents = dma_map_sg(chan->device->dev, sgl, sg_real_len, DMA_TO_DEVICE);
	tx = dmaengine_prep_slave_sg(chan,sgl,nents,DMA_MEM_TO_DEV,flags);
	if (!tx) {
		printk("tx init error\n");
		return 0;
	}
	return tx;
}

static bool hackDMAsg(struct dma_async_tx_descriptor *tx) {
	struct sunxi_desc *txd = 0;
	struct sunxi_dma_lli *lli;
	int i = 0, j = 0;

	txd = to_sunxi_desc(tx);
	lli = txd->lli_virt;
	while(lli) {
		// need update register gpio dst if not bank0, perform before list for update
		if((j%2)==0) {
			lli->dst = get_bank_reg_phys(send_plan[i++].bank);
		} else {
			lli->cfg = 0x290000;
			lli->dst = UART_BASE;
		}
		lli = lli->v_lln;
		j++;
	}
	
	return true;
}

static void init_dma_properties(void) {
	dma_cap_mask_t mask;
	// start send dma, perform and hack dma cyclic query
	// strcmp(dev_name(chan->device->dev),"sunxi_dmac")==0
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY|DMA_SLAVE|DMA_SG, mask);
	chan = dma_request_channel(mask, 0, NULL);
	if (chan) {
		// prep_dma_sg
		printk("DMA: %s\n",dma_chan_name(chan));
		if(strcmp(dev_name(chan->device->dev),"sunxi_dmac")!=0) {
			printk("DMA Driver is not sunxi_dmac: %s\n",dev_name(chan->device->dev));
			release_dma_properties();
		}
	}
}

static void cb_complete_dma_send(void *data);

static void restart_dma_send(void) {
//	printk("performDMAsg, %d\n",count_send_plan);
	tx = performDMAsg();
//	printk("performedDMAsg, %d\n",count_send_plan);
	if(tx) {
		if(hackDMAsg(tx)) {
			#ifdef DEBUGSERVO
			if(!is_dumped_sunxi_sg_list) {
				dump_sunxi_sg_list(tx,0);
				is_dumped_sunxi_sg_list = true;
			}
			#endif
			tx->callback = cb_complete_dma_send;
			cookie = dmaengine_submit(tx);
			
//			dma_cache_sync(chan->device->dev, senddma_gpiobuf, senddma_gpiosyncbytes, DMA_TO_DEVICE);
			
			dma_async_issue_pending(chan);
			if(!is_start_dma_send) {
				is_start_dma_send = true;
			}
		} else {
			printk("DMA not hack sg\n");
		}
	} else {
		printk("DMA not perform sg\n");
	}
}

static void cb_complete_dma_send(void *data) {
/*
	enum dma_status status;
	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
	if(status!=DMA_SUCCESS) {
		printk("DMA status: %d\n",status);
	}
*/	
	if(sg_real_len) {
		dma_unmap_sg(chan->device->dev, sgl, sg_real_len, DMA_TO_DEVICE);
		sg_real_len = 0;
	}
	
	restart_dma_send();
}

static bool check_need_reinit(void) {
	bool checkUsePort[NUM_GPIO_REGISTER];
	int i,bank;
	bool r = false;

	for(i=0;i<NUM_GPIO_REGISTER;i++) {
		checkUsePort[i] = false;
	}
	for(i=0;i<NUM_PINS;i++) {
		if(pinGpio_Mode[i]==SERVO_MODE) {
			bank = get_bank_reg(i);
			checkUsePort[bank] = true;
		}
	}
	for(i=0;i<NUM_GPIO_REGISTER;i++) {
		if((checkUsePort[i] && !UseServoRegister[i]) || (!checkUsePort[i] && UseServoRegister[i])) {
			UseServoRegister[i] = checkUsePort[i];
			r = true;
		}
	}
	return r;
}

static int get_UseServoRegister(void) {
	int r = 0,i;
	for(i=0;i<NUM_GPIO_REGISTER;i++) {
		if(UseServoRegister[i]) {
			r++;
		}
	}
	return r;
}

// perform GPIO register data for SERVO and OUTPUT pins 
static void performGPIOBuf(void) {
	int i,j;
	c_pins = 0;
	for(i=0;i<NUM_PINS;i++) {
		if(pinGpio_Mode[i]==SERVO_MODE) {
			pins[c_pins++] = i;
		}
	}
	printk("Pins Sort: %d\n",c_pins);

	// sort pins over time
	for(i=0;i<c_pins-1;i++) {
		for(j=i+1;j<c_pins;j++) {
			if(pinGpio_Set[pins[j]]<pinGpio_Set[pins[i]]) {
				swap(pins[i],pins[j]);
			}
		}
	}
		
	//UseServoRegister_count
	c_regs = 0;
	for(i=0;i<NUM_GPIO_REGISTER;i++) {
		if(UseServoRegister[i]) {
			regs[c_regs++] = i;
		}
	}
}

static void performSendPlan(void) {
	int bank,bit,i,curr,last;
	int reg_values[NUM_GPIO_REGISTER];
	
	first_odd_gpio_send_bytes = (senddma_sequence_periode*100000/3)/senddma_uarttime1byte;
	count_send_plan = 0;

	memset(reg_values,0,sizeof(reg_values));
	for(i=0;i<NUM_PINS;i++) {
		bank = get_bank_reg(i);
		bit = get_bitoffset_reg(i);
		if((pinGpio_Mode[i]==OUTPUT_MODE || pinGpio_Mode[i]==SERVO_MODE) && UseServoRegister[bank]) {
			if(pinGpio_Set[i]>0) {
				reg_values[bank]|=BIT(bit);
			}
		}
	}

	// start fill data	
	for(i=0;i<c_regs;i++) {
		send_plan[count_send_plan].bank = get_bank_reg(regs[i]);
		send_plan[count_send_plan].value = reg_values[regs[i]];
		send_plan[count_send_plan].sleep = 0;
		count_send_plan++;
	}

	last = 0;	
	for(i=0;i<c_pins;i++) {
		bank = get_bank_reg(pins[i]);
		bit = get_bitoffset_reg(pins[i]);
		reg_values[bank]&=~BIT(bit);
		curr = pinGpio_Set[pins[i]];
		if(((curr-last)==0) && (send_plan[count_send_plan-1].bank==bank)) {
			send_plan[count_send_plan-1].value = reg_values[bank];
		} else {
			send_plan[count_send_plan-1].sleep+=(curr-last);
			if(send_plan[count_send_plan-1].sleep>senddma_sequence_periode) {
				send_plan[count_send_plan-1].sleep=senddma_sequence_periode;
			}
			send_plan[count_send_plan].bank = bank;
			send_plan[count_send_plan].value = reg_values[bank];
			send_plan[count_send_plan].sleep = 0;
			count_send_plan++;
		}
		last = curr;
	}
	if(senddma_sequence_periode-last>0) {
		send_plan[count_send_plan-1].sleep+=(senddma_sequence_periode-last);
		if(send_plan[count_send_plan-1].sleep>senddma_sequence_periode) {
			send_plan[count_send_plan-1].sleep=senddma_sequence_periode;
		}
	}

	#ifdef DEBUGSERVO
	dumpWeightPins();
	dumpSendPlan();
	#endif
}

// only change one pin servo value, resort priority
static void update_servopin(int pin) {
	int i,i_pin,n_pin;
	i_pin = -1;

	// sort pins over time
	for(i=0;i<c_pins;i++) {
		if(pin==pins[i]) {
			i_pin = i;
			break;
		}
	}
	if(i_pin<0) {
		printk("OpiServo: not find pin %d\n",pin);
		return;
	}
	n_pin = i_pin+1;
	while((n_pin<c_pins) && (pinGpio_Set[pins[n_pin]]<pinGpio_Set[pins[i_pin]])) {
		swap(pins[n_pin],pins[i_pin]);
		n_pin++;
		i_pin++;
	}
	n_pin = i_pin-1;
	while((n_pin>=0) && (pinGpio_Set[pins[n_pin]]>pinGpio_Set[pins[i_pin]])) {
		swap(pins[n_pin],pins[i_pin]);
		n_pin--;
		i_pin--;
	}
}

// check current port list and new if different
// stop dma and reinit dma buffer
// if not different need_reupdate = true
static bool reinit(void) {
	check_need_reinit();
	UseServoRegister_count = get_UseServoRegister();
	printk("Init DMA use servo register: %d\n",UseServoRegister_count);
	if(UseServoRegister_count) {
		performGPIOBuf();
		if(c_regs) {
			// probably must lock mutex, probably conflict with callback
			performSendPlan();
		}
	}
	if(!is_start_dma_send) {
		restart_dma_send();
	}
	#ifdef DEBUGSERVO
	is_dumped_sunxi_sg_list = false;
	#endif
	return true;
}

static void do_pin_command(int pin,char *val) {
	long servo_wl;
	if(strcmp(val,"INPUT")==0) {
		pinGpio_Mode[pin]=INPUT_MODE;
		pinMode(pin,INPUT_MODE);
	} else if(strcmp(val,"PUDUP")==0) {
		pullUpDnControl(pin, PUD_UP);
	} else if(strcmp(val,"PUDDOWN")==0) {
		pullUpDnControl(pin, PUD_DOWN);
	} else if(strcmp(val,"PUDOFF")==0) {
		pullUpDnControl(pin, PUD_OFF);
	} else if(strcmp(val,"OUTPUT")==0) {
		pinGpio_Mode[pin]=OUTPUT_MODE;
		pinGpio_Set[pin] = 0;
		pinMode(pin,OUTPUT_MODE);
		digitalWrite(pin,0);
		if(is_use_servo_port(pin)) {
			need_reupdate = true;
		}
	} else if(strcmp(val,"SERVO")==0) {
		pinGpio_Mode[pin]=SERVO_MODE;
		pinGpio_Set[pin] = 0;
		pinMode(pin,SERVO_MODE);
		need_reinit = true;
	} else if(strcmp(val,"T")==0 && pinGpio_Mode[pin]==OUTPUT_MODE) {
		pinGpio_Set[pin] = 1;
		if(is_use_servo_port(pin)) {
			need_reupdate = true;
		} else {
			digitalWrite(pin,1);
		}
	} else if(strcmp(val,"F")==0 && pinGpio_Mode[pin]==OUTPUT_MODE) {
		pinGpio_Set[pin] = 0;
		if(is_use_servo_port(pin)) {
			need_reupdate = true;
		} else {
			digitalWrite(pin,0);
		}
	} else if(strcmp(val,"-")==0 || strcmp(val,"NONE")==0) {
		if(is_use_servo_port(pin)) {
			if(pinGpio_Mode[pin]==SERVO_MODE) {
				need_reinit = true;
			} else {
				if(is_use_servo_port(pin)) {
					need_reupdate = true;
				}
			}
		}
		pinGpio_Mode[pin]=NONE_MODE;
		pinMode(pin,INPUT_MODE);
		pinGpio_Set[pin] = 0;
	} else if(pinGpio_Mode[pin]==SERVO_MODE && kstrtol(val,10,&servo_wl)==0) {
		pinGpio_Set[pin] = servo_wl;
		if(!need_reinit) {
			update_servopin(pin);
		}
		need_reupdate = true;
	} else {
		printk("Pin:%d=%s bad command\n",pin,val);
	}
}

static void update_command(char *data,int len) {
	char buf[MAX_BUF_VALUE];
	int pin;
	char val[MAX_BUF_VALUE];
	if(charn2str(data,len,buf,sizeof(buf)) && sscanf(buf, "%d=%s", &pin, val) == 2) {
		if(pin>=0 && pin<NUM_PINS) {
			#ifdef DEBUGSERVO
			printk("%d=%s\n",pin,val);
			#endif
			do_pin_command(pin,val);
		} else {
			printk("Bad num pins: %d\n",pin);
		}
	} else {
		printk("Bad query format\n");
	}
}

static void fill_get_result(struct private_data* const pdata) {
	int i;
	pdata->rd_len = 0;
	for(i=0;i<NUM_PINS;i++) {
		if(pinGpio_Mode[i]==INPUT_MODE || pinGpio_Mode[i]==OUTPUT_MODE) {
			pinGpio_Set[i] = digitalRead(i);
			pdata->rd_len += snprintf(
				pdata->rd_data+pdata->rd_len,
				sizeof(pdata->rd_data)-pdata->rd_len, 
				"%d=%s\n", i, pinGpio_Set[i]?"T":"F"
			);
		} else if(pinGpio_Mode[i]==SERVO_MODE) {
			pdata->rd_len += snprintf(
				pdata->rd_data+pdata->rd_len,
				sizeof(pdata->rd_data)-pdata->rd_len, 
				"%d=%d\n", i, pinGpio_Set[i]
			);
		}
	}
}

static void parse_and_update_command(struct private_data* const pdata) {
	int wr_complete = 0;
	char *p = pdata->wr_data, *start = p, *end = p + pdata->wr_len;
	need_reinit = false;
	need_reupdate = false;
	while(p<end) {
		if(*p == 0x0A) {
			update_command(start,p-start);
			wr_complete += (p-start)+1;
			start = p+1;
		}
		p++;
	}
	if(wr_complete>0) {
		memcpy(pdata->wr_data, pdata->wr_data+wr_complete, pdata->wr_len-wr_complete);
		pdata->wr_len-=wr_complete;
	}
	if(need_reinit || need_reupdate) {
		if(reinit()) {
			need_reupdate = false;
		}
	}
}

// kmalloc the temporary data required for each user:
static int dev_open(struct inode *inod, struct file *fil) {
	fil->private_data = kmalloc(sizeof(struct private_data), GFP_KERNEL);
	if (0 == fil->private_data)
	{
		printk(KERN_WARNING "OPIServo: Failed to allocate user data\n");
		return -ENOMEM;
	}
	memset(fil->private_data, 0, sizeof(struct private_data));
	return 0;
}

static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos) {
	ssize_t ret = 0;
	struct private_data* const pdata = filp->private_data;
	// Only proceed if we have private data, else return EOF.
	if (pdata) {
		if (0 == *f_pos) {
			fill_get_result(pdata);
		}
		if (*f_pos < pdata->rd_len) {
			if (count > pdata->rd_len - *f_pos)
				count = pdata->rd_len - *f_pos;
			if (copy_to_user(buf, pdata->rd_data + *f_pos, count))
				return -EFAULT;
			*f_pos += count;
			ret = count;
		}
	}
	return ret;
}

static ssize_t dev_write(struct file *filp,const char *buf,size_t count,loff_t *f_pos) {
	struct private_data* const pdata = filp->private_data;
	if (0 == pdata)
		return -EFAULT;
	if (count+pdata->wr_len > sizeof(pdata->wr_data) - 1)
		count = sizeof(pdata->wr_data) - pdata->wr_len;
	if (copy_from_user(pdata->wr_data+pdata->wr_len, buf, count))
		return -EFAULT;
	pdata->wr_len+= count;
	parse_and_update_command(pdata);

	return count;
}

static int dev_close(struct inode *inod,struct file *fil) {
	struct private_data* const pdata = fil->private_data;
	int ret = 0;
	if (pdata) {
		// Free process data.
		kfree(pdata);
	}

	return ret;
}

static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	return -EINVAL;
}

module_init(opiservo_kernel_init);
module_exit(opiservo_kernel_exit);

MODULE_DESCRIPTION("OPIServo, Multiple Servo Driver for the Orange Pi one");
MODULE_AUTHOR("Sergey Shkuliov <sergey.sckuliov@gmail.com>");
MODULE_LICENSE("GPL v2");

