/*************************************************************************
    > File Name: serial.c
    > Author: wyg
    > Mail: wyg_0802@126.com 
    > Created Time: 2016年12月16日 星期五 08时56分30秒
 ************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>

#define debug(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define info(fmt, ...) printk(KERN_INFO fmt, ##__VA_ARGS__)
#define DEV_NAME "serial"
#define DEV_NUM 2
#define DEV_REG_NUM 8
#define CLASS_NAME "serial"
#define FIFO_SIZE 16
#define DEFUALT_SPEED 115200
#define WORD_LEN_5 0
#define WORD_LEN_6 1
#define WORD_LEN_7 2
#define WORD_LEN_8 3
#define STOP_BIT_1 0
#define STOP_BIT_X 1
#define PARITY_NO  0
#define PARITY_ODD 1
#define PARITY_EVEN 3
#define PARITY_HIGH 5
#define PARITY_LOW 7

#define RBR 0
#define THR 0
#define DLL 0
#define DLM 1
#define IER 1
#define IIR 2
#define FCR 2
#define LCR 3
#define MCR 4
#define LSR 5
#define MSR 6
#define SCR 7

#define SPEED_NUM 10
struct serial_speed
{
	int speed;
	unsigned char dll;
	unsigned char dlm;
};
static struct serial_speed speed_table[SPEED_NUM] = {
	{50,	0x00, 0x09},	{300,	0x80, 0x01},
	{1200,	0x60, 0x00},	{2400,	0x30, 0x00},
	{4800,	0x18, 0x00},	{9600,	0x0c, 0x00},
	{19200, 0x06, 0x00},	{38400,	0x03, 0x00},
	{57600, 0x02, 0x00},	{115200,0x01, 0x00},
};
static int major = 0;
static dev_t devno;
static struct class *serial_class;
static struct cdev dev;
struct serial_device
{
	struct device *dvs;
	char *name;
	int base;
	int irq;
	int minor;
	int speed;
	atomic_t atmc_open;
	spinlock_t splock;
	unsigned word_len:2;
	unsigned stop_bit:1;
	unsigned parity:3;
	struct task_struct *tsk;
	int stop;
	struct semaphore sem_read;
};
static struct serial_device sdev[2] = 
{
	[0]			=
				{
					.name		=	DEV_NAME"0",
					.base		=	0x2f8,
					.irq		=	3,
					.atmc_open	=	ATOMIC_INIT(1),
					.speed		=	DEFUALT_SPEED,
					.word_len	=	WORD_LEN_8,
					.stop_bit	=	STOP_BIT_1,
					.parity		=	PARITY_NO,
				},
	[1]			=
				{
					.name		=	DEV_NAME"1",
					.base		=	0x3f8,
					.irq		=	4,
					.atmc_open	=	ATOMIC_INIT(1),
					.speed		=	DEFUALT_SPEED,
					.word_len	=	WORD_LEN_8,
					.stop_bit	=	STOP_BIT_1,
					.parity		=	PARITY_NO,
				},
};
#define CLEAR_FIFO(ser_dev) outb(0x027, ser_dev->base + FCR)
#define DISABLE_FIFO(ser_dev) outb(0x00, ser_dev->base + FCR)
#define CAN_READ(ser_dev) (inb(ser_dev->base + LSR) & 0x1)
#define CAN_WRITE(ser_dev) (inb(ser_dev->base + LSR) & 0x20)
#define HAVE_ERROR(ser_dev) (inb(ser_dev->base + LSR) & 0x20)
#define INT_RECV(ser_dev) (inb(ser_dev->base + IIR)& 0x40)
#define INT_SEND(ser_dev) (inb(ser_dev->base + IIR)& 0x20)
#define READB(ser_dev)	(inb(ser_dev->base + RBR))
#define WRITEB(ser_dev, c)	(outb(c, ser_dev->base + THR))
#define HAVE_DEV(ser_dev) (!(inb(ser_dev->base + LCR) == 0xff))
static inline void set_lcr(struct serial_device *ser_dev)
{
	outb( ((ser_dev->parity << 3) | (ser_dev->word_len << 2) | (ser_dev->word_len))&0x3f, ser_dev->base + LCR);
}
static inline void set_speed(struct serial_device *ser_dev)
{
	int i;
	unsigned char old_lcr;
	for(i = 0; i < SPEED_NUM; i++){
		if(speed_table[i].speed == ser_dev->speed)
			break;
	}
	if(i == SPEED_NUM){
		return;
	}
	old_lcr = inb(ser_dev->base + LCR);
	outb(old_lcr | 0x80, ser_dev->base+LCR);
	outb(speed_table[i].dll, ser_dev->base+DLL);
	outb(speed_table[i].dlm, ser_dev->base+DLM);
	outb(old_lcr, ser_dev->base+LCR);
}
static inline void enable_send_intr(struct serial_device *ser_dev)
{
	unsigned char old_ier;
	old_ier = inb(ser_dev->base + IER);
	old_ier |= 0x02;
	outb(old_ier, ser_dev->base + IER);
	outb(0x08, ser_dev->base + MCR);
}
static inline void disable_send_intr(struct serial_device *ser_dev)
{
	unsigned char old_ier;
	old_ier = inb(ser_dev->base + IER);
	old_ier &= ~0x02;
	outb(old_ier, ser_dev->base + IER);
	outb(0x08, ser_dev->base + MCR);
}

static inline void enable_recv_intr(struct serial_device *ser_dev)
{
	unsigned char old_ier;
	old_ier = inb(ser_dev->base + IER);
	old_ier |= 0x01;
	outb(old_ier, ser_dev->base + IER);
	outb(0x08, ser_dev->base + MCR);
}
static inline void disable_recv_intr(struct serial_device *ser_dev)
{
	unsigned char old_ier;
	old_ier = inb(ser_dev->base + IER);
	old_ier &= ~0x01;
	outb(old_ier, ser_dev->base + IER);
	outb(0x08, ser_dev->base + MCR);
}
static void  init_serial(struct serial_device *ser_dev)
{
	int i;
	disable_recv_intr(ser_dev);
	disable_send_intr(ser_dev);
	set_speed(ser_dev);
	set_lcr(ser_dev);
	CLEAR_FIFO(ser_dev);
	printk("Base:%x\n", ser_dev->base);
	for(i = 0; i < DEV_REG_NUM; i++){
		printk("reg%d: %x\n", i, inb(ser_dev->base + i));
	}
}
char tmp_buf[1024*1024];
int cnt = 0;
static int serial_read_thread(void *data)
{

	int ret = 0;
	struct serial_device *ser_dev = (struct serial_device *)data;
	while(!ser_dev->stop)
	{

		while(HAVE_DEV(ser_dev) && CAN_READ(ser_dev)){
			tmp_buf[cnt] =  READB(ser_dev);
			if(cnt<1024*1024-1)
				cnt++;
		}
	/*	if(HAVE_ERROR(ser_dev)){
			debug("Error:overrun.\n");
			CLEAR_FIFO(ser_dev);
		}
		*/
		/*
		WRITEB(ser_dev, 'a');
		if(HAVE_ERROR(ser_dev)){
			debug("Error:overrun.\n");
		}
		debug("\nserial_read_thread running.\n");
		*/
		ret = down_timeout(&ser_dev->sem_read, HZ/20);
	}
	return 0;
}
static irqreturn_t serial_irq_handler(int irq, void *dev_id)
{
	struct serial_device *ser_dev = (struct serial_device*)dev_id;
	if(INT_RECV(ser_dev)){
		up(&ser_dev->sem_read);
	}
	return IRQ_HANDLED;
}
static int serial_open(struct inode *ind, struct file *flp)
{
	int ret;
	cnt = 0;
	struct serial_device *ser_dev = &sdev[MINOR(ind->i_rdev)];
	if(!atomic_dec_and_test(&ser_dev->atmc_open)){
		atomic_inc(&ser_dev->atmc_open);
		return -EBUSY;
	}
	flp->private_data = ser_dev;
	init_serial(ser_dev);
	if(inb(ser_dev->base + LSR) == 0xff){
		ret = -ENODEV;
		goto failed1;
	}
	ret = request_irq(ser_dev->irq, serial_irq_handler, IRQF_SHARED|IRQF_TRIGGER_RISING, ser_dev->name, ser_dev);
	if(ret < 0){
		ret = -EBUSY;
		goto failed1;
	}

	ser_dev->stop = 0;
	ser_dev->tsk = kthread_run(serial_read_thread, ser_dev, "serial read");
	if(ser_dev->tsk == NULL){
		debug("kthread run failed.\n");
		ret = -ENOMEM;
		goto failed2;
	}
	printk("ser_dev:%p\n", ser_dev);
	enable_recv_intr(ser_dev);
	return 0;
failed2:
	free_irq(ser_dev->irq, ser_dev);
failed1:
	atomic_inc(&ser_dev->atmc_open);
	return ret;
}
static int serial_release(struct inode *ind, struct file *flp)
{
	struct serial_device *ser_dev = &sdev[MINOR(ind->i_rdev)];
	DISABLE_FIFO(ser_dev);
	disable_send_intr(ser_dev);
	disable_recv_intr(ser_dev);
	free_irq(ser_dev->irq, ser_dev);
	ser_dev->stop = 1;
	kthread_stop(ser_dev->tsk);
	atomic_inc(&ser_dev->atmc_open);
	tmp_buf[cnt] = 0;
	printk("%s\n", tmp_buf);
	printk("recv %d bytes\n", cnt);
	return 0;
}
static struct file_operations serial_fops = 
{
	.owner		=	THIS_MODULE,
	.open		=	serial_open,
	.release	=	serial_release,
};

static int __init serial_init(void)
{
	int i;
	int ret = 0;
	if(major != 0){
		devno = MKDEV(major, 0);
		ret = register_chrdev_region(devno, DEV_NUM, DEV_NAME);
	}else{
		devno = 0;
		ret = alloc_chrdev_region(&devno, 0, DEV_NUM, DEV_NAME);
	}
	if(ret < 0){
		info("register chrdev region failed\n");
		goto failed1;
	}else{
		major = MAJOR(devno);
		sdev[0].minor = 0;
		sdev[1].minor = 1;
	}
	cdev_init(&dev, &serial_fops);
	dev.owner = THIS_MODULE;
	ret = cdev_add(&dev, devno, DEV_NUM);
	if(ret < 0){
		info("cdev add failed.\n");
		goto failed2;
	}

	serial_class = class_create(THIS_MODULE, CLASS_NAME);
	if(IS_ERR(serial_class)){
		info("serial class create failed.\n");
		goto failed3;
	}
	
	for(i = 0; i < DEV_NUM; i++){
		spin_lock_init(&sdev[i].splock);
		request_region(sdev[i].base, DEV_REG_NUM, sdev[i].name);
		sema_init(&sdev[i].sem_read, 0);
		sdev[i].dvs = device_create(serial_class, NULL, devno+sdev[i].minor, &dev, sdev[i].name);
		if(IS_ERR(sdev[i].dvs)){
			sdev[i].dvs = NULL;
			goto failed4;
		}
	}
	info("serial inited.\n");
	return 0;
failed4:
	class_destroy(serial_class);
failed3:
	cdev_del(&dev);
failed2:
	unregister_chrdev_region(devno, DEV_NUM);
failed1:
	return ret;
}

static void __exit serial_exit(void)
{
	int i;
	for(i = 0; i < DEV_NUM; i++){
		release_region(sdev[i].base, DEV_REG_NUM);
		if(NULL != sdev[i].dvs){
			device_destroy(serial_class, devno+sdev[i].minor);
		}
	}
	class_destroy(serial_class);
	cdev_del(&dev);
	unregister_chrdev_region(devno, DEV_NUM);
	info("serial exited.\n");
}

module_init(serial_init);
module_exit(serial_exit);
MODULE_LICENSE("GPL");
