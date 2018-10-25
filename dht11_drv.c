#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/uaccess.h>

typedef unsigned char u8;
typedef unsigned short u16;

static int dht11_major = 0;
static int dht11_gpio;
static struct class *dht11_class;//类
static struct device *dht11_dev;//设备
static const char* dht11_name = "dht11";


typedef struct	DHT11_SENSOR_DATA
{
	u16 temp;//温度
	u16 hum;//湿度
}dht11_data;

#define DHT11_DQ_High    gpio_direction_output(dht11_gpio, 1)
#define DHT11_DQ_Low     gpio_direction_output(dht11_gpio, 0)
#define DHT11_IO_IN		 gpio_direction_input(dht11_gpio)


#define delay_us(x)		udelay(x)
#define delay_ms(x)		msleep(x)

static u8 DHT11_Read_DQ(void)    
{
	DHT11_IO_IN;
	return gpio_get_value(dht11_gpio);
}

//复位DHT11
static void DHT11_Rst(void)   
{                 
	DHT11_DQ_Low;
	msleep (20);  //拉低至少18ms
	DHT11_DQ_High; //DQ=1 
	delay_us (30);  //主机拉高20~40us
}

//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
static u8 DHT11_Check(void)    
{   
	u8 retry=0;//定义临时变量
	DHT11_IO_IN;//SET INPUT 
	while ((DHT11_Read_DQ()==1)&&retry<100)//DHT11会拉低40~80us
	{
		retry++;
		delay_us(1);
	}; 
	if(retry>=100)return 1;
	else retry=0;
	while ((DHT11_Read_DQ()==0)&&retry<100)//DHT11拉低后会再次拉高40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;    
	return 0;
}

//从DHT11读取一个位
//返回值：1/0
static u8 DHT11_Read_Bit(void)  
{
	u8 retry=0;
	while((DHT11_Read_DQ()==1)&&retry<100)//等待变为低电平
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while((DHT11_Read_DQ()==0)&&retry<100)//等待变高电平
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);//等待40us
	if(DHT11_Read_DQ()==1)
	return 1;
	else 
	return 0;   
}
//从DHT11读取一个字节
//返回值：读到的数据
static u8 DHT11_Read_Byte(void)    
{        
    u8 i,dat;
    dat=0;
	for (i=0;i<8;i++) 
	{
		dat<<=1; 
		dat|=DHT11_Read_Bit();
	}    
	return dat;
}
 
//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
static u8 DHT11_Read_Data(u16 *temp,u16 *humi)    
{        
	u8 buf[5];
	u8 i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//读取40位数据
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0]<<8|buf[1];
			*temp=buf[2]<<8|buf[3];
			printk("buf=%d,%d,%d,%d,%d\n",buf[0],buf[1],buf[2],buf[3],buf[4]);
		}
	}else return 1;
	return 0;    
}
//初始化DHT11的IO口 DQ 同时检测DHT11的存在
//返回1:不存在
//返回0:存在     
static void DHT11_Init(void)
{     
	DHT11_Rst();  //复位DHT11
	DHT11_Check();//等待DHT11的回应
}

int DHT11_open(struct inode *inode, struct file *flips)
{
	printk("--------------%s--------------\n",__FUNCTION__);
	return 0;
}

static ssize_t DHT11_read(struct file *file, char __user *buf,
		    size_t nbytes, loff_t *ppos)
{	
	printk("--------------%s--------------\n",__FUNCTION__);
	
	dht11_data Last_dht11_data;
	if(DHT11_Read_Data(&Last_dht11_data.temp,&Last_dht11_data.hum) == 0);//读取温湿度值 
	{
		if ( copy_to_user(buf,&Last_dht11_data,sizeof(Last_dht11_data)) )
		{
			return EFAULT ;
		}
	}

}

static int DHT11_close(struct inode *inode, struct file *flip)
{
	printk("--------------%s--------------\n",__FUNCTION__);
	return 0;
}

static struct file_operations dht11_fops = {
	.owner = THIS_MODULE,
	.read = DHT11_read,
	.open = DHT11_open,
	.release = DHT11_close,
};

static const struct of_device_id of_dht11_match[] = {
	{ .compatible = "mygpio-leds", },
	{},
};

MODULE_DEVICE_TABLE(of, of_dht11_match);



static int dht11_probe(struct platform_device *pdev)
{
	int ret;  
	enum of_gpio_flags flag;//(flag == OF_GPIO_ACTIVE_LOW) ?

	printk("-------%s-------------\n", __FUNCTION__);
	
    struct device_node *dht11_gpio_node = pdev->dev.of_node; 
	
	dht11_gpio = of_get_named_gpio_flags(dht11_gpio_node->child, "gpios", 0, &flag); 

	if (!gpio_is_valid(dht11_gpio)) 
	{
    	printk("dht11-gpio: %d is invalid\n", dht11_gpio); 
		return -ENODEV;
    }
	else
		printk("dht11-gpio: %d is valid!\n", dht11_gpio);
	if (gpio_request(dht11_gpio, "dht11-gpio")) 
	{ 
        printk("gpio %d request failed!\n", dht11_gpio); 
        gpio_free(dht11_gpio); 
        return -ENODEV;
    }
	else
		printk("gpio %d request success!\n", dht11_gpio); 
	//能够读到配置信息之后就可以开始创建设备节点
	dht11_major = register_chrdev(0, "dht11",&dht11_fops);
	if(dht11_major <0)
	{
		printk(KERN_ERR "reg error!\n");
		goto err_0;		
	}
	else 
		printk("dht11_major =%d\n",dht11_major);
	
	dht11_class = class_create(THIS_MODULE,"dht11_class");//creat class
	if( IS_ERR(dht11_class))
	{
		printk(KERN_ERR "fail create class\n");
		ret = PTR_ERR(dht11_class);
		goto err_1;
	}
	
	//creat dht11_dev--->>/dev/dht11_dev
	dht11_dev = device_create(dht11_class, NULL,MKDEV(dht11_major,0), NULL, dht11_name);
	if(IS_ERR(dht11_dev))
	{
		printk(KERN_ERR "fail create device!\n");
		ret = PTR_ERR(dht11_dev);
		goto err_2;		
	}
	
	//init dht11
	DHT11_Init();
	printk("dht11 Initing...\n");
	return 0;
	
err_2:
		device_destroy(dht11_class,MKDEV(dht11_major,0));
err_1:
		class_destroy(dht11_class);
err_0:
		unregister_chrdev(dht11_major,dht11_name);	
		return -1;
}

static int dht11_remove(struct platform_device *pdev)
{
	printk("-------%s-------------\n", __FUNCTION__);
	device_destroy(dht11_class,MKDEV(dht11_major,0));
	class_destroy(dht11_class);
	unregister_chrdev(dht11_major,dht11_name);
	return 0;
}

static void dht11_shutdown(struct platform_device *pdev)
{
	printk("-------%s-------------\n", __FUNCTION__);
}

static struct platform_driver dht11_driver = {
	.probe		= dht11_probe,
	.remove		= dht11_remove,
	.shutdown	= dht11_shutdown,
	.driver		= {
		.name	= "dht11_driver",
		.of_match_table = of_dht11_match,
	},
};

module_platform_driver(dht11_driver);

MODULE_AUTHOR("jackeyt,bbs.elecfans.com");
MODULE_DESCRIPTION("DHT11 Sensor driver");
MODULE_LICENSE("GPL");

