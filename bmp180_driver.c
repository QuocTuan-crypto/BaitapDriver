#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#define DRIVER_NAME "bmp180_ioctl"
#define CLASS_NAME "bmp180"
#define DEVICE_NAME "bmp180"
#define BMP180_IOCTL_MAGIC 'b'
#define BMP180_IOCTL_READ_TEMPERATURE _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESSURE _IOR(BMP180_IOCTL_MAGIC, 2, int)

#define BMP180_CALIB_START  0xAA
#define BMP180_CALIB_BYTES  22
#define BMP180_CMD_TEMPERATURE  0x2E
#define BMP180_CMD_PRESSURE 0x34
#define BMP180_REG_CONTROL  0xF4
#define BMP180_REG_RESULT   0xF6
#define OSS 0  

static struct i2c_client *bmp180_client;
static struct class* bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int bmp180_major_number;
// Hàm đọc nhiệt độ và áp suất
static int bmp180_get_temp_press(struct i2c_client *client, long *temperature, long *pressure)
{
    int ret;
    u8 calib[BMP180_CALIB_BYTES];
    s16 AC1, AC2, AC3, B1, B2, MB, MC, MD;
    u16 AC4, AC5, AC6;
    u8 buf[3];
    s16 UT;
    s32 X1, X2, X3, B5,B6,T;
    s32 UP, B3;
    u32 B4;
    u64 B7,P;
    // Đọc hệ số hiệu chuẩn từ EEPROM
    ret = i2c_smbus_read_i2c_block_data(client, BMP180_CALIB_START, BMP180_CALIB_BYTES, calib);
    if (ret < 0) return ret;
    // Ghép byte cao và byte thấp thành giá trị 16 bit
    AC1 = (calib[0] << 8) | calib[1]; 
    AC2 = (calib[2] << 8) | calib[3];
    AC3 = (calib[4] << 8) | calib[5];
    AC4 = (calib[6] << 8) | calib[7];
    AC5 = (calib[8] << 8) | calib[9];
    AC6 = (calib[10] << 8) | calib[11];
    B1  = (calib[12] << 8) | calib[13];
    B2  = (calib[14] << 8) | calib[15];
    MB  = (calib[16] << 8) | calib[17];
    MC  = (calib[18] << 8) | calib[19];
    MD  = (calib[20] << 8) | calib[21];
    // In tất cả giá trị hệ số hiệu chuẩn
    pr_info("calib: ");
    for (int i = 0; i < BMP180_CALIB_BYTES; i++) {
        pr_info("%02X ", calib[i]);
    }
    // Đọc nhiệt độ chưa bù (UT)
    ret = i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, BMP180_CMD_TEMPERATURE);
    if (ret < 0) return ret;
    msleep(5);
    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_RESULT, 2, buf);
    if (ret < 0) return ret;
    UT = (buf[0] << 8) | buf[1];
    // Tính nhiệt độ 
    X1 = ((UT - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    T  = (B5 + 8) >> 4;  // Đơn vị 0.1 C
    *temperature = T ; 
    // Đọc áp suất chưa bù (UP)
    ret = i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, BMP180_CMD_PRESSURE + (OSS << 6));
    if (ret < 0) return ret;
    msleep(5 + (3 << OSS));
    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_RESULT, 3, buf);
    if (ret < 0) return ret;
    UP = (((s32)buf[0] << 16) | ((s32)buf[1] << 8) | buf[2]) >> (8 - OSS);
    // Tính áp suất 
    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((((s32)AC1) * 4 + X3) << OSS) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (u32)(X3 + 32768)) >> 15;
    B7 = ((u32)UP - B3) * (50000 >> OSS);
    if (B7 < 0x80000000ULL)
        {P = B7 << 1;
        do_div(P, B4);
        }
    else
        {P = B7;
        do_div(P, B4);
        P = P << 1;}
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);
    *pressure = P;   // đơn vị PA 
    pr_info("t=%d P=%llu\n", T,P);
    return 0;
}
static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{    
    long result = 0;
    long temp, press;

    switch (cmd) {
        case BMP180_IOCTL_READ_TEMPERATURE:
            // Gọi hàm lấy nhiệt độ và áp suất
            bmp180_get_temp_press(bmp180_client, &temp, &press);
            result = (long)(temp); 
            break;
        case BMP180_IOCTL_READ_PRESSURE:
            bmp180_get_temp_press(bmp180_client, &temp, &press);
            result = (long)(press); 
            break;
        default:
            return -EINVAL;
    }
    // Copy giá trị trả về vào bộ nhớ người dùng
    if (copy_to_user((long __user *)arg, &result, sizeof(result))) {
        return -EFAULT;
    }

    return 0;
}

static int bmp180_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "bmp180 device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "bmp180 device closed\n");
    return 0;
}

static struct file_operations bmp180_fops = {
    .open = bmp180_open,
    .unlocked_ioctl = bmp180_ioctl,
    .release = bmp180_release,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    bmp180_client = client;
    // Đăng ký thiết bị ký tự
    bmp180_major_number = register_chrdev(0, "bmp180", &bmp180_fops);
    if (bmp180_major_number < 0) {
        printk(KERN_ERR "BMP180: Failed to register a major number\n");
        return bmp180_major_number;
    }
    printk(KERN_INFO "BMP180: Major number registered: %d\n", bmp180_major_number);
    // Tạo class /sys/class/bmp180
    bmp180_class = class_create(THIS_MODULE, "bmp180");
    if (IS_ERR(bmp180_class)) {
        printk(KERN_ERR "BMP180: Failed to create device class\n");
        unregister_chrdev(bmp180_major_number, "bmp180");
        return PTR_ERR(bmp180_class);
    }
    printk(KERN_INFO "BMP180: Device class created\n");
    // Tạo thiết bị /dev/bmp180
    bmp180_device = device_create(bmp180_class, NULL, MKDEV(bmp180_major_number, 0), NULL, "bmp180");
    if (IS_ERR(bmp180_device)) {
        printk(KERN_ERR "BMP180: Failed to create device\n");
        class_destroy(bmp180_class);  
        unregister_chrdev(bmp180_major_number, "bmp180"); 
        return PTR_ERR(bmp180_device);
    }
    printk(KERN_INFO "BMP180: Driver probed and device created successfully\n");
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    printk(KERN_INFO "bmp180_remove: Cleaning up resources\n");
    device_destroy(bmp180_class, MKDEV(bmp180_major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(bmp180_major_number, "bmp180");
    printk(KERN_INFO "BMP180: Driver removed\n");
}

static const struct i2c_device_id bmp180_id[] = {
    { "bmp180", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, bmp180_id);
static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "invensense,bmp180", },
    { },
};
MODULE_DEVICE_TABLE(of, bmp180_of_match);

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp180_of_match),
    },
    .probe      = bmp180_probe,
    .remove     = bmp180_remove,
};

static int __init bmp180_init(void)
{
    printk(KERN_INFO "Initializing bmp180 driver\n");
    return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
    printk(KERN_INFO "Exitingbmp180 driver\n");
    i2c_del_driver(&bmp180_driver);
}
module_init(bmp180_init);
module_exit(bmp180_exit);
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("bmp180 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("GPL");
