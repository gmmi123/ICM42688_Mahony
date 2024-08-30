#include "icm42688.h"
#include "spi.h"
#include "MahonyAHRS.h"

#define correct_Time_define 1000    //上电去0飘 1000次取平均
/*接口*/
/*用户CS接口*/
#define SPI_SCL3300_CS_LOW() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,0)
#define	SPI_SCL3300_CS_HIGH() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,1)

/*SPI底层函数接口*/
uint8_t hal_Spi1_ReadWriteByte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 5); /*halstm32的spi读写函数*/
    return rxdata;
}

/*变量*/
/*Mahony变量*/
//uint8_t first_mahony=0; 
/*陀螺仪变量*/
float LSB_ACC_GYRO[2]={0};//陀螺仪数据转换变量
float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z  ;// ICM42688加速度原始数据       
float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z ; // ICM42688角速度原始速度数据
//float gyro_correct[3]={0};
//uint32_t correct_times=0;
/*面向ICM42688的spi读写函数封装*/
/*使用pBuffer进行数据交换*/
void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
{
    uint8_t i = 0;
    for(i = 0; i < len; i ++)
    {
		*pBuffer = hal_Spi1_ReadWriteByte(*pBuffer);
        pBuffer++;
    }

}
 
/*******************************************************************************
* 名    称： icm42688_read_reg
* 功    能： 读取单个寄存器的值
* 入口参数： reg: 寄存器地址
* 出口参数： 当前寄存器地址的值
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
*******************************************************************************/
uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    SPI_SCL3300_CS_LOW();
    reg |= 0x80;
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&regval, 1);
    SPI_SCL3300_CS_HIGH();
    return regval;
}
 
/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{

    reg |= 0x80;
    SPI_SCL3300_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(buf, len);
    SPI_SCL3300_CS_HIGH();

}
 
 
/*******************************************************************************
* 名    称： icm42688_write_reg
* 功    能： 向单个寄存器写数据
* 入口参数： reg: 寄存器地址 value:数据
* 出口参数： 0
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{

    SPI_SCL3300_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&value, 1);
    SPI_SCL3300_CS_HIGH();
    return 0;
}


/*另一个spi读取函数*/
/*addr为ICM42688寄存器地址,dat为接收的数据*/
void icm42688_readReg(uint8_t addr, uint8_t *dat)
{
	*dat=icm42688_read_reg(addr);
}
/*另一个spi写函数*/
/*addr为ICM42688寄存器地址,dat为要写入的数据*/
void icm42688_writeReg(uint8_t addr, uint8_t dat)
{
	icm42688_write_reg(addr,dat);
}


/*ICM42688的初始化*/
int8_t Icm42688_Init(void)
{

	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*软重启*/
	icm42688_writeReg(0x11,0x01);
	HAL_Delay(30);
	/*读取中断位 切换SPI*/
//	buf = IMU->ReadReg(0x2D);
	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*Gyro设置*/
	icm42688_writeReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	icm42688_writeReg(0x50,0x06);//16G 1KHz
	/*电源管理*/
	icm42688_writeReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
	
	HAL_Delay(30);

	
	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*中断输出设置*/
//	icm42688_writeReg(0x14,0x12);//INT1 INT2 脉冲模式，低有效
	/*Gyro设置*/
	icm42688_writeReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	icm42688_writeReg(0x50,0x06);//16G 1KHz
	/*LSB设置*/
	LSB_ACC_GYRO[0] = LSB_ACC_16G;
	LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem设置&Gyro_Config1*/
	icm42688_writeReg(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	icm42688_writeReg(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	icm42688_writeReg(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	icm42688_writeReg(0x63,0x00);//Null
	/*INT_CONFIG1*/
//	icm42688_writeReg(0x64,0x00);//中断引脚正常启用
	/*INT_SOURCE0*/
	icm42688_writeReg(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	icm42688_writeReg(0x66,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_writeReg(0x68,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_writeReg(0x69,0x00);//Null
	
/*****抗混叠滤波器@536Hz*****/
	
	/*GYRO抗混叠滤波器配置*/
	/*指定Bank1*/
	icm42688_writeReg(0x76,0x01);
	/*GYRO抗混叠滤波器配置*/
	icm42688_writeReg(0x0B,0xA0);//开启抗混叠和陷波滤波器
	icm42688_writeReg(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
	icm42688_writeReg(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
	icm42688_writeReg(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
	/*ACCEL抗混叠滤波器配置*/
	/*指定Bank2*/
	icm42688_writeReg(0x76,0x02);
	/*ACCEL抗混叠滤波器配置*/
	icm42688_writeReg(0x03,0x18);//开启滤波器 ACCEL_AFF_DELT 12 (default 24)
	icm42688_writeReg(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
	icm42688_writeReg(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

/*****自定义滤波器1号@111Hz*****/

	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*滤波器顺序*/
	icm42688_writeReg(0x51,0x12);//GYRO滤波器1st
	icm42688_writeReg(0x53,0x05);//ACCEL滤波器1st
	/*滤波器设置*/
	icm42688_writeReg(0x52,0x33);//111Hz 03
	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*电源管理*/
	icm42688_writeReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
	HAL_Delay(5000);
	return 0;
}


/*获取ICM42688的加速度原始值*/
void Get_Acc_ICM42688(void)
{
    unsigned char dat[6];
	icm42688_read_regs(ICM42688_ACCEL_DATA_X1,dat,6);
    icm42688_acc_x = (short int)(((short int)dat[0] << 8) | dat[1]);
    icm42688_acc_y = (short int)(((short int)dat[2] << 8) | dat[3]);
    icm42688_acc_z = (short int)(((short int)dat[4] << 8) | dat[5]);

}
/*获取ICM42688的角速度原始值*/
void Get_Gyro_ICM42688(void)
{
    unsigned char dat[6];
	icm42688_read_regs(ICM42688_GYRO_DATA_X1,dat,6);
    icm42688_gyro_x = (short int)(((short int)dat[0] << 8) | dat[1]);
    icm42688_gyro_y = (short int)(((short int)dat[2] << 8) | dat[3]);
    icm42688_gyro_z = (short int)(((short int)dat[4] << 8) | dat[5]);
}



void GetAngle(float* Roll,float* Pitch,float * Yaw)
{

	static float temp[3]={0,0,0};
	Get_Acc_ICM42688();//获取陀螺仪原始数据
	Get_Gyro_ICM42688();
	MahonyAngle(icm42688_acc_x,icm42688_acc_y,icm42688_acc_z,
				icm42688_gyro_x,icm42688_gyro_y,icm42688_gyro_z,16,2000,temp);
	*Roll = temp[0];
	*Pitch = temp[1];
	*Yaw = temp[2];
		
}


