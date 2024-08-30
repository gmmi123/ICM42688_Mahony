#include "MahonyAHRS.h"
#include "dsp/fast_math_functions.h"    
#include "arm_math.h"
/*Mahony使用*/
/*
第一种情况：陀螺仪已经转换原始数据并且校准
1.确保已经加载DSP库
2.在主函数中调用一次Mahony_Init()函数,函数的参数为：更新周期(单位:us)
3.启用一个中断函数，中断周期为更新周期，在中断中调用Mahony_update()函数进行更新
MahonyAngle()传入的6轴数据需为校准后的数据


第二种情况：已有陀螺仪原始数据但未校准
1.确保已经加载DSP库
2.在主函数中调用一次Mahony_Init()函数,函数的参数为：更新周期(单位:us)
3.启用一个中断函数，中断周期为更新周期，在中断中调用MahonyAngle()函数进行更新
MahonyAngle()传入的6轴数据需为原始数据,
*/


//CorrectDate函数:
/*陀螺仪原始数据转换函数*/
/*ax,ay,az,gx,gy,gz为陀螺仪的6轴数据*/
/*ACCrange为加速度的满量程，如16g满量程，填16*/   //详见:https://blog.csdn.net/cyj972628089/article/details/113293682
/*GYROrange为角速度的满量程，如2000满量程，填2000*/
/*Date为转化后的数据数组，依次对应为ax,ay,az,gx,gy,gz*/

void CorrectDate(float ax,float ay,float az,
				 float gx,float gy,float gz,
			     float ACCrange,float GYROrange,float* Date)
{
	Date[0]=ax*ACCrange*9.8/32768;
	Date[1]=ay*ACCrange*9.8/32768;
	Date[2]=az*ACCrange*9.8/32768;
	Date[3]=gx*GYROrange/32768*3.1415926/180;
	Date[4]=gy*GYROrange/32768*3.1415926/180;
	Date[5]=gz*GYROrange/32768*3.1415926/180;
//	Date[0]=ax*0.0047856934f;
//	Date[1]=ay*0.0047856934f;/*已确定量程可以提前算出来，减少计算*/
//	Date[2]=az*0.0047856934f;
//	Date[3]=gx*0.0010652644f;
//	Date[4]=gy*0.0010652644f;
//	Date[5]=gz*0.0010652644f;
}

/*校准陀螺仪并结算角度*/
/*
传入标准数据，调用roll_mahony,pitch_mahony,yaw_mahony使用角度或数组
*/
void Get_MahonyAngle(float ax,float ay,float az,float gx,float gy,float gz,float* Date)
{
	
	static uint32_t correct_Time_define =1000;//1000次零飘处理
	static char first_mahony = 0;//第一次计算标志位
	
	static char State=1;
	static float gyro_corrects[3]={0,0,0,};
	static uint32_t correct_times=0;
	if(State==1)//角速度零飘处理
	{
		gyro_corrects[0]+= gx;
		gyro_corrects[1]+= gy;
		gyro_corrects[2]+= gz;
		correct_times++;
		if(correct_times>=correct_Time_define)
		{
		  gyro_corrects[0]/=correct_Time_define;
		  gyro_corrects[1]/=correct_Time_define;
		  gyro_corrects[2]/=correct_Time_define;
		  State=2; //go to 2 state
		}
	
	}
	if(State==2)
	{

		gx-=gyro_corrects[0];//角速度数据减去零飘
		gy-=gyro_corrects[1];
		gz-=gyro_corrects[2];
		if(first_mahony==0)//Mahony初始化
		{
		  first_mahony++;
		  MahonyAHRSinit(ax,ay,az,0,0,0);
		}
		/*传入标准数据，进行数据更新*/
		Mahony_update(gx,gy,gz,ax,ay,az,0,0,0);
		Mahony_computeAngles();//计算角度
		Date[0] = roll_mahony;
		Date[1] = pitch_mahony;
		Date[2] = yaw_mahony;
	}

}

//
void MahonyAngle(float ax,float ay,float az,
				 float gx,float gy,float gz,
			     float ACCrange,float GYROrange,float* Date)
{
	static float Temp[6]={0,0,0,0,0,0};//暂存转换数据
	CorrectDate(ax,ay,az,gx,gy,gz,ACCrange,GYROrange,Temp);//转换原始数据
	Get_MahonyAngle(Temp[0],Temp[1],Temp[2],Temp[3],Temp[4],Temp[5],Date);//校准并转换角度

}


//-------------------------------------------------------------------------------------------
// Definitions
//这里都是可以调整的参数
//---------------------------------------***********************************
float twoKi;		// 2 * integral gain (Ki)
float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
float invSampleFreq;
float roll_mahony, pitch_mahony, yaw_mahony;
char anglesComputed;
//*-*-*-**-----------------------------------------------------------------------------
static float invSqrt(float x)  // if use other platform please use float Mahony_invSqrt(float x)
{
	volatile float tmp = 1.0f;
	tmp /= __sqrtf(x);
	return tmp;
}


#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
void Mahony_Init(float sampleFrequency)
{
	twoKi = twoKiDef;	// 2 * integral gain (Ki)
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	anglesComputed = 0;
	invSampleFreq = 1.0f / sampleFrequency;
}



float Mahony_invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) 
    {
	    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	    mx *= recipNorm;
	    my *= recipNorm;
	    mz *= recipNorm;
	}

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
    	magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
    	magY = my * cos_roll - mz * sin_roll;
    	init_yaw  = atan2f(-magY, magX);
    }
    else
    {
    	init_yaw=0.0f;
    }

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1= sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3= cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Convert gyroscope degrees/sec to radians/sec
//	gx *= 0.0174533f;
//	gy *= 0.0174533f;
//	gz *= 0.0174533f;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex  * invSampleFreq;	// integral error scaled by Ki
            integralFBy += twoKi * halfey  * invSampleFreq;
            integralFBz += twoKi * halfez  * invSampleFreq;
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f *   invSampleFreq);		// pre-multiply common factors
    gy *= (0.5f  * invSampleFreq);
    gz *= (0.5f  * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_computeAngles()
{
	arm_atan2_f32(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2,&roll_mahony);
	roll_mahony *= 57.29578f;  
	pitch_mahony =57.29578f * asinf(-2.0f * (q1*q3 - q0*q2));
	arm_atan2_f32(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3,&yaw_mahony); 
	yaw_mahony *=57.29578f;
	anglesComputed = 1;
}
float getRoll() {
	if (!anglesComputed) Mahony_computeAngles();
	return roll_mahony;
}
float getPitch() {
	if (!anglesComputed) Mahony_computeAngles();
	return pitch_mahony;
}
float getYaw() {
	if (!anglesComputed) Mahony_computeAngles();
	return yaw_mahony;
}
//============================================================================================
// END OF CODE
//============================================================================================
