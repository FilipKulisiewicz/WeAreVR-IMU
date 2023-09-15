#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>    
#include <cstdlib>
#include <chrono>

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
    
static int fd, s_iCurBaud = 921600;
static volatile char s_cDataUpdate = 0;

const int c_uiBaud[] = {921600 , 4800 , 9600 , 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};

static void AutoScanSensor(char* dev);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

int main(int argc,char* argv[]){
	if(argc < 2)
	{
		printf("please input dev name\n");
		return 0;
	}

    if((fd = serial_open((const char*)argv[1] , 921600)<0))
	 {
	     printf("open %s fail\n", argv[1]);
	     return 0;
	 }
	else printf("open %s success\n", argv[1]);

	float fAcc[3], fGyro[3], fAngle[3];
	char cBuff[1];
	time_t time;
				
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	
	printf("\r\n********************** wit-motion Normal example  ************************\r\n");
	AutoScanSensor(argv[1]);
	printf("Chiptime[ms]	ax[g]	ay[g]	az[g]	wx[rad/s]	wy[rad/s]	wz[rad/s]	magx	magy	magz \r\n");

	auto timeStamp = std::chrono::steady_clock::now();
	const std::chrono::duration<double>  WAIT_TIME(0.002f);
	
	struct tm t;
	time_t time_since_epoch;
			
	while(1)
	{	
		while(serial_read_data(fd, (unsigned char*)cBuff, 1))
		{
			WitSerialDataIn(cBuff[0]);
		}

		while ((std::chrono::steady_clock::now() - timeStamp) < WAIT_TIME){
			continue;
		}

		timeStamp = std::chrono::steady_clock::now();

		if(s_cDataUpdate)
		{
		
			for(int i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = (sReg[GX+i] / 32768.0f * 2000.0f) * M_PI / 180.0f; //[rad/s]
				// fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;				
			}
			t = {0};  // Initalize to all 0's
			t.tm_isdst  = 1; 	//daylight savings
			t.tm_year = (sReg[YYMM] & 0x00ff) + 100; // This is year-1900, sReg[YYMM] contaions 'This is year'%100 (e.g. 23)  
			t.tm_mon = (sReg[YYMM] >> 8) - 1;
			t.tm_mday = sReg[DDHH] & 0x00ff;
			t.tm_hour = sReg[DDHH] >> 8;
			t.tm_min = sReg[MMSS] & 0x00ff;
			t.tm_sec = sReg[MMSS] >> 8;
			time = mktime(&t) * 1000 + sReg[MS];	// to float [miliseconds]
			
			if(s_cDataUpdate & ACC_UPDATE)
			{
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			// if(s_cDataUpdate & ANGLE_UPDATE)
			// {
			// 	s_cDataUpdate &= ~ANGLE_UPDATE;
			// }
			if(s_cDataUpdate & MAG_UPDATE)
			{
				s_cDataUpdate &= ~MAG_UPDATE;
			}		

			printf("%ld; %f; %f; %f; %f; %f; %f; %d; %d; %d\r\n", time, fAcc[0], fAcc[1], fAcc[2], fGyro[0], fGyro[1], fGyro[2], sReg[HX], sReg[HY], sReg[HZ]);
		}	
	}
    serial_close(fd);
	return 0;
}


static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    for(int i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}


static void Delayms(uint16_t ucMs)
{ 
	usleep(ucMs*1000);
}
 
	
static void AutoScanSensor(char* dev)
{
	int i, iRetry;
	char cBuff[1];
	
	for(i = 0; i < 10; i++)
	{
		serial_close(fd);
		s_iCurBaud = c_uiBaud[i];
		fd = serial_open((const char*)dev , c_uiBaud[i]);
		
		printf("%d baud search sensor\r\n\r\n", c_uiBaud[i]);
		iRetry = 3;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(300);
			while(serial_read_data(fd, (unsigned char*)cBuff, 1))
			{
				WitSerialDataIn(cBuff[0]);
			}
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}
