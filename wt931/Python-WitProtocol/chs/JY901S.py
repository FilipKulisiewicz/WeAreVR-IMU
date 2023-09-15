# coding:UTF-8
"""
    测试文件
    Test file
"""
import time
import copy
import datetime
import platform
import struct
import sys
import math
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

welcome = """ Welcome to the Wit-Motoin sample program """
_writeF = None                    #写文件  Write file
_IsWriteF = False                 #写文件标识    Write file identification

def readConfig(device):
    """
    读取配置信息示例    Example of reading configuration information
    :param device: 设备模型 Device model
    :return:
    """
    time.sleep(0.1)
    tVals = device.readReg(0x02,3)  #读取数据内容、回传速率、通讯速率   Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("return result：" + str(tVals))
    else:
        print("no result")
    tVals = device.readReg(0x23,2)  #读取安装方向、算法  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("return result：" + str(tVals))
    else:
        print("no result")

def setConfig(device):
    """
    设置配置信息示例    Example setting configuration information
    :param device: 设备模型 Device model
    :return:
    """
    device.unlock()
    time.sleep(0.1) 
    device.writeReg(0x03, 0x0c)      # Set the transmission back rate to 10HZ, 500Hz (0x0c)
    time.sleep(0.1)
    tVals = device.readReg(0x02,1)[0]
    time.sleep(0.1)
    print(tVals)
    device.writeReg(0x02, tVals | 0x201) # 0b01000000001 # turn on the time and quaternions
    time.sleep(0.1)
    print(device.readReg(0x02,1)[0])
    time.sleep(0.1)
    device.writeReg(0x23, 0)       # Set the installation direction: horizontal and vertical
    time.sleep(0.1)
    device.writeReg(0x24, 0x01)       # Set the installation direction: nine axis 0x00, six axis 0x01
    time.sleep(0.1)  
    # ----------
    
    print("Calibration value threshold: " + str(device.readReg(0x61,1)[0]))
    print("Calibration time threshold: " + str(device.readReg(0x63,1)[0]))
    print("Werror: " + str(device.readReg(0x6a,1)[0]))

    device.writeReg(0x61, 0x00ff)
    time.sleep(0.1)

    device.writeReg(0x63, 0x0001)
    time.sleep(0.1)

    print("Calibration value threshold: " + str(device.readReg(0x61,1)[0]))
    print("Calibration time threshold: " + str(device.readReg(0x63,1)[0]))

    # -----------
    time.sleep(0.1)  
    device.save()



def AccelerationCalibration(device):
    """
    加计校准    Acceleration calibration
    :param device: 设备模型 Device model
    :return:
    """
    device.AccelerationCalibration()                 # Acceleration calibration
    print("加计校准结束")

def FiledCalibration(device):
    """
    磁场校准    Magnetic field calibration
    :param device: 设备模型 Device model
    :return:
    """
    device.BeginFiledCalibration()                   # 开始磁场校准   Starting field calibration
    if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower()=="y":
        device.EndFiledCalibration()                 # 结束磁场校准   End field calibration
        print("结束磁场校准")


def onUpdate(deviceModel):
    """
    数据更新事件  Data update event
    :param deviceModel: 设备模型    Device model
    :return:
    """

    dt_object = datetime.datetime.strptime(deviceModel.getDeviceData("Chiptime"), "%Y-%m-%d %H:%M:%S.%f")
    '''print(str("{:.3f}".format(dt_object.timestamp()))  + "; "
         # , str(deviceModel.getDeviceData("temperature")) + ";"
         , str("{:.4f}".format(deviceModel.getDeviceData("accX"))) + "; " + str("{:.4f}".format(deviceModel.getDeviceData("accY"))) +"; "+ str("{:.4f}".format(deviceModel.getDeviceData("accZ"))) + "; "
         , str("{:.4f}".format(math.radians(deviceModel.getDeviceData("gyroX")))) + "; " + str("{:.4f}".format(math.radians(deviceModel.getDeviceData("gyroY")))) +"; "+ str("{:.4f}".format(math.radians(deviceModel.getDeviceData("gyroZ")))) + "; "
         # , str(deviceModel.getDeviceData("angleX")) + "," + str(deviceModel.getDeviceData("angleY")) +","+ str(deviceModel.getDeviceData("angleZ")) + ";"
         , str(deviceModel.getDeviceData("magX")) + "; " + str(deviceModel.getDeviceData("magY"))+ "; "+ str(deviceModel.getDeviceData("magZ")) + "; "
         # , str(deviceModel.getDeviceData("lon")) + "," + str(deviceModel.getDeviceData("lat")) + ";"
         # , str(deviceModel.getDeviceData("Yaw")) + ","  + str(deviceModel.getDeviceData("Speed")) + ";"
         , str(deviceModel.getDeviceData("q1")) + "; " + str(deviceModel.getDeviceData("q2")) + "; " + str(deviceModel.getDeviceData("q3"))+ "; " + str(deviceModel.getDeviceData("q4"))
          ) '''
    # print("time:" + str(time.time())
    #      , " temp:" + str(deviceModel.getDeviceData("temperature"))
    #      , " acc:" + str(deviceModel.getDeviceData("accX")) +","+  str(deviceModel.getDeviceData("accY")) +","+ str(deviceModel.getDeviceData("accZ"))
    #      , " gyro:" + str(deviceModel.getDeviceData("gyroX")) +","+ str(deviceModel.getDeviceData("gyroY")) +","+ str(deviceModel.getDeviceData("gyroZ"))
    #      , " angle:" + str(deviceModel.getDeviceData("angleX")) +","+ str(deviceModel.getDeviceData("angleY")) +","+ str(deviceModel.getDeviceData("angleZ"))
    #      , " mag:" + str(deviceModel.getDeviceData("magX")) +","+ str(deviceModel.getDeviceData("magY"))+","+ str(deviceModel.getDeviceData("magZ"))
    #      , " lon:" + str(deviceModel.getDeviceData("lon")) + " lat:" + str(deviceModel.getDeviceData("lat"))
    #      , " Yaw:" + str(deviceModel.getDeviceData("Yaw")) + " speed:" + str(deviceModel.getDeviceData("Speed"))
    #      , " quat:" + str(deviceModel.getDeviceData("q1")) + "," + str(deviceModel.getDeviceData("q2")) + "," + str(deviceModel.getDeviceData("q3"))+ "," + str(deviceModel.getDeviceData("q4"))
    #       )
    if (_IsWriteF):    #记录数据    Record data
        Tempstr = " " + str(deviceModel.getDeviceData("Chiptime"))
        Tempstr += "\t"+str(deviceModel.getDeviceData("accX")) + "\t"+str(deviceModel.getDeviceData("accY"))+"\t"+ str(deviceModel.getDeviceData("accZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("gyroX")) +"\t"+ str(deviceModel.getDeviceData("gyroY")) +"\t"+ str(deviceModel.getDeviceData("gyroZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("angleX")) +"\t" + str(deviceModel.getDeviceData("angleY")) +"\t"+ str(deviceModel.getDeviceData("angleZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("temperature"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("magX")) +"\t" + str(deviceModel.getDeviceData("magY")) +"\t"+ str(deviceModel.getDeviceData("magZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("lon")) + "\t" + str(deviceModel.getDeviceData("lat"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("Yaw")) + "\t" + str(deviceModel.getDeviceData("Speed"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q1")) + "\t" + str(deviceModel.getDeviceData("q2"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q3")) + "\t" + str(deviceModel.getDeviceData("q4"))
        Tempstr += "\r\n"
        _writeF.write(Tempstr)

def startRecord():
    """
    开始记录数据  Start recording data
    :return:
    """
    global _writeF
    global _IsWriteF
    _writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")    #新建一个文件
    _IsWriteF = True                                                                        #标记写入标识
    Tempstr = "Chiptime"
    Tempstr +=  "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(°)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\tlon\tlat"
    Tempstr += "\tYaw\tSpeed"
    Tempstr += "\tq1\tq2\tq3\tq4"
    Tempstr += "\r\n"
    _writeF.write(Tempstr)
    #print("开始记录数据")

def endRecord():
    """
    结束记录数据  End record data
    :return:
    """
    global _writeF
    global _IsWriteF
    _IsWriteF = False             # 标记不可写入标识    Tag cannot write the identity
    _writeF.close()               #关闭文件 Close file
    #print("结束记录数据")

if __name__ == '__main__':

    print(welcome)
    """
    初始化一个设备模型   Initialize a device model
    """
    device = deviceModel.DeviceModel(
        "我的JY901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )
    usb_path = sys.argv[1]
    if (platform.system().lower() == 'linux'):
        device.serialConfig.portName = usb_path   #设置串口   Set serial port "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "COM39"          #设置串口   Set serial port
    device.serialConfig.baud = 921600                   #设置波特率  Set baud rate
    device.openDevice() 
    setConfig(device)
    readConfig(device)                                  #读取配置信息 Read configuration information
    print("Chiptime	ax(g)	ay(g)	az(g)	wx(rad/s)	wy(rad/s)	wz(rad/s)	magx	magy	magz	q1	q2	q3	q4")
    device.dataProcessor.onVarChanged.append(onUpdate)  #数据更新事件 Data update event

    #startRecord()                                       # 开始记录数据    Start recording data
    input()
    device.closeDevice()
    #endRecord()                                         #结束记录数据 End record data
