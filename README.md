Novatel GPS Driver
==================
  
This project provides a cross-platform interface for the Novatel OEM4 and OEMV series of GPS receivers.  The Novatel SPAN system is also supported. 

#update log
2018-03-28 15:11:58 [author]scott yu,[log]init proj for huituo
2018-03-31 11:59:06 [author]scott yu,[log]�����˵���ģʽ����������ģʽ�����ִ���
2018-04-08 17:50:12 [author]scott yu,[log]�����豸�����Զ������ܣ��豸����launch��ߵ���������������ã����Խ��й켣�ɼ�������Ҫ����.gps�ļ��������

#usage for user
## novatel_node
source ~/novatel_ws/devel/setup.bash
roslaunch novatel novatel.launch

## read .gps file to output track file
cd ~/novatel_ws/devel/lib/novatel/
cpy xxx.gps novatel_ws/devel/lib/novatel/
./novatel_read_from_file xxx.gps  //���Դ�tower��ȡ���ǵĲ���ʱ��ɼ���.gps���� �������� ����ϵ����


#debug log
2018-03-31 15:06:46 src/�µ� A���Ҳ���������b��
	һ�������ռ��е�A����Ҫʹ��B�е����ݣ���Ҫ��A���е�package.xml�б��������Ҫ��B�������������
	ͬʱ��Ҫ��B��������д��A����cmakelists.txt�����������ط�������A����novatel��B����gps-msgs
		find_package(catkin COMPONENTS serial roslib roscpp rosconsole tf gps_msgs sensor_msgs nav_msgs)
		catkin_package(
				INCLUDE_DIRS include
				LIBRARIES novatel
				CATKIN_DEPENDS serial roslib roscpp rosconsole tf gps_msgs nav_msgs sensor_msgs
				DEPENDS Boost
			)
	�����ϵͳ�Դ���package����ֱ�ӿ���findpackage�ҵ��ģ�����ϵͳ���������еģ���Ҫ��ô����


## conf
### config the pc
get the Authentication of comm to usb
sudo vim  /etc/udev/rules.d/70-tty.rules
KERNEL=="ttyUSB[0-9]*",MODE=="0666"
KERNEL=="ttyS[0-9]*",MODE=="0666"


##info about novatel protocol

## minicomʹ��
sudo minicom
ctrl-a o ����
ctrl-a z e ���·��������
ctrl-a q �˳�

������治ͣ���ϴ�����·�����
�м���������Ȼ��Ѹ�ٻس�����


### �������
	�����ִ���Ϣ
	�Ǹ�������� �е��� log psrposa ontime 1 �е��� log psrposb ontime 1
		psrpos+a ����+b ����һ��ascallһ��16����
			����ʾ����ʾ��
			log����a��B�ĸ�ʽ�ֱ����£�
				a:�������#BEST****��ͷ���ַ�������
				b:��0xaa 0x44 0x12��ͷ�Ķ��������ݣ���������������.gps��ʽ�Ķ�����������һ�µ�
### config 
#### cmd eg:
log version  //�鿴�汾
com com1 115200  //����com1 ������
log bestposa ontime 1  //����bestposa���ݵĸ���Ƶ��Ϊ1hz
log bestposa ontime 0.1   //���ø���Ƶ��Ϊ10hz
log comconfig  //��ӡ����com��������Ϣ
freset  //�ָ���������
log loglist  //�鿴�������
#### config the mobile station for xcmg
##### cmd list
freset
com com2 115200 n 8 1 n off n
interfacemode com2 rtca none off
com com1 115200
log com1 inspvasb ontime 0.01
log com1 imratepvab onnew
log insposb ontime 1
saveconfig


com com2 115200 n 8 1 n off n
interfacemode com2 rtca none off
log inspvaa ontime 1
log inspvab ontime 1
log version
unlogall



#####�������ϸ����
freset
	�ָ�����������
com com2 115200 n 8 1 n off n
	com2����rtk��̨���ݵ��ƶ�վ�����ݴ��ڣ���Ҫ��������ڵĲ���
interfacemode com2 rtca none off
	�����﷨��ʽ��INTERFACEMODE [port] rxtype txtype [responses]
		com2�ڽ��յĲ������ģʽΪrtca������ģʽΪnone�����û��Թر�
com com1 115200
log com1 inspvasb ontime 0.01
	����inspvas����100hz���
log com1 imuratepvab onnew
	����imu�����������Ҫ������ASYNCHINSLOGGING ENABLE
log insposb ontime 1
	����inspos������1hz���
saveconfig
	��������


### ������methodology
	Ҫ֪��ʲômsg��id�����ݣ�����ֱ���ڡ�OEM7_Commands_Logs_Manual.pdf���ĵ��в�ѯ��Ӧ��id������
	Ȼ����߻���log��ָ�
	�������ҵ�Best available UTM data��msg id������
		Message ID: 726
		Log Type: Synch
		Recommended Input:
		log bestutma ontime 1
		ASCII Example:
		#BESTUTMA,COM1,0,73.0,FINESTEERING,1419,336209.000,02000040,eb16,2724;
		SOL_COMPUTED,NARROW_INT,11,U,5666936.4417,707279.3875,1063.8401,-
		16.2712,WGS84,0.0135,0.0084,0.0173,"AAAA",1.000,0.000,8,8,8,8,0,01,0,0
		3*a6d06321
		
	������������ʾ����:log bestutma ontime 1���ݣ�������������novatel�ߵ�ϵͳ���bestutm����Ϣ
	1�������ݸ������ڣ���λΪsec


# Installation 

## ROS Install

The serial and Novatel packages are both "wet" packages and require Catkin.  To build the libraries, first create a Catkin workspace (you can skip this step if you are adding the packages to an existing workspace.)

	mkdir -p ~/novatel_ws/src
	cd ~/novatel_ws/src
	catkin_init_workspace
	wstool init ./
	
Next, add the Serial, GPS Messages, and Novatel packages to the workspace:

	wstool set serial --git git@github.com:wjwwood/serial.git
	wstool set novatel --git git@github.com:GAVLab/novatel.git
	wstool set gps_msgs --git git@github.com:GAVLab/gps_msgs.git
	wstool update

Finally, build the packages:

	cd ../
	catkin_make


## Standalone Install

Although Catkin is the preferred build method, both packages can be installed without Catkin.  An older version of the serial library can be installed using the instructions below:

	git clone git://github.com/wjwwood/serial.git
	cd serial
	git checkout fuerte
	make
	sudo make install
	
Next, the Novatel library can be installed by:

	git clone git@github.com:GAVLab/novatel.git
	cd novatel
	make
	sudo make install
	
The above commands will build and install the Novatel static library.  To also build an example program for testing the interface:

	cd build
	cmake ../ -DNOVATEL_BUILD_EXAMPLES=ON
	make
	
To build the optional Novatel tests:

	cd build
	cmake ../ -DNOVATEL_BUILD_TESTS=ON
	make


# Operation




## Callback Definitions


## Supported Messages

# License

The BSD License

Copyright (c) 2018

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Authors

Scott Yu <hongsong.yu2010@gmail.com>
