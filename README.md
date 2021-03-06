<!-- TOC -->

- [usage for user](#usage-for-user)
  - [run novatel_node](#run-novatel_node)
- [log](#log)
  - [update log](#update-log)
  - [debug log](#debug-log)
  - [采集轨迹数据](#采集轨迹数据)
  - [read .gps file to output track file](#read-gps-file-to-output-track-file)
- [remote debug on ipc](#remote-debug-on-ipc)
  - [ssh connect](#ssh-connect)
  - [linux下串口调试助手简介](#linux下串口调试助手简介)
    - [minicom使用](#minicom使用)
    - [其他备选调试助手](#其他备选调试助手)
- [novatel](#novatel)
  - [问题解决](#问题解决)
  - [conf](#conf)
    - [config the ipc](#config-the-ipc)
    - [设置惯导基本参数](#设置惯导基本参数)
  - [info about novatel protocol](#info-about-novatel-protocol)
    - [总体规则：](#总体规则)
    - [config](#config)
      - [cmd eg:](#cmd-eg)
      - [config the mobile station for xcmg](#config-the-mobile-station-for-xcmg)
        - [cmd list](#cmd-list)
        - [命令集合详细解释](#命令集合详细解释)
    - [方法论methodology](#方法论methodology)
- [Installation](#installation)
  - [ROS Install](#ros-install)
  - [Standalone Install](#standalone-install)
- [Operation](#operation)
  - [Callback Definitions](#callback-definitions)
  - [Supported Messages](#supported-messages)
- [License](#license)
- [Authors](#authors)
- [novatel](#novatel-1)

<!-- /TOC -->
This project provides a cross-platform interface for the Novatel OEM4 and OEMV series of GPS receivers.  The Novatel SPAN system is also supported. 


# usage for user
## run novatel_node
cd ~/yuhs_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="novatel"
source ~/novatel_ws/devel/setup.bash
roslaunch novatel novatel.launch

cd ~/catkin_ws
catkin_make
roslaunch novatel novatel.launch


# log
## update log
2018-03-28 15:11:58 [author]scott yu,[log]init proj for huituo
2018-03-31 11:59:06 [author]scott yu,[log]增加了调试模式和正常工作模式的区分代码
2018-04-08 17:50:12 [author]scott yu,[log]增加设备设置自动化功能，设备利用launch里边的配置命令进行配置，可以进行轨迹采集，不需要经过.gps文件即可完成




## debug log
2018-03-31 15:06:46 src/下的 A包找不到依赖的b包
一个工作空间中的A包需要使用B中的数据，需要在A包中的package.xml中编译和运行要对B包进行相关依赖
	同时需要将B包的名字写到A包的cmakelists.txt中下面两个地方，比如A包是novatel，B包是gps-msgs
		find_package(catkin COMPONENTS serial roslib roscpp rosconsole tf gps_msgs sensor_msgs nav_msgs)
		catkin_package(
				INCLUDE_DIRS include
				LIBRARIES novatel
				CATKIN_DEPENDS serial roslib roscpp rosconsole tf gps_msgs nav_msgs sensor_msgs
				DEPENDS Boost
			)
	如果是系统自带的package，是直接可以findpackage找到的，不是系统环境变量中的，需要这么操作

## 采集轨迹数据
设置 launch/novatel.launch里边的
<param name="track_file_output_path_only_xy" value="" />
<param name="file_xyh_path" value="" />
的路径信息，即可完成采集相应的轨迹数据
1，track_file_output_path_only_xy，采集的轨迹数据是 序号 x y
2，track_file_output_path_only_xy，采集的轨迹数据是 序号 x y heading


## read .gps file to output track file

将cmakelist中的这句话add_executable(novatel_read_from_file examples/novatel_read_from_file.cpp)的注释放开

cd ~/novatel_ws/devel/lib/novatel/
//可以从tower获取我们的测试时候采集的.gps数据 如有疑问 请联系作者
cpy xxx.gps novatel_ws/devel/lib/novatel/ ./novatel_read_from_file xxx.gps  

# remote debug on ipc
## ssh connect
问题：ssh connection refused
解决：在工控机上运行
sudo apt-get -purge  remove  openssh-server
sudo apt-get install openssh-server
然后重启 ssh服务
sudo /etc/init.d/ssh restart

## linux下串口调试助手简介
### minicom使用
sudo minicom
ctrl-a o 设置
ctrl-a z e 打开下发命令回显,下发的命令即可显示出来，然后回车即可
ctrl-a q 退出

如果下面不停的上传命令，下发方法
中键拷贝命令然后迅速回车即可

### 其他备选调试助手
cli串口助手
	minicom
gui com助手
	gtkterm
	cutecom
	moni

# novatel
## 问题解决
- inspva数据上来经纬度为0，但是bestpos数据有

用window下的novatel软件进行配置组合惯导信息
1、获取组合惯导配置手册《novatel组合惯导配置手册.docx》如果找不到 请联系 hongsong.yu
2、获取imu在车上的姿态，imu是fsas型号，手册中有imu的方位信息
3、imu的朝向和车体方向的关系请参照手册中的信息
4、打开win下的软件，工具栏有个wizard，打开里边的span aliament即可进行设置
5、设置完成后，会弹出一个界面，显示组合惯导数据，这个里边没有数据
6、要查看数据，还是通过inspvaa的命令在串口终端查看，会发现数据上来了
- 串口助手没有数据上来
排查方法
1、
## conf
### config the ipc
get the Authentication of comm to usb
sudo vim  /etc/udev/rules.d/70-tty.rules
KERNEL=="ttyUSB[0-9]*",MODE=="0666"
KERNEL=="ttyS[0-9]*",MODE=="0666"

### 设置惯导基本参数
gps主天线(ant1，车行驶方向前面的那个)相对于imu的偏移量

x:0.95
y:0.70
z：1.62

## info about novatel protocol




### 总体规则：
- 不区分大消息
- 那个命令里边 有的是 log psrposa ontime 1 有的是 log psrposb ontime 1
-	psrpos+a 或者+b 代表：一个ascall一个16进制
- 如下示例所示：
	- log命令a和B的格式分别如下：
	- a:输出的是#BEST****打头的字符串数据
	- b:以0xaa 0x44 0x12打头的二进制数据，跟我们软件保存的.gps格式的二进制数据是一致的
### config 
#### cmd eg:
log version  //查看版本
com com1 115200  //设置com1 波特率
log bestposa ontime 1  //设置bestposa数据的更新频率为1hz
log bestposa ontime 0.1   //设置更新频率为10hz
log comconfig  //打印所有com口配置信息
freset  //恢复出厂设置
log loglist  //查看输出配置
#### config the mobile station for xcmg
##### cmd list
freset
com com2 115200 n 8 1 n off on
interfacemode com2 rtca none off
serialconfig com1 115200
log com1 inspvab ontime 0.1
log com1 imuratepvab onnew
log insposb ontime 1
saveconfig



com com2 115200 n 8 1 n off on
interfacemode com2 rtca none off
log inspvaa ontime 1
log inspvab ontime 1
log version
unlogall
log inspvaxa ontime 1
log bestposa ontime 1



#####命令集合详细解释
freset
	恢复到出厂设置
com com2 115200 n 8 1 n off n
	com2口是rtk电台数据到移动站的数据串口，需要配置这个口的参数
interfacemode com2 rtca none off
	命令语法格式：INTERFACEMODE [port] rxtype txtype [responses]
		com2口接收的差分数据模式为rtca，发送模式为none，设置回显关闭
com com1 115200
log com1 inspvasb ontime 0.01
	设置inspvas数据100hz输出
log com1 imuratepvab onnew
	设置imu数据输出，需要先设置ASYNCHINSLOGGING ENABLE
log insposb ontime 1
	设置inspos的数据1hz输出
saveconfig
	保存设置




### 方法论methodology
	要知道什么msg的id的内容，可以直接在《OEM7_Commands_Logs_Manual.pdf》文档中查询对应的id的描述
	然后里边会有log的指令，
	比如我找到Best available UTM data的msg id的内容
		Message ID: 726
		Log Type: Synch
		Recommended Input:
		log bestutma ontime 1
		ASCII Example:
		#BESTUTMA,COM1,0,73.0,FINESTEERING,1419,336209.000,02000040,eb16,2724;
		SOL_COMPUTED,NARROW_INT,11,U,5666936.4417,707279.3875,1063.8401,-
		16.2712,WGS84,0.0135,0.0084,0.0173,"AAAA",1.000,0.000,8,8,8,8,0,01,0,0
		3*a6d06321
		
	如上面内容所示，有:log bestutma ontime 1内容，这个命令就是让novatel惯导系统输出bestutm的信息
	1代表数据更新周期，单位为sec

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
=======
# novatel
driver for novatel oem6
