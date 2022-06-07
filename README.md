**任务二描述：47分**

**使用CC2530单片机实现可燃气体检测的系统。按接线图进行设备安装与部署，使用所给的代码利用IAR编程实现ZigBee黑板采集可燃气体电压数据，并发送给串口，再通过串口连接到物联网网关的A1B1接口，最后由物联网网关发送传感数据至云平台，云平台上显示上报的传感器数据（与任务一在同一个云平台项目中显示），通过串口调试助手也能查看到可燃气体传感电压数据。**

**设备列表：**

1、 ZigBee黑板1个

2、 可燃气体传感器1个

3、CC Debugger仿真器1个

4、RS485转232转接头一个、公对公串口线一个。

5、导线若干

6、工具包一套

**任务要求：**

在工程源码目录“..\\work\\Task\\”文件夹下创建IAR工程，工程名字和工作空间的名字均为：“Task+准考证号后12位”，把“Task” 文件夹下的“test.c”文件**添加到IAR工程中**，配置好工程选项参数，确保工程编译成功。（4分）。在“test.c”中添加代码实现以下功能：

1、ZigBee模块一上电，所有LED灯不亮。每发送一次传感数据时LED (P1_0)灯闪烁1次。（10分）

2、采集可燃气体传感器电压数据的ADC相关参数要求如下：3.3V电压，128位抽取率，单通道0。

3、串口通信要求采用波特率115200，8位数据位，1位停止位，无校验位，无流控。

4、定时时间要求使用定时器1，32MHz时钟频率，工作模式为模模式，128分频，通道0，定时器的时间周期为200ms。

5、先将ZigBee模块通过串口线接在PC机上，在工程中增加条件编译变量“debug”，当有定义过“debug”时，每2秒采集一次可燃气体传感器数据并发送到串口。将”串口助手”ASCII码数据界面截图保存至“图集.docx”中的9.png处。（6分）

6、接着将ZigBee模块连接到物联网网关的A1B1接口，修改条件编译变量“debug”，重新编译、烧写，每隔2秒把可燃气体数据按照物联网网关协议帧发送到串口。将 “串口助手数据”界面截图保存至“图集.docx”中的10.png处。（12分）

7、以上操作成功并完成后，云平台上相应界面将显示物联网网关在线，并显示出光照传感器、人体红外传感器数据、可燃气体传感器实时数据。将云平台上显示的传感器实时数据的界面截图保存至“图集.docx”中的11.png处。（15分）
