<div align="center">
  <img src="imgs/head.png" width = "300" height = "300" alt="KanbanMusume"><br>
  <h1>stm32-seesaw</h1>
  基于HAL库的stm32机器人跷跷板项目<br><br>
</div>

## 开发流程
首先使用`STM32 Cude MX`生成带`Makefile`的HAL库工程模版，然后修改`Core`文件夹下的用户代码，使用`make`命令编译，在`build`目录即可获得`seesaw.elf`等文件，接下来按下`BOOT0+RESET`使单片机进入下载模式，使用usb将其连接到电脑，用`STM32 Cude Programmer`将编译好的程序烧录进单片机。

## 硬件构成
小车为三轮，车身由铝合金框架连接构成，在其上固定有面包板，用杜邦线连接单片机、电机、蓝牙模块、电池盒与传感器，同时用扎线带等固定好分线器（位于侧面与底部）与电源模块（位于底部），具体实物图与分模块说明如下。

<table>
	<tr>
		<td align="center"><img src="imgs/car_above.jpg"></td>
		<td align="center"><img src="imgs/car_below.jpg"></td>
        <td align="center"><img src="imgs/car_core.jpg"></td>
	</tr>
    <tr>
		<td align="center">上面</td>
		<td align="center">底面</td>
        <td align="center">核心</td>
	</tr>
</table>
<table>
	<tr>
		<td align="center"><img src="imgs/car_side.jpg"></td>
	</tr>
    <tr>
		<td align="center">侧面</td>
	</tr>
</table>

### 1. 单片机
单片机型号为`STM32F401CCUx`，具体实物如上图。定义其引脚与中断如下，详细可见`seesaw.ioc`，该文件可用`STM32 Cude MX`打开。

![pins](imgs/pins.jpg)

![ints](imgs/ints.jpg)

### 2. 电机
电机型号为`MG995`，使用单片机`TIM1`产生的`PWM`波驱动，实物图如下。

![motor](imgs/motor.jpg)

### 3. 传感器
传感器为九轴传感器`GY953`，通过`UART1`与单片机通信，具有加速度、角加速度、欧拉角，磁场的测量功能，并且还可以以固定频率（默认`50Hz`）输出数据，具体使用文档可以在网络上找到，不再赘述。项目中只使用了角加速度与欧拉角，并使用其固定频率作为外部时钟中断。

### 4. 蓝牙模块
蓝牙模块作为调试使用，通过`UART2`与单片机通信，在调试完毕后可以移除。

### 5. 电源模块
电源模块位于小车底部，提供恒定的`5V`电压源。不过当电机转速突变时，其将产生约`9V`的尖峰，可能会导致系统工作失常。本项目的代码通过避免速度突变使系统工作正常，不过也可通过加入电容调节。

