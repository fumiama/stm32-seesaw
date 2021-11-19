<div align="center">
  <img src="imgs/head.png" width = "300" height = "300" alt="KanbanMusume"><br>
  <h1>stm32-seesaw</h1>
  基于HAL库的stm32机器人跷跷板项目<br><br>
</div>

## 开发流程
首先使用`STM32 Cude MX`生成带`Makefile`的HAL库工程模版，然后修改`Core`文件夹下的用户代码，使用`make`命令编译，在`build`目录即可获得`seesaw.elf`等文件，接下来按下`BOOT0+RESET`使单片机进入下载模式，使用`STM32 Cude Programmer`将编译好的程序烧录进单片机。

## 硬件构成
小车为三轮，车身由铝合金框架连接构成，在其上固定有面包板，用杜邦线连接单片机、电机、蓝牙模块、电池盒与传感器，同时用扎线带等固定好分线器（位于侧面与底部）与电源模块（位于底部），具体实物图如下。

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
单片机型号为`STM32F401CCUx`，具体如上图。定义其引脚与中断如下，详细可见`seesaw.ioc`，该文件可用`STM32 Cude MX`打开。

![pins](imgs/pins.jpg)

![pins](imgs/ints.jpg)
