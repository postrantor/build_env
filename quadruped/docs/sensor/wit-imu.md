---
date: 2024-12-29 08:42:02
---

## wit-motion

[HWT906 产品资料](https://wit-motion.yuque.com/wumwnr/docs/cezr1a)

[witmotion_ros - ROS Wiki](https://wiki.ros.org/witmotion_ros)
[WCHSoftGroup/tty_uart: linux tty uart application](https://github.com/WCHSoftGroup/tty_uart)
[CH9344SER_LINUX.ZIP - 南京沁恒微电子股份有限公司](https://www.wch.cn/downloads/CH9344SER_LINUX_ZIP.html)

## sdk

https://wit-motion.yuque.com/wumwnr/ltst03/es0tfkrptxfv9dcq?singleDoc#%20%E3%80%8ASDK%E3%80%8B
https://github.com/WITMOTION/WitStandardProtocol_JY901

## python

```
pip install pyserial
```

## connect serial port

对于 HWT906 型号设备，通过串口调试板+传感器的组合形式，通过 usb 将串口调试板与 raspberrypi-4b 连接。

### 查看连接设备

在连接串口设备后，通过 `lsusb` 查看外接的所有 usb 设备：

```bash
> lsusb
# Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
# Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# Bus 001 Device 009: ID 1a86:7523 QinHeng Electronics CH340 serial converter
# Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
# Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

则外接的 usb 串口设备会挂载到 linux 系统中 `/dev` 路径下的文件。

进一步的可以通过 `ll /dev/ | grep ttyUSB*` 查看挂载的串口 usb 设备。

再者，通过 `udevadm info /dev/ttyUSB<0>` 查看指定的设备详细信息：

```bash
P: /devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.3/1-1.3:1.0/ttyUSB0/tty/ttyUSB0
N: ttyUSB0
S: serial/by-id/usb-FTDI_Quad_RS232-HS-if00-port0
S: serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0
E: DEVLINKS=/dev/serial/by-id/usb-FTDI_Quad_RS232-HS-if00-port0 /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0
E: DEVNAME=/dev/ttyUSB0
E: DEVPATH=/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.3/1-1.3:1.0/ttyUSB0/tty/ttyUSB0
E: ID_BUS=usb
E: ID_MM_CANDIDATE=1
E: ID_MODEL=Quad_RS232-HS
E: ID_MODEL_ENC=Quad\x20RS232-HS
E: ID_MODEL_FROM_DATABASE=FT4232H Quad HS USB-UART/FIFO IC
E: ID_MODEL_ID=6011
E: ID_PATH=platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0
E: ID_PATH_TAG=platform-fd500000_pcie-pci-0000_01_00_0-usb-0_1_3_1_0
E: ID_PCI_CLASS_FROM_DATABASE=Serial bus controller
E: ID_PCI_INTERFACE_FROM_DATABASE=XHCI
E: ID_PCI_SUBCLASS_FROM_DATABASE=USB controller
E: ID_REVISION=0800
E: ID_SERIAL=FTDI_Quad_RS232-HS
E: ID_TYPE=generic
E: ID_USB_DRIVER=ftdi_sio
E: ID_USB_INTERFACES=:ffffff:
E: ID_USB_INTERFACE_NUM=00
E: ID_VENDOR=FTDI
E: ID_VENDOR_ENC=FTDI
E: ID_VENDOR_FROM_DATABASE=Future Technology Devices International, Ltd
E: ID_VENDOR_ID=0403
E: MAJOR=188
E: MINOR=0
E: SUBSYSTEM=tty
E: TAGS=:systemd:
E: USEC_INITIALIZED=13633694
E: net.ifnames=0
```

### 调试连接问题

在连接设备过程中可能遇到问题，如设备未能挂载到 `/dev` 设备下。

#### 尝试更新 ch340 驱动

仓库地址：[p7xxtm1/serial-drive](ssh://p7xxtm1/~/Repositories/Posgraduate/BAA_Project/drive/serial.git)

> [注]：
> [github](https://github.com/WCHSoftGroup/ch9344ser_linux.git)

编译驱动后更新到系统中。

#### 通过 dmesg 查看

```bash
dmesg | grep tty
```

系统加载驱动过程的详细信息：

```log
[  760.872870] usb_ch341 1-1.1:1.0: ttyCH341USB0: ch341 USB device
[  761.557780] usb 1-1.1: usbfs: interface 0 claimed by usb_ch341 while 'brltty' sets config #1
```

从 `dmesg` 的输出中可以看出，CH340 USB 设备已被正确识别为 `ttyCH341USB0`，但无法正常使用的原因是 `brltty` 服务抢占了设备。

`brltty` 是一个盲文显示设备驱动程序，在某些情况下，它会错误地与 USB 串口设备冲突，导致设备无法正常工作。

#### 解决方法

##### 1. 禁用 `brltty` 服务

由于一般用户不需要盲文设备支持，可以安全地禁用 `brltty` 服务：

```bash
sudo systemctl stop brltty
sudo systemctl disable brltty
```

然后重新插拔设备，并检查 `/dev/` 是否出现了相应的设备文件，例如 `/dev/ttyCH341USB0` 或 `/dev/ttyUSB0`。

##### 2. 手动卸载 `brltty` (可选)

如果不需要 `brltty` 服务，可以将其卸载：

```bash
sudo apt remove brltty
```

重新插拔设备后再次检查。

##### 3. 手动指定设备驱动

如果 `brltty` 服务仍然抢占设备，可以通过以下步骤强制重新绑定 CH340 驱动：

1. 查看设备信息：

   ```bash
   lsusb
   ```

   确认 CH340 的设备 ID，例如：

   ```
   Bus 001 Device 004: ID 1a86:7523 QinHeng Electronics CH340 serial converter
   ```

2. 卸载错误绑定的驱动：

   ```bash
   sudo rmmod usbserial
   sudo rmmod ch341
   ```

3. 重新加载 CH340 驱动：

   ```bash
   sudo modprobe ch341
   ```

4. 确保设备未被 `brltty` 占用：

   ```bash
   dmesg | grep tty
   ```

   应该看到类似以下输出：

   ```
   [    7.677624] usb_ch341 1-1.3:1.0: ttyUSB0: ch341 USB device
   ```

##### 4. 确保设备出现在 `/dev/` 路径中

检查 `/dev/` 是否生成了设备文件：

```bash
ls /dev/ttyCH341USB0 /dev/ttyUSB*
```

如果设备出现，则可以直接使用，例如通过以下命令打开串口通信：

```bash
# apt install screen
sudo screen /dev/ttyCH341USB0 9600
```

断开会话：`ctrl + a -> d`
重连会话：`screen -r`

---

通过以上方法，应该可以解决设备冲突问题并正常使用 CH340 串口设备。
