# Udev Rules

## Setting Rules of Devices for Dual Arm

1. Copy rules to system
```bash
sudo cp *.rules /etc/udev/rules.d
```

2. Restart udev
```bash
# maybe not work following lines
sudo /etc/init.d/udev restart
# or
sudo udevadm control --reload-rules
```
* Or re-plugin device

## How To Create the Rule :sunglasses:

### For Example:

1. Find the port of your device

```bash
$ ls /dev/tty*

/dev/ttyUSB0
```

2. Find idVendor and idProduct for your device

```bash
$ lsusb

Bus 003 Device 071: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light
```

3. Find serial for your device <br>
    * This step is optional if you need to specify the device.

```bash
$ udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1

    ATTRS{serial}=="IFD65000W650005"
```

4. Create the file "99-example.rules" in "/etc/udev/rules.d" <br>
    * Section of <span style="color:red;">**ATTRS{serial}**</span> is optional if you need to specify the device.

```bash
$ sudo vim /etc/udev/rules.d/99-example.rules

edit or add the following line:
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="IFD65000W650005", SYMLINK+="arc/motion"
```

5. Print the infomation of device

```bash
$ ls -l /dev/arc

lrwxrwxrwx 1 root root 10  五  25 20:33 motion -> ../ttyUSB0
```

