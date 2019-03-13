# wrs2018/disposing
## Realsense 
follow by website:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md


```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo rm -f /etc/apt/sources.list.d/realsense-public.list.
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
sudo apt-get install ros-kinetic-rgbd-launch libusb-1.0-0-dev libglfw3-dev libgtk-3-dev -y
modinfo uvcvideo | grep "version:"  
#this command is check the kernel is updated! it should show something "realsense" string
```

## YoloV3 package setup
Cuda install
Download cuda from this website
https://developer.nvidia.com/cuda-90-download-archive
```bash
cd ~Download
sudo dpkg -i cuda-repo-ubuntu1604-9-0-local_9.0.176-1_amd64.deb
sudo apt-key add /var/cuda-repo-<version>/7fa2af80.pub
sudo apt-get update
sudo apt-get install cuda
```

Download cudnn from this website
https://developer.nvidia.com/rdp/cudnn-download
seletced "Download cuDNN v7.3.1 (Sept 28, 2018), for CUDA 9.0"
->"cuDNN v7.3.1 Library for Linux"
```bash
cd ~Download
tar zxvf cudnn-9.0-linux-x64-v7.3.1.20.tgz
cd cuda
sudo cp include/cudnn.h /usr/local/cuda/include
sudp cp lib64/libcudnn* /usr/local/cuda/lib64
sudo chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
``` 

Environment setup
  Install python2,3 type tensorflow , keras and opencv
```bash
sudo apt install python-dev python-pip python3-dev python3-pip -y
pip install matplotlib 
pip install tensorflow-gpu
pip install keras
pip3 install opencv-python
pip3 install tensorflow-gpu
pip3 install keras
pip3 install rospkg  
#let ros can run python3 code
```
try this command test tensorflow install sucessful
```bash
python -c "import tensorflow as tf; print(tf.__version__)"
python3 -c "import tensorflow as tf; print(tf.__version__)"
```
`