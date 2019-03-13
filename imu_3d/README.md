# IMU 
1. 執行時會出現問題
> 執行imu_3d時會因為執行權限不夠無法抓到device

2. 必須將ros的環境變數加入sudo
```bash
sudo visudo
```

3. 將以下程式碼加到最下面

```bash
Defaults env_keep += "ROS_MASTER_URI"
```

4. 複製 .rule

```bash
sudo cp src/imu_3d/rule/01-imu.rules /etc/udev/rules.d/ 
```

5. 重起udev
```bash
sudo service udev reload
sudo service udev restart
```

5. 執行程式

```bash
rosrun imu_3d imu_3d
```

