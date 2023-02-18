# CheryCtrl

## Install
**Install python package**

```bash
pip install canlib
```

**Install  libcanlib.so**

Download link: [Kvaser LINUX Driver and SDK Download](https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest) or [Intranet Download](http://10.12.120.70:5000/d/f/724877724816028007)
```bash
pip install canlib
tar zxvf linuxcan.tar.gz
cd linuxcan
make -j16
sudo make install
```

**Test  canlib**
```bash
cd chery_can_driver/
python test_can.py
```

If not working, go to ```linuxcan```, unplug the device, and reinstall again
```bash
make -j16
sudo make install
```
or check ```Secure Boot in BIOS is disable```


## Run
```bash
cd scripts
python ros_node.py
```

## ROS

| Interface Type | Topic Name     | Message Type                                                 |
| -------------- | -------------- | ------------------------------------------------------------ |
| Publisher      | /vehicle_state | Float64MultiArray, [speed, speed_ts, steer, steer_ts, gas, gas_ts] |
| Subscriber     | /vechile_ctrl  | Float64MultiArray, [flag, gas, steer, steer_speed, brake]    |

Note: speed (km/h), steer >= 0

**Control Flag:**

1: Auto mode

2: D gear

3: Warm start

4: Roll steer

5: Ctrl (normal control mode)

Other: Emergency stop
