# tractor-control

### installation

```bash
conda activate capac

pip install pyserial
pip install pynmea2
pip install canlib

pip install keyboard==0.13.5
pip install pynput==1.7.6

sudo apt install ros-melodic-can-msgs
```

fangxiangpan  1 and a half

feedback max 4780


note:
C-RS232
note web page I/O setting 'bo te lv'

### simulation

```bash
pip install osqp==0.6.2.post0
```

[自动驾驶轨迹跟踪(一)-模型预测控制(MPC)代码实现](https://cloud.tencent.com/developer/article/1989738)

[自动驾驶轨迹跟踪-模型预测控制(MPC)](https://mp.weixin.qq.com/s?__biz=MzUwOTg3NTQ4NQ==&mid=2247487839&idx=1&sn=cbdec5f9d30b619eed0c60a8d96d97ef&chksm=f90ad52dce7d5c3bd3877e9fd3902318c246fb54aca7537fc7a8909fc9ee1775c93ce2f715c8&scene=21#wechat_redirect)

[code mpc linear](https://github.com/YoungTimes/algorithms/blob/main/mpc/mpc_linear.py)




```bash
python -m carla_utils.config -x /home/ff/github/zdk/tractor-control/envs/drift_map/train.xodr
```



### real

rtk:

```bash
sudo chown zdk:zdk /dev/ttyUSB0
python rtk_driver/rtk_driver.py
```

latitude: 30.258824339333334
longitude: 119.72750057183333
 

D.
front: 1.49
left 0.5
up 2.33


E.
-90


F.
front: 0.09
left: 0.46
up: 1.3


1.17
wheelbase 1.07



```bash
rosrun rviz rviz -d ~/github/zdk/tractor-control/envs/agri.rviz
```



### controller

```

rwfb:
metric:  error path, max: 0.1227, min: 0.0, mean: 0.0043, last: 0.0001, ratio: 94.76 %

rwfb real:
metric:  error path, max: 0.1567, min: 0.0, mean: 0.0066, last: 0.0002, ratio: 92.08 %


pid:
metric:  error path, max: 0.1135, min: 0.0, mean: 0.0055, last: 0.0001, ratio: 91.32000000000001 %
pid real:
metric:  error path, max: 0.1392, min: 0.0, mean: 0.0116, last: 0.0111, ratio: 81.6 %


pid new param, one-order delay 0.95:
metric:  error path, max: 0.1174, min: 0.0, mean: 0.0057, last: 0.0001, ratio: 92.84 %

pid new param, one-order delay 0.95, real:
metric:  error path, max: 0.1659, min: 0.0, mean: 0.0089, last: 0.0002, ratio: 91.47999999999999 %

pid new param, one-order delay 0.95, zero-order delay 10:
metric:  error path, max: 0.3154, min: 0.0, mean: 0.0144, last: 0.0001, ratio: 89.44 %

metric:  error path, max: 0.5259, min: 0.0, mean: 0.0506, last: 0.0476, ratio: 57.04 %
```


