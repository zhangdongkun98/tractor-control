# tractor-control

### installation

```bash
conda activate capac

pip install pyserial
pip install pynmea2
pip install canlib

pip install keyboard==0.13.5
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
