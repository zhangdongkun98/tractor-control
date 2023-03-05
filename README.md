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
