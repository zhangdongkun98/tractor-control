#!/usr/bin/env python3
import depthai as dai
import cv2
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import rospy
from sensor_msgs.msg import Image

def timeDeltaToS(delta) -> float:
        return delta.total_seconds()

def clamp(num, v0, v1):
    return max(v0, min(num, v1))

class CvBridge():
    def __init__(self):
        self.numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                                        'int16': '16S', 'int32': '32S', 'float32': '32F',
                                        'float64': '64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)

    def cv2_to_imgmsg(self, cvim, encoding = "passthrough"):
        img_msg = Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
        if encoding == "passthrough":
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding

        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cvim.tobytes()
        img_msg.step = len(img_msg.data) // img_msg.height
        return img_msg

class CameraImuArray:
    def __init__(self):
        self.FPS = 10
        self.RESOLUTION = dai.ColorCameraProperties.SensorResolution.THE_800_P
        self.cam_list = ['cam_a', 'cam_b', 'cam_c', 'cam_d']
        self.cam_socket_opts = {
            'cam_a': dai.CameraBoardSocket.CAM_A,
            'cam_b': dai.CameraBoardSocket.CAM_B,
            'cam_c': dai.CameraBoardSocket.CAM_C,
            'cam_d': dai.CameraBoardSocket.CAM_D,
        }

        self.pipeline = dai.Pipeline()
        self.cam = {}
        self.xout = {}

        # color
        self.controlIn = self.pipeline.create(dai.node.XLinkIn)
        self.controlIn.setStreamName('control')
        for camera_name in self.cam_list:
            self.cam[camera_name] = self.pipeline.createColorCamera()
            self.cam[camera_name].setResolution(self.RESOLUTION)
            if camera_name == 'cam_d':  # ref trigger
                self.cam[camera_name].initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.OUTPUT)
            else:  # other trigger
                self.cam[camera_name].initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
            self.cam[camera_name].setBoardSocket(self.cam_socket_opts[camera_name])
            self.xout[camera_name] = self.pipeline.createXLinkOut()
            self.xout[camera_name].setStreamName(camera_name)
            self.cam[camera_name].isp.link(self.xout[camera_name].input)
            self.cam[camera_name].setFps(self.FPS)
            self.controlIn.out.link(self.cam[camera_name].inputControl)

        self.config = dai.Device.Config()
        self.config.board.gpio[6] = dai.BoardConfig.GPIO(dai.BoardConfig.GPIO.OUTPUT, dai.BoardConfig.GPIO.Level.HIGH)
        self.device = dai.Device(self.config)

        # Define sources and outputs
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)
        self.xlinkOut.setStreamName("imu")
        fps_imu = 200
        # enable ACCELEROMETER_RAW at 500 hz rate
        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, fps_imu)
        # enable GYROSCOPE_RAW at 400 hz rate
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, fps_imu)
        # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
        # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        self.imu.setBatchReportThreshold(1)
        # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        # if lower or equal to batchReportThreshold then the sending is always blocking on device
        # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        self.imu.setMaxBatchReports(10)
        # Link plugins IMU -> XLINK
        self.imu.out.link(self.xlinkOut.input)

    def start(self):
        self.device.startPipeline(self.pipeline)

        # Output queue for imu bulk packets
        self.imuQueue = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        self.imu = Imu()

        self.output_queue_dict = {}
        for camera_name in self.cam_list:
            self.output_queue_dict[camera_name] = self.device.getOutputQueue(name=camera_name, maxSize=1, blocking=False)

    def read_data(self):
        output_img = {}
        output_ts = {}
        for camera_name in self.cam_list:
            output_data = self.output_queue_dict[camera_name].tryGet()
            if output_data is not None:
                timestamp = output_data.getTimestampDevice()
                img = output_data.getCvFrame()
                img = cv2.rotate(img, cv2.ROTATE_180)
                output_img[camera_name] = img
                output_ts[camera_name] = timestamp
                # print(camera_name, timestamp, timestamp.microseconds, img.shape)
            else:
                # print(camera_name, 'No ouput')
                output_img[camera_name] = None
                output_ts[camera_name] = None
        return output_img, output_ts

if __name__ == '__main__':
    bridge = CvBridge()

    pub_dict = {}
    rospy.init_node('camera_imu_array', anonymous=True)
    rate = rospy.Rate(2000)
    for camera_name in ['cam_a', 'cam_b', 'cam_c', 'cam_d']:
        pub_dict[camera_name] = rospy.Publisher('/img/'+str(camera_name), Image, queue_size=0)
        pub_dict["imu"] = rospy.Publisher('/imu', Imu, queue_size=0)

    camera_imu_array = CameraImuArray()
    camera_imu_array.start()

    baseTs = None
    Ts = 0
    # dt_cam_time = -1
    # dt_imu_time = -1
    # dt_lidar_time = -1
    dt_time = -1

    time_now = rospy.Time.now()
    rospy.loginfo("当前时间为:%.2f", time_now.to_sec())

    controlQueue = camera_imu_array.device.getInputQueue(camera_imu_array.controlIn.getStreamName())

    # Defaults and limits for manual focus/exposure controls
    expTime = 500
    expMin = 1
    expMax = 33000

    sensIso = 500
    sensMin = 100
    sensMax = 1600

    expStep = 500  # us
    isoStep = 50

    expTime = clamp(expTime, expMin, expMax)
    sensIso = clamp(sensIso, sensMin, sensMax)
    print("Setting manual exposure, time:", expTime, "iso:", sensIso)
    ctrl = dai.CameraControl()
    ctrl.setManualExposure(expTime, sensIso)
    controlQueue.send(ctrl)
    
    while not rospy.is_shutdown():

        # key = cv2.waitKey(1)
        # if key == ord('q'):
        #     break
        # elif key == ord('e'):
        #     print("Autoexposure disable")
        #     ctrl = dai.CameraControl()
        #     ctrl.setAutoExposureLock(1)
        #     controlQueue.send(ctrl)
        # elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
        #     if key == ord('i'): expTime -= expStep
        #     if key == ord('o'): expTime += expStep
        #     if key == ord('k'): sensIso -= isoStep
        #     if key == ord('l'): sensIso += isoStep
        #     expTime = clamp(expTime, expMin, expMax)
        #     sensIso = clamp(sensIso, sensMin, sensMax)
        #     print("Setting manual exposure, time:", expTime, "iso:", sensIso)
        #     ctrl = dai.CameraControl()
        #     ctrl.setManualExposure(expTime, sensIso)
        #     controlQueue.send(ctrl)

        imuData = camera_imu_array.imuQueue.get()  # blocking call, will wait until a new data has arrived

        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope
            imu_time = acceleroValues.getTimestampDevice()

            if dt_time == -1:
                time_now = rospy.Time.now().to_sec()  # 当前ros时间
                imu_time = timeDeltaToS(imu_time)  # 当前加速度时间
                dt_time = time_now - imu_time  # 计算出当前加速度时间加上dt为当前ros时间（相机和IMU哪个先来就对齐哪个）
                print("imu_time:", imu_time)
                print("rospy.Time.now():", time_now)
                print("dt_time:", dt_time)
                print("")
                imu_time = time_now
            else:
                imu_time = timeDeltaToS(imu_time)
                imu_time += dt_time  # 加速度时间加上dt变为ros时间

        camera_imu_array.imu.linear_acceleration.x = acceleroValues.x
        camera_imu_array.imu.linear_acceleration.y = acceleroValues.y
        camera_imu_array.imu.linear_acceleration.z = acceleroValues.z
        camera_imu_array.imu.angular_velocity.x = gyroValues.x
        camera_imu_array.imu.angular_velocity.y = gyroValues.y
        camera_imu_array.imu.angular_velocity.z = gyroValues.z
        camera_imu_array.imu.header = Header()
        camera_imu_array.imu.header.stamp.secs = int(imu_time)
        camera_imu_array.imu.header.stamp.nsecs = int(
            (
                (imu_time)-int(imu_time)
            )*1000*1000*1000
        )

        output_img, output_ts = camera_imu_array.read_data()

        for key in output_img.keys():
            if output_img[key] is None:
                continue
        
        for key in output_img.keys():
            if output_img[key] is None:
                continue
            if dt_time == -1:
                time_now = rospy.Time.now().to_sec()  # 当前ros时间
                cam_time = timeDeltaToS(output_ts[key])  # 当前相机时间
                dt_time = time_now - cam_time  # 计算出当前相机时间加上dt为当前ros时间（相机和IMU哪个先来就对齐哪个）
                print(key, ":", cam_time)
                print("rospy.Time.now():", time_now)
                print("dt_time:", dt_time)
                print("")
                cam_time = time_now
            else:
                cam_time = timeDeltaToS(output_ts[key])
                cam_time += dt_time  # 加速度时间加上dt变为ros时间

            frame = output_img[key]
            # if key == "cam_d":
            #     cv2.imshow("cam_d", frame)
            img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img.header = Header()
            img.header.stamp.secs = int(cam_time)
            img.header.stamp.nsecs = int(
                (
                    (cam_time)-int(cam_time)
                )*1000*1000*1000
            )
            if dt_time != -1:
                pub_dict[key].publish(img)
        if dt_time != -1:
            pub_dict["imu"].publish(camera_imu_array.imu)
        rate.sleep()
