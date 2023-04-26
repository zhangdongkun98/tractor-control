
import numpy as np
from collections import deque

class LatPID(object):
    def __init__(self, dt):

        self._dt = dt

        self._k_p = 0.5
        self._k_i = 5 * dt  ## 0.0
        self._k_d = 0.01 / dt

        DELAY = 0.24165010452270508
        LAG = 0.7157812540648414
        lag = LAG
        delay = DELAY

        Kp = (dt + delay/2) / (lag + delay/2)
        Ki = dt + delay/2
        Kd = dt * delay / (delay + 2*dt)
        self._k_p = Kp
        # self._k_i = Ki * dt *100 *2
        # self._k_d = Kd / dt *0
        self._k_i = Ki * dt *2
        self._k_d = Kd / dt *0
        # self._k_d = Kd / dt /3
        # self._k_i = 0.0
        # self._k_d = 0.0
        print(f'PID parameter: Kp: {self._k_p}, Ki: {self._k_i}, Kd: {self._k_d}')

        # self._k_p = 0.5
        # self._k_i = 0.1  ## 0.0
        # self._k_d = 0.6

        self._e_buffer = deque(maxlen=1000)
        self._e_buffer.append(0.0)
        self._e_buffer.append(0.0)
        # self.w_param = None

    def run_step(self, error):
        _dot = error *10

        _ie = sum(self._e_buffer) + _dot
        _de = _dot - self._e_buffer[-1]
        self._e_buffer.append(_dot)


        # print('\n')
        # print('error (m): ', error)
        # print('pid error: ', _dot, _ie, _de)
        # print('pid: ', self._k_p * _dot, self._k_i * _ie, self._k_d * _de)

        res = self._k_p * _dot + self._k_i * _ie + self._k_d * _de
        return res, np.clip(res, -1.0, 1.0), self._k_p * _dot, self._k_i * _ie, self._k_d * _de




class IncrementalPID(LatPID):
    def __init__(self, dt):
        super().__init__(dt)
        self.last_u = 0.0

    def run_step(self, error):
        _dot = error *10


        delta_p = _dot - self._e_buffer[-1]
        delta_i = _dot
        delta_d = _dot + self._e_buffer[-2] - 2*self._e_buffer[-1]

        self._e_buffer.append(_dot)



        # print('\n')
        # print('error (m): ', error)
        # print('pid error: ', _dot, _ie, _de)
        # print('pid: ', self._k_p * _dot, self._k_i * _ie, self._k_d * _de)

        delta_u = self._k_p * delta_p + self._k_i * delta_i + self._k_d * delta_d
        res = delta_u + self.last_u
        self.last_u = np.clip(res, -1.0, 1.0)

        return res, np.clip(res, -1.0, 1.0), self._k_p * delta_p, self._k_i * delta_i, self._k_d * delta_d





pid = LatPID(1/50)
incremental_pid = IncrementalPID(1/50)



errors = [0.2] * 200
errors.append(-0.2)

for i, error in enumerate(errors):

    res, res_clip, res_p, res_i, res_d = pid.run_step(error)
    print(f'time {i}, res: {res}, res_clip: {res_clip}, res_p: {res_p}, res_i: {res_i}')

    res, res_clip, res_p, res_i, res_d = incremental_pid.run_step(error)
    print(f'time {i}, res: {res}, res_clip: {res_clip}, res_p: {res_p}, res_i: {res_i}')


    print()


