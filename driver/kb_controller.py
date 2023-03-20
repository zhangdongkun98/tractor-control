
import threading
import time
from pynput import keyboard
from driver.can import CanDriver


class KeyBoardController(object):
    def __init__(self, driver: CanDriver):
        self.driver = driver

        self.rotation_deg = 10

        self.stop_listen = threading.Event()
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.stop_listen.clear()
        self.listener.start()

    
    def callback_left(self):
        print(f'left {self.rotation_deg}')
        self.driver.set_rotation(-self.rotation_deg)
        
    def callback_right(self):
        print(f'right {self.rotation_deg}')
        self.driver.set_rotation(self.rotation_deg)

    def callback_up(self):
        print(f'up')
        self.driver.set_delta_gear( 500 / self.driver.max_gear )

    def callback_down(self):
        print(f'down')
        self.driver.set_delta_gear( -500 / self.driver.max_gear )



    def run(self):
        while not self.stop_listen.is_set():
            time.sleep(0.5)
        self.listener.stop()



    def on_press(self, key):
        if key == keyboard.Key.up:
            print(f'up')
            self.driver.set_delta_gear( 500 / self.driver.max_gear )
        if key == keyboard.Key.down:
            print(f'down')
            self.driver.set_delta_gear( -500 / self.driver.max_gear )
        if key == keyboard.Key.left:
            print(f'left {self.rotation_deg}')
            self.driver.set_rotation(-self.rotation_deg)
        if key == keyboard.Key.right:
            print(f'right {self.rotation_deg}')
            self.driver.set_rotation(self.rotation_deg)
        if key == keyboard.Key.esc:
            self.stop_listen.set()

        print('finish')
        return

