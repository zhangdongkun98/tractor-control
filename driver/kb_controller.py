
import threading
import time
import tqdm
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
        print('Start keyboard')
        return

    
    def run(self):
        while not self.stop_listen.is_set():
            time.sleep(0.5)
        self.listener.stop()



    def on_press(self, key):
        print('\n')
        if key == keyboard.Key.up:
            print(f'press up')
            self.driver.set_delta_gear( 500 / self.driver.max_gear )
            print('finish up')
        if key == keyboard.Key.down:
            print(f'down')
            self.driver.set_delta_gear( -500 / self.driver.max_gear )
        if key == keyboard.Key.left:
            print(f'left {self.rotation_deg}')
            self.driver.set_rotation(-self.rotation_deg)
        if key == keyboard.Key.right:
            print(f'right {self.rotation_deg}')
            self.driver.set_rotation(self.rotation_deg)

        if hasattr(key, 'char') and key.char == 'w':
            print(f'press up')
            self.driver.set_delta_gear( 500 / self.driver.max_gear )
            print('finish up')
        if hasattr(key, 'char') and key.char == 's':
            print(f'down')
            self.driver.set_delta_gear( -500 / self.driver.max_gear )
        if hasattr(key, 'char') and key.char == 'a':
            print(f'left {self.rotation_deg}')
            self.driver.set_rotation(-self.rotation_deg)
        if hasattr(key, 'char') and key.char == 'd':
            print(f'right {self.rotation_deg}')
            self.driver.set_rotation(self.rotation_deg)

        if key == keyboard.Key.esc:
            self.stop_listen.set()
        return

