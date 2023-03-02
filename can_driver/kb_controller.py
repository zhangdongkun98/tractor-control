
import keyboard
from .driver import CanDriver

class KeyBoardController(object):
    def __init__(self, driver: CanDriver):
        self.driver = driver

        keyboard.add_hotkey('left', self.callback_left)
        keyboard.add_hotkey('right', self.callback_right)

        self.rotation_deg = 50
    
    def callback_left(self):
        print(f'left {self.rotation_deg}')
        self.driver.set_rotation(-self.rotation_deg)
        
    def callback_right(self):
        print(f'right {self.rotation_deg}')
        self.driver.set_rotation(self.rotation_deg)


    def run(self):
        keyboard.wait('esc')
