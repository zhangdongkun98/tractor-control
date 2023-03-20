
import keyboard
from driver.can import CanDriver


class KeyBoardController(object):
    def __init__(self, driver: CanDriver):
        self.driver = driver

        keyboard.add_hotkey('left', self.callback_left)
        keyboard.add_hotkey('right', self.callback_right)
        keyboard.add_hotkey('up', self.callback_up)
        keyboard.add_hotkey('down', self.callback_down)

        self.rotation_deg = 10
    
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
        keyboard.wait('esc')
