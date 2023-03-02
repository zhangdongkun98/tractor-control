import keyboard  #监听键盘

def test_a():
    print('aaa')

def test(x):
    print(x)

if __name__ == '__main__':
    keyboard.add_hotkey('a', test_a)
    #按f1输出aaa
    keyboard.add_hotkey('d', test, args=('b',))
    #按ctrl+alt输出b

    print('haha')
    keyboard.wait('esc')
    #wait里也可以设置按键，说明当按到该键时结束
