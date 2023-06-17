import signal
from xbox360controller import Xbox360Controller
from math import floor

AXIS_THRESHOLD = 0.03
BUTTON_MAP = {
              "button_y": 1,
              "button_b": 2,
              "button_a": 3,
              "button_x": 4,
              "button_select": 5,
              "button_start": 6
             }
button_pressed = [False for _ in range(6)]
axis_value = [128, 128]

def debug_controller():
    print("=" * 20)
    print(axis_value)
    print(button_pressed)
    print("=" * 20)

def convert_axis(num):
    if (-AXIS_THRESHOLD < num < AXIS_THRESHOLD):
        return 128
    else:
        return floor(num * 127 + 128)

def on_button_pressed(button):
    button_pressed[BUTTON_MAP[button.name] - 1] = True
    debug_controller()

def on_button_released(button):
    button_pressed[BUTTON_MAP[button.name] - 1] = False
    debug_controller()

def on_axis_moved(axis):
    axis_value[0] = convert_axis(axis.x)
    axis_value[1] = convert_axis(axis.y)
    debug_controller()

try:
    with Xbox360Controller(0, axis_threshold=AXIS_THRESHOLD) as controller:
        # Button A events
        controller.button_a.when_pressed = on_button_pressed
        controller.button_a.when_released = on_button_released

        controller.button_b.when_pressed = on_button_pressed
        controller.button_b.when_released = on_button_released

        controller.button_x.when_pressed = on_button_pressed
        controller.button_x.when_released = on_button_released

        controller.button_y.when_pressed = on_button_pressed
        controller.button_y.when_released = on_button_released

        controller.button_start.when_pressed = on_button_pressed
        controller.button_start.when_released = on_button_released

        controller.button_select.when_pressed = on_button_pressed
        controller.button_select.when_released = on_button_released
        # Left and right axis move event
        controller.axis_l.when_moved = on_axis_moved
        # controller.axis_r.when_moved = on_axis_moved
        signal.pause()
except KeyboardInterrupt:
    pass