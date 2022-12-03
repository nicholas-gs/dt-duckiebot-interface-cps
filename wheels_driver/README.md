## wheels_driver

### Testing using keyboard

Send commands using a keyboard.

```bash
ros2 run wheels_driver wheels_driver_test_node --ros-args -p veh:=<robot name>
```

You should see the following printed out:

```txt
This node takes key presses from the keyboard and publishes them as WheelsCmdStamped and BoolStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
        w
   a    s    d
   z    x
e : increase velocity
q : decrease velocity
x : stop
z : (activate/deactivate) emergency stop
CTRL-C to quit
"""
```
