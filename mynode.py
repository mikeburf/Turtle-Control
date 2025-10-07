import pyglet

from pyglet.math import Vec2
import pyglet.input as inp
import threading
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

MAX_VEL_LINEAR = 0.22
MAX_VEL_ANGULAR = 3.0

MAX_DELTAV_LINEAR = 0.02
MAX_DELTAV_ANGULAR = 0.3

# this holds all the input data
class InputHandler:

    def __init__(self):

        self.inputs = dict() # map of controllers to states
        self.running = True
        self.status = 0
        print()

    def on_stick_motion(self, controller, stick, vector):

        if(stick == "leftstick"):
            self.inputs[controller] = vector

    def get_final_output(self):
        #print(len(self.inputs))
        if len(self.inputs) == 0: return Vec2(0, 0)
        total = Vec2(0, 0)
        for vec in self.inputs.values():
            total += vec
        return total / len(self.inputs)
            
    
def clamp(x, min, max):
    if x < min: return min
    if x > max: return max
    return x

def clamp_accel(current_v, target_v, max_delta):
    if current_v < target_v:
        return min(current_v + max_delta, target_v)
    if current_v > target_v:
        return max(current_v - max_delta, target_v)
    return current_v
    

def loop(handler, pub):

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    try:
        while (handler.running):
            indir = handler.get_final_output()
            target_linear_velocity = indir.y * MAX_VEL_LINEAR
            target_angular_velocity = indir.x * MAX_VEL_ANGULAR

            control_linear_velocity = clamp_accel(control_linear_velocity, target_linear_velocity, MAX_DELTAV_LINEAR)
            control_angular_velocity = clamp_accel(control_angular_velocity, target_angular_velocity, MAX_DELTAV_ANGULAR)

            twist = Twist()
            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print("Thread stopped!", e)
    finally:
        pass
            
        


# this uses the controller manager, which is more abstracted and therefore I dont trust it
def main():
    manager = inp.ControllerManager() # this handles hotplugging
    handler = InputHandler() # this handles controller

    # these will be called when a controller is plugged/unplugged
    # copied from some ros2 thing somewhere
    def on_connect(controller):
        controller.open()
        controller.rumble_play_weak(1.0, 0.1)
        print("\nConnected:", controller)
        handler.inputs[controller] = Vec2(0, 0)
        controller.push_handlers(handler)


    def on_disconnect(controller):
        print("\nDisconnected:", controller)
        handler.inputs.pop(controller)
        controller.remove_handlers(handler)

    def debug_callback(dt):
        print("\rCurrent Input:" + " "*50, handler.get_final_output())
        handler.status += 1

    
    manager.on_connect = on_connect
    manager.on_disconnect = on_disconnect

    pyglet.clock.schedule_interval(debug_callback, 1) # print the final output every second

    for controller in manager.get_controllers():
        on_connect(controller)

    rclpy.init()


        
    try:
        print("Ctrl-C to quit")
        qos = QoSProfile(depth=10)
        node = rclpy.create_node('mynode')
        pub = node.create_publisher(Twist, 'cmd_vel', qos)

        
        threading.Thread(target=loop, args=(handler, pub)).start()
        print("Thread started")
        pyglet.app.run() # runs the pyglet loop that handles input
        print("Pyglet finished")
    except KeyboardInterrupt:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        handler.running = False
        pass
    finally:
        handler.running = False
        
 
        #rclpy.shutdown()

    print ("done.")

if __name__ == "__main__":
    main()

