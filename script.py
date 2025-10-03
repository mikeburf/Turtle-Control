import pyglet
from pyglet.math import Vec2
import pyglet.input as inp
import avgmethods

# this holds the state of a specific controller
class ControllerState:
    def __init__(self):
        self.vector = Vec2(0, 0)

# this holds all the input data
class InputHandler:

    def __init__(self):

        self.inputs = dict() # map of controllers to states
        print()

    def on_stick_motion(self, controller, stick, vector):

        if(stick == "leftstick"):
            self.inputs[controller].vector = vector



# this uses the controller manager, which is more abstracted and therefore I dont trust it
def main():
    manager = inp.ControllerManager() # this handles hotplugging
    handler = InputHandler() # this handles controller states

    # these will be called when a controller is plugged/unplugged
    # copied from some ros2 thing somewhere
    def on_connect(controller):
        controller.open()
        controller.rumble_play_weak(1.0, 0.1)
        print("Connected:", controller)
        handler.inputs[controller] = ControllerState()
        controller.push_handlers(handler)


    def on_disconnect(controller):
        print("Disconnected:", controller)
        handler.inputs.pop(controller)
        controller.remove_handlers(handler)

    
    manager.on_connect = on_connect
    manager.on_disconnect = on_disconnect

    # initializes already connected controllers
    if controllers := manager.get_controllers():
        on_connect(controllers[0])
        
    pyglet.app.run() # runs the pyglet loop that handles input
    print ("done.")

if __name__ == "__main__":
    main()

    print("exit.")
