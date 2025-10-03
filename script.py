import pyglet
from pyglet.math import Vec2 as Vec2
import pyglet.input as inp
import avgmethods


class InputHandler:

    def __init__(self):

        self.inputs = dict() # map of controllers to current values
        print()
        self.debug()

    def on_stick_motion(self, controller, stick, vector):

        if(stick == "leftstick"):
            self.inputs[controller] = vector
            self.debug()

        

    def debug(self):
        print("\r" + str(sum(self.inputs.values())), end="")



# this uses the controller manager, which is more abstracted and therefore I dont trust it
def main():
    manager = inp.ControllerManager()
    handler = InputHandler()

    def on_connect(controller):
        controller.open()
        controller.rumble_play_weak(1.0, 0.1)
        print("Connected:", controller)
        controller.push_handlers(handler)


    def on_disconnect(controller):
        print("Disconnected:", controller)
        controller.remove_handlers(handler)

    
    manager.on_connect = on_connect
    manager.on_disconnect = on_disconnect

    if controllers := manager.get_controllers():
        on_connect(controllers[0])
        
    
    print ("done.")

if __name__ == "__main__":
    main()
    pyglet.app.run()
    print("exit.")
