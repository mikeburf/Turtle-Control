import pyglet.input as inp
import avgmethods

USE_MANAGER = True

# this uses the controller manager, which is more abstracted and therefore I dont trust it
def main():
    manager = inp.ControllerManager()
    controllers = manager.get_controllers()

    # copied from pyglet docs
    @manager.event
    def on_connect(controller):
        # code to handle newly connected
        # (or re-connected) controllers
        controller.open()
        print("Connect:", controller)
        
    # copied from pyglet docs
    @manager.event
    def on_disconnect(controller):
        # code to handle disconnected Controller
        print("Disconnect:", controller)

    for controller in controllers:
        print(controller)
        
    input()
    print ("done.")

if __name__ == "__main__":
    main()
