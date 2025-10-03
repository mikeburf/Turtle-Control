import pyglet.input as inp
import avgmethods

USE_MANAGER = False

# this uses the controller manager, which is more abstracted and therefore I trust it less
def main_manager():
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
        print(controller.is_open)
        print(controller.controls)
    input()
    print ("done.")

# this is more low-level
def main_devices():
    devices = inp.get_devices()
    for device in devices:
        print(device)
        print(device.is_open)
        print(device.controls)

if __name__ == "__main__":
    if USE_MANAGER: main_manager()
    else: main_devices()
