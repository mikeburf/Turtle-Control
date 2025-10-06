import pyglet.input as inp


def main():
    devices = inp.get_devices()
    for device in devices:
        print(device.name)
        print(device.get_controls())
        print()

if __name__ == "__main__":
    main()