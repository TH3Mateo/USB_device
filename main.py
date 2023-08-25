
import importlib.util as ilu
import time
spec1 = ilu.spec_from_file_location("coms",r"D:\PycharmProjects\heater_gui\connection_module.py")
coms = ilu.module_from_spec(spec1)
spec1.loader.exec_module(coms)
spec2 = ilu.spec_from_file_location("hal",r"D:\PycharmProjects\heater_gui\dev_spec_layer.py")
hal = ilu.module_from_spec(spec2)
spec2.loader.exec_module(hal)

msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00,0x00]

def main():
    print("main")
    usb = coms.USB_device()
    usb.start()
    print("init done")
    usb.transmit_queue.put(msg)
    while True:
        while usb.receive_queue.qsize() > 0:
            print(usb.receive_queue.get())
        x = input("enter command: ")
        if len(x) ==2:
            msg[0] = int(x[0])
            msg[-1] = int(x[1])
            usb.transmit_queue.put(msg)
            time.sleep(0.5)
        else:
            print("invalid command")
        # usb.connection.write(msg)
main()

# print(str.encode("5"))