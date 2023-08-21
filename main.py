
import importlib.util as ilu
spec1 = ilu.spec_from_file_location("coms",r"C:\\Users\\M\Desktop\\python_code\\softHeater\\connection_module.py")
coms = ilu.module_from_spec(spec1)
spec1.loader.exec_module(coms)
spec2 = ilu.spec_from_file_location("hal",r"C:\\Users\\M\Desktop\\python_code\\softHeater\\dev_spec_layer.py")
hal = ilu.module_from_spec(spec2)
spec2.loader.exec_module(hal)

def main():
    usb = coms.USB_device()
    usb.start()
    while True:
        while usb.receive_queue.qsize() > 0:
            print(usb.receive_queue.get())
        
        x = input("enter command: ")
        usb.transmit_queue.put(x)

main()