import serial, time
def get_gains():
    s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    s.write(b"c ?\n")
    start = time.time()
    while time.time() - start < 2:
        print(s.readline().decode().strip())
    s.close()
get_gains()
