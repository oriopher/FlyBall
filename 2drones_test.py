from tello import Tello
import time

if __name__ == '__main__':
    t1 = Tello(iface_ip="192.168.10.2")
    t2 = Tello(iface_ip="192.168.10.3")
    t1.connect()
    t2.connect()
    time.sleep(5)
    t1.takeoff()
    t2.takeoff()
    t0 = time.time()
    while time.time() - t0 < 10:
        t1.send_rc_control(0, 0, 0, 0)
        t2.send_rc_control(0, 0, 0, 0)
    t1.land()
    t2.land()
