import socket
import struct
import threading
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 4242


class MowerInterface(threading.Thread):
    def __init__(self, sock):
        super().__init__()
        self.power_left = 0.0
        self.power_right = 0.0
        self.power_mow = 0.0
        self.sock = sock
        self.stopped = False

    def run(self, *args, **kwargs):
        while not self.stopped:
            self.sock.sendto(struct.pack("<fff", self.power_left, self.power_right, self.power_mow), (UDP_IP, UDP_PORT))
            time.sleep(0.1)

    def stop(self):
        self.stopped = True


if __name__ == '__main__':
    print("UDP target IP:", UDP_IP)
    print("UDP target port:", UDP_PORT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    robot = MowerInterface(sock)
    robot.start()
    while True:
        time.sleep(1)
        robot.power_left = 1.0
        time.sleep(1)
        robot.power_left = 0.0

    robot.stop()
