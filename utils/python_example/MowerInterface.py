import socket
import struct
import threading
import time


class _MowerSenderThread(threading.Thread):
    def __init__(self, mower_ip):
        super().__init__()
        self.mower_ip = mower_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.lock = threading.Lock()
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.mower_enabled = False
        self.stopped = False

    def set_speeds(self, linear, angular, mower_enabled):
        self.lock.acquire()
        self.speed_linear = linear
        self.speed_angular = angular
        self.mower_enabled = mower_enabled
        self.lock.release()

    def reset_emergency(self):
        self.lock.acquire()
        self.sock.sendto(struct.pack("<ffbb", 0, 0, 0, True),
                         (self.mower_ip, 4242))
        self.lock.release()

    def stop(self):
        self.lock.acquire()
        self.stopped = True
        self.lock.release()
        print("waiting for sender to stop")
        self.join()

    def run(self, *args, **kwargs):
        print("Started sender thread")
        while True:
            self.lock.acquire()
            if self.stopped:
                self.lock.release()
                break
            self.sock.sendto(struct.pack("<ffbb", self.speed_linear, self.speed_angular, self.mower_enabled, False),
                             (self.mower_ip, 4242))
            self.lock.release()
            time.sleep(0.02)
        print("Stopped sender thread")


class _MowerReceiverThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind(("0.0.0.0", 4243))
        self.lock = threading.Lock()
        self.v_batt = 0.0
        self.v_charge = 0.0
        self.charge_current = 0.0
        self.has_emergency = False
        self.stopped = False
        self.has_data = False

    def stop(self):
        self.lock.acquire()
        self.stopped = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
        except socket.error:
            pass
        self.lock.release()
        print("waiting for receiver to stop")
        self.join()

    def run(self, *args, **kwargs):
        print("Started receiver thread")
        while True:
            data = self.sock.recv(1000)
            self.lock.acquire()
            if self.stopped:
                self.lock.release()
                break
            if len(data) == 13:
                (self.has_emergency, self.v_charge, self.v_batt, self.charge_current) = struct.unpack("<bfff", data)
                self.has_data = True
            self.lock.release()
        print("Stopped receiver thread")

    def get_emergency(self):
        self.lock.acquire()
        val = self.has_emergency
        self.lock.release()
        return val

    def get_v_batt(self):
        self.lock.acquire()
        val = self.v_batt
        self.lock.release()
        return val

    def get_v_charge(self):
        self.lock.acquire()
        val = self.v_charge
        self.lock.release()
        return val

    def get_charge_current(self):
        self.lock.acquire()
        val = self.charge_current
        self.lock.release()
        return val

    def get_has_data(self):
        self.lock.acquire()
        val = self.has_data
        self.lock.release()
        return val


class MowerInterface:
    def __init__(self, mower_ip):
        self._sender = _MowerSenderThread(mower_ip)
        self._receiver = _MowerReceiverThread()
        self._sender.start()
        self._receiver.start()

    def set_speeds(self, linear, angular, mower_enabled=False):
        self._sender.set_speeds(linear, angular, mower_enabled)

    def reset_emergency(self):
        self._sender.reset_emergency()

    def stop(self):
        print("Stopping the Mower Interface")
        self._sender.stop()
        self._receiver.stop()

    def get_emergency(self):
        return self._receiver.get_emergency()

    def get_v_batt(self):
        return self._receiver.get_v_batt()

    def get_v_charge(self):
        return self._receiver.get_v_charge()

    def get_charge_current(self):
        return self._receiver.get_charge_current()

    def has_status(self):
        return self._receiver.get_has_data()
