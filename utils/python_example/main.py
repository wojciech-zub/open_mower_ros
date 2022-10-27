from MowerInterface import MowerInterface
import time

# This is an example program to control the OpenMower via Python

if __name__ == '__main__':
    # Create a MowerInterface to talk to the mower, put the IP address of the mower here
    # That's all you need to do
    robot = MowerInterface("192.168.42.120")

    # wait for some status to arrive, so that we know that we're connected to the mower
    while not robot.has_status():
        print("waiting for robot")
        time.sleep(1.0)

    print("got data from robot, we're ready to go")

    # reset emergency to allow the robot to actually move.
    # emergency will be set on timeout and lift or emergency stop button.
    # You need to reset it using this function or the robot won't move!
    robot.reset_emergency()
    time.sleep(1)

    # drive forward with 20% power for one second
    robot.set_speeds(0.2, 0.0)
    time.sleep(1)
    # stop the mower
    robot.set_speeds(0.0, 0.0)
    time.sleep(1)
    # rotate in place for one second
    robot.set_speeds(0.0, 1.5)
    time.sleep(1)
    # stop the mower
    robot.set_speeds(0.0, 0.0)
    time.sleep(1)
    # rotate in place in the other direction for one second
    robot.set_speeds(0.0, -1.5)
    time.sleep(1)
    # stop the mower
    robot.set_speeds(0.0, 0.0)
    time.sleep(1)
    # drive back with 20% power for one second
    robot.set_speeds(-0.2, 0)
    time.sleep(1)
    # stop the mower
    robot.set_speeds(0.0, 0.0)
    time.sleep(1)
    # enable mowing motor for 5 sec
    robot.set_speeds(0.0, 0.0, True)
    time.sleep(5)
    # stop the mower
    robot.set_speeds(0.0, 0.0)


    while True:
        print("robot status: stamp: %s, emergency: %s, v-batt: %s, v-charge: %s, charge current:%s, ticks left: %s, ticks right: %s, gx: %s, gy: %s, gz: %s, ax: %s, ay: %s, az: %s" % (
            robot.get_timestamp(), robot.get_emergency(), robot.get_v_batt(), robot.get_v_charge(), robot.get_charge_current(), robot.get_ticks_left(), robot.get_ticks_right(), robot.get_imu_gx(), robot.get_imu_gy(), robot.get_imu_gz(), robot.get_imu_ax(), robot.get_imu_ay(), robot.get_imu_az())
        )
        time.sleep(0.5)

    robot.stop()
