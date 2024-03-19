import time
import copy
import select
from pathlib import Path
import jsonpickle
import requests

from picamera2 import Picamera2, Preview
from libcamera import Transform


# import json
# import serial
# import socket
# import imagezmq

bluetooth_poll = select.poll()
bluetooth_poll_registered = False
bluetooth_path = Path('/dev/rfcomm0')
bluetooth_file = None

def handle_blueooth_read():
    global bluetooth_poll
    global bluetooth_poll_registered
    global bluetooth_file

    if not bluetooth_poll_registered:
        print("Attempting to set up bluetooth")
        if bluetooth_path.exists():
            bluetooth_file = open(bluetooth_path, 'r')#open('/dev/rfcomm0')
            bluetooth_poll.register(bluetooth_file)
            bluetooth_poll_registered = True
            print("{} poll set up".format(bluetooth_path))
        else:
            print("{} not present".format(bluetooth_path))
    
    poll_out = bluetooth_poll.poll(0.01)
    #print("Bluetooth poll: {}".format(poll_out))
    try:
        print("{} changed? {}".format(bluetooth_path, poll_out[0][1] & select.POLLIN))
    except:
        pass

    received_line = None
    if len(poll_out) > 0 and (poll_out[0][1] & select.POLLIN):
        try:
            received_line = bluetooth_file.readline()
            print(received_line)
            if received_line != '':
                print("Bluetooth received: {}".format(received_line))
            else:
                print("Nothing received")
        except:
            bluetooth_poll_registered = False
            bluetooth_poll.unregister(bluetooth_file)
            bluetooth_poll = select.poll()
            bluetooth_file = None
            print("{} no longer present".format(bluetooth_path))
    
    return received_line

bluetooth_write_queue = []
def handle_blueooth_writes():
    global bluetooth_write_queue
    if bluetooth_file is not None:
        with open(bluetooth_path, 'wb') as f:
            while len(bluetooth_write_queue) > 0:
                try:
                    to_write = bluetooth_write_queue[0]
                    to_write = to_write.encode(encoding='ascii')
                    f.write(to_write)
                    print("Wrote to Bluetooth: {}".format(to_write))
                    bluetooth_write_queue.pop(0)
                except:
                    return False
    return True

mapping_dict = {
    'Bullseye': 'bullseye',
    'id36': '',
    'id39': 'right_arrow',
    'id40': 'left_arrow'
}

def perform_classification():
    print("Taking image with camera and sending for classification")
    res = requests.get(url='http://localhost:5000/do_classification')
    model_out = jsonpickle.decode(res.content)
    if len(model_out['cls']) == 0:
        detected_class = 'None'
    else:
        detected_class = mapping_dict[model_out['names'][str(int(model_out['cls'][0]))]]
    print("First class detected: {}", detected_class)
    return detected_class

#stm_poll = select.poll()
#stm_poll_registered = False
stm_path = Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0002-if00-port0')
#stm_file = None
def perform_stm_write(to_write):
    to_write = to_write.encode(encoding='ascii')
    if stm_path.exists():
        with open(stm_path, 'wb') as f:
            f.write(to_write)
    else:
        with open(Path.home() / 'stm_surrogate_out', 'wb') as f:
            f.write(to_write)
    print("Sent command to STM: {}".format(to_write))

# Globals
START = False
TASK_FINISH = False

""" ULTRASONIC """
STOP_DISTANCE = 70
DISTANCE_TRAVELED = 0
ULTRASOUND_DISTANCE = 1000

# set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# ================================================
# Code for Ultrasonic
def get_distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

def movement_task():
    global START, DISTANCE_TRAVELED, ULTRASOUND_DISTANCE, CV_RESPONSE, TASK_FINISH

    while not START:
        tablet_data = TABLET_SER.readline().decode().strip()
        if tablet_data:
            print("========== RECEIVED START FROM ANDROID TABLET ==========")
            START = True


    while START and not TASK_FINISH:
        print("========== MOVEMENT TASK STARTED ==========")
        approach_obstacle_and_advance(25,80, True)
        DISTANCE_TRAVELED = 0
        current_response = perform_classification()
        if(current_response == "left_arrow"):
            perform_stm_write("nq042")
            perform_stm_write("ne043")
            time.sleep(0.1)

            approach_obstacle_and_advance(45,100, True)

            perform_stm_write("ne015")
            current_response = perform_classification()
            if current_response == "left_arrow":
                perform_stm_write("nc015")
                task_LL()
                task_endR()
            elif current_response == "right_arrow":
                task_LR()
                task_endL()
            else:
                print("No L or R recognized, retrying")
                perform_stm_write("ne015")
                current_response = perform_classification()
                if current_response == "left_arrow":
                    perform_stm_write("nc015")
                    perform_stm_write("nc015")
                    task_LL()
                    task_endR()
                elif current_response == "right_arrow":
                    perform_stm_write("nc015")
                    task_LR()
                    task_endL()
                

        elif (current_response == "right_arrow"):
            perform_stm_write("ne042")
            perform_stm_write("nq043")
            time.sleep(0.1)
            approach_obstacle_and_advance(45,100, True)
            perform_stm_write("nq015")
            current_response = perform_classification()
            if current_response == "left_arrow":
                task_RL()
                task_endR()
                break
            elif current_response == "right_arrow":
                perform_stm_write("nz015")
                task_RR()
                task_endL()
                break
            else:
                print("No L or R recognized, retrying")
                perform_stm_write("nq015")
                current_response = perform_classification()
                if current_response == "left_arrow":
                    perform_stm_write("nz015")
                    task_RL()
                    task_endR()
                    break
                elif current_response == "right_arrow":
                    perform_stm_write("nz015")
                    perform_stm_write("nz015")
                    task_RR()
                    task_endL()
                    break
                
        else:
            print("No left_arrow or R recognized")

# test passed
def task_LL():
    perform_stm_write("nq043")
    perform_stm_write("nw020")
    perform_stm_write("ne133")
    perform_stm_write("nw040")
    perform_stm_write("ne089")


# test passed
def task_LR():
    perform_stm_write("ne072")
    perform_stm_write("nw001")
    perform_stm_write("nq177")
    perform_stm_write("nw047")
    perform_stm_write("nq090")

# test passed
def task_RL():
    perform_stm_write("nq072")
    perform_stm_write("nw001")
    perform_stm_write("ne180")
    perform_stm_write("nw038")
    perform_stm_write("ne088")

# test passed
def task_RR():
    perform_stm_write("ne043")
    perform_stm_write("nw015")
    perform_stm_write("nq132")
    perform_stm_write("nw047")
    perform_stm_write("nq088")

def task_endL():

    global DISTANCE_TRAVELED, TASK_FINISH
    distance_command = DISTANCE_TRAVELED + 32

    perform_stm_write("nw{:03}".format(int(distance_command)))
    DISTANCE_TRAVELED = 0

    perform_stm_write("nq043")
    perform_stm_write("nw040")
    perform_stm_write("ne045")  
    perform_stm_write("ns000")
    time.sleep(0.5) 


    approach_obstacle_and_advance(18, 60, False)
    TASK_FINISH = True


def task_endR():
    global DISTANCE_TRAVELED, TASK_FINISH
    distance_command = DISTANCE_TRAVELED + 38
    perform_stm_write("nw{:03}".format(int(distance_command)))
    DISTANCE_TRAVELED = 0

    perform_stm_write("ne043")
    perform_stm_write("nw033")
    perform_stm_write("nq047")
    perform_stm_write("ns000")
    time.sleep(1)

    approach_obstacle_and_advance(18, 60, False)
    TASK_FINISH = True

def approach_obstacle_and_advance(target_distance, dash_distance, back_conpensate):
    global DISTANCE_TRAVELED, ULTRASOUND_DISTANCE

    desired_stop = False
    while not desired_stop:
        print("========== APPROACHING OBSTACLE ==========")
        ULTRASOUND_DISTANCE = get_distance()
        print("Ulltrasonic reads: " + str(ULTRASOUND_DISTANCE))
        time.sleep(0.006)
        if ULTRASOUND_DISTANCE >= dash_distance:  # it's safe to dash
            perform_stm_write("nw{:03}".format(dash_distance-10))
            DISTANCE_TRAVELED += dash_distance-10
        elif (
            ULTRASOUND_DISTANCE <= dash_distance
            and ULTRASOUND_DISTANCE > target_distance
        ):
            perform_stm_write("nw{:03}".format(round(ULTRASOUND_DISTANCE - target_distance),0))
            desired_stop = True
            DISTANCE_TRAVELED += round((ULTRASOUND_DISTANCE - target_distance),0)
        elif ULTRASOUND_DISTANCE < target_distance and ULTRASOUND_DISTANCE > 2:

            if(back_conpensate):
                perform_stm_write("nx{:03}".format(round(target_distance - ULTRASOUND_DISTANCE),0))
                DISTANCE_TRAVELED -= round((ULTRASOUND_DISTANCE - target_distance),0)
            desired_stop = True
            
        else:
            print("========== Unexpected Behaviour! ==========")
            perform_stm_write("ns000")


def main():
    try:
        print("Sending dummy command")
        perform_stm_write("ns000")
        print("========== ROBOT RESETED, WAITING FOR START ==========")
        movement_task()

    except KeyboardInterrupt:
        print("Stopped by User - Keyboard Interrupt")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()  # yeet.