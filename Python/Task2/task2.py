import copy
import select
import time
from pathlib import Path
import requests
import jsonpickle

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
    
    
def main_loop():
	# W1000 -> go straight 10cm
	# A1050 -> go 10cm left, 50 degreee
	# S 
	# D	

	"""first obstacle will be 10cm by 10cm"""
	case_l = [
		"""first image recognition = 'left"""
		'S2000', # move 20cm backwards
		'A4045', # Turn left 40cm, 45 degree
		'D4060', # Turn Right 40cm, 60 degree
		'A3045', # Turn Left 30cm, 45 degree
		# 'FL057', 'FR057',

		"""move towards to 2nd obstacle"""
		'W3000', # Move forward, towards 2nd obstacle
		# 'US030',  # move until 30cm before 2nd obstacle and save d to buff 0
		# 'FR020',
	]

	case_r = [
		"""first image recognition = 'right'"""
		'S2000', # move 20cm backwards
		'D4045', # Turn Right 40cm, 45 degree
		'A4060', # Turn Left 40cm, 60 degree
		'D3045', # Turn Right 30cm, 45 degree
		# 'FR057', 'FL057',

		"""move over to 2nd obstacle"""
		'W3000', # Move forward, towards 2nd obstacle
		# 'US030',  # move until 30cm before 2nd obstacle and save d to buff 0
		# 'FL020',
	]

	"""Second obstacle will be 60cm by 10cm"""
	case_ll = [ # 1st image: left, 2nd image: left
		"""second image recognition = 'left'"""
		'S2000', # move 20cm backwards
		'A4090', # Turn left 40cm, 90 degree
		'D2090', # Turn right 20cm, 90 degree
		# roundabout the second obstacle
		# 'D7090', # Turn right 70cm, 90 degree
		'D1090', # Turn right 10cm, 90 degree
		'W7000', # Go straight for 70cm
		'D1090', # Turn right 10cm, 90 degree

		"""back to base (carpark)"""
		'W',
		# 'US120', # move until 20cm before parking wall
		# 'US120'  # move until 20cm before parking wall
	]

	case_lr = [ # 1st image: left, 2nd image: right
		"""second image recognition = 'right'"""
  		'S2000', # move 20cm backwards
		'', # Turn right
		'W2000', # Go straight 20cm
		'A3090', # Turn left 30cm, 180 degree
		'A3090', 
		# can turn A3000?
		'W6000', # Go straight
		'A3090', # Turn left 30cm, 90 degree

		"""back to base (carpark)"""
		# 'US120', # move until 20cm before parking wall
		# 'US120'  # move until 20cm before parking wall
	]

	case_rr = [ # 1st image: right, 2nd image: right
		"""second image recognition = 'right'"""
		# 'BL020', 'FR067', 'FL067', 'FW025', 'FL090', 'FW060', 'FL090',
		# 'RT070',  # d + 70
		# 'FL090', 'FR090',

		"""back to base (carpark)"""
		# 'US120', # move until 20cm before parking wall
		# 'US120'  # move until 20cm before parking wall
	]

	case_rl = [ # # 1st image: right, 2nd image: left
		"""second image recognition = 'left'"""
		# 'FL070', 'FW025', 'FR090', 'FW025', 'FR090', 'FW060', 'FR090',
		# 'RT075',  # d + 75
		# 'FR090', 'FL090',

		"""back to base (carpark)"""
		# 'US120', # move until 20cm before parking wall
		# 'US120'  # move until 20cm before parking wall
	]

	# instr_list = []
	init = ['W5000'] # move 50cm forward  
	# will stop 10cm before obstacle
	

	"""CHANGE ACCORDINGLY, TESTING"""
	# img1 = "left_arrow"
	# img2 = "left_arrow"

	perform_stm_write(init[0])
	
	# image = img_capture()
	# img1 = process_img(image)
	img1 = perform_classification()
	print(f'image1:{img1}')
 
	"""continue moving forward if there is no image detected"""
	while img1 == "":
		perform_stm_write(init[0])
		img1 = perform_classification()
		print(f'image1:{img1}')
  

	if "right_arrow" in img1:
		# move_stm(case_r)
		for instr in case_r:
			print(f'instruction:{instr}')
			perform_stm_write(instr=instr)
		# time.sleep(10)
		# image2 = img_capture()
		# img2 = process_img(image2)
		img2 =  perform_classification()
		print(f'image2:{img2}')

	elif "left_arrow" in img1:
		# move_stm(case_l)
		for instr in case_l:
			print(f'instruction:{instr}')
			perform_stm_write(instr=instr)
		# time.sleep(10)
		# image2 = img_capture()
		# img2 = process_img(image2)
		img2 =  perform_classification()
		print(f'got img2:{img2}')


	# Checking if the first image is left and the second image is left.
	if "left_arrow" in img1 and "left_arrow" in img2:
		for instr in case_ll:
			perform_stm_write(instr)

	# Checking if the first image is left and the second image is right.
	if "left_arrow" in img1 and "right_arrow" in img2:
		for instr in case_lr:
			perform_stm_write(instr)

	# Checking if the first image is right and the second image is left.
	if "right_arrow" in img1 and "left_arrow" in img2:
		for instr in case_rl:
			perform_stm_write(instr)

	# Checking if the first image is right and the second image is right.
	if "right_arrow" in img1 and "right_arrow" in img2:
		for instr in case_rr:
			perform_stm_write(instr)

    

main_loop()