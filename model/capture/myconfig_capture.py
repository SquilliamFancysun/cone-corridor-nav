"""DonkeyCar overrides for cone-capture runs. Lives in ~/mycar/ on the car.

    python manage.py drive --myconfig=myconfig_capture.py

During capture DonkeyCar is the drive-by-wire stack and nothing else: it reads
the F710 and drives the VESC. It must NOT touch the OAK-D, because only one
process can hold that device and capture_cones.py needs it.

Two lines below do that work; everything else is copied from myconfig.py so the
car handles identically to a normal drive.
"""

# --- the two that matter ------------------------------------------------------

# Release the OAK-D. MockCamera feeds the vehicle loop a blank frame so the
# pipeline is happy while capture_cones.py owns the real camera.
CAMERA_TYPE = "MOCK"

# Do not write tubs. Capture sessions are the dataset; a half-recorded tub of
# blank MockCamera frames alongside them is just confusing.
AUTO_RECORD_ON_THROTTLE = False

# --- copied from myconfig.py, unchanged --------------------------------------

DRIVE_LOOP_HZ = 20
MAX_LOOPS = None

IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3

DRIVE_TRAIN_TYPE = "VESC"
VESC_MAX_SPEED_PERCENT = .2
VESC_SERIAL_PORT = "/dev/ttyACM0"
VESC_HAS_SENSOR = True
VESC_START_HEARTBEAT = True
VESC_BAUDRATE = 115200
VESC_TIMEOUT = 0.05
VESC_STEERING_SCALE = 0.5
VESC_STEERING_OFFSET = 0.5

USE_JOYSTICK_AS_DEFAULT = True
JOYSTICK_MAX_THROTTLE = 0.5
JOYSTICK_STEERING_SCALE = 1.0
CONTROLLER_TYPE = 'F710'
USE_NETWORKED_JS = False
NETWORK_JS_SERVER_IP = None
JOYSTICK_DEADZONE = 0.01
JOYSTICK_THROTTLE_DIR = -1.0
JOYSTICK_DEVICE_FILE = "/dev/input/js0"

USE_FPV = False
