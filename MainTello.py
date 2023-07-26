from djitellopy import tello
import time
import KeyboardTelloModule as kp
import cv2

# Function to calculate the horizontal and vertical differences between the face center and the image center
def calculate_differences(image, faces):
    image_height, image_width, _ = image.shape
    face = faces[0]  # Assuming only one face is detected
    face_center_x = face[0] + face[2] // 2
    face_center_y = face[1] + face[3] // 2
    image_center_x = image_width // 2
    image_center_y = image_height // 2

    print("image_center", image_center_x)
    print("face_center", face_center_x)

    print("image_center", image_center_y)
    print("face_center", face_center_y)

    horizontal_diff = face_center_x - image_center_x
    vertical_diff = face_center_y - image_center_y
    return horizontal_diff[0], vertical_diff[0]  # Access the first element of the arrays

# Function to control the drone's movement to follow the face
def follow_face(horizontal_diff, vertical_diff, max_speed=50):
    lr, fb, ud, yv = 0, 0, 0, 0
    print(horizontal_diff, vertical_diff)
    horizontal_speed = int(horizontal_diff / 10)  # Scale the horizontal difference to adjust movement speed
    vertical_speed = int(vertical_diff / 10)      # Scale the vertical difference to adjust movement speed

    # Left/Right movement
    if horizontal_diff > 10:
        lr = min(horizontal_speed, max_speed)
    elif horizontal_diff < -10:
        lr = max(horizontal_speed, -max_speed)

    # Up/Down movement (maintain a constant 1-meter distance)
    desired_vertical_diff = 120  # Adjust this value based on your setup to maintain a 1-meter distance
    if vertical_diff > desired_vertical_diff + 10:
        ud = min(vertical_speed, max_speed)
    elif vertical_diff < desired_vertical_diff - 10:
        ud = max(vertical_speed, -max_speed)

    print(lr, fb, ud, yv)
    return [lr, fb, ud, yv]



# Function to perform face recognition
def detect_faces(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return image

follow_face_mode = False

def getKeyboardInput():
    global follow_face_mode
    # LEFT RIGHT, FRONT BACK, UP DOWN, YAW VELOCITY
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 80
    liftSpeed = 80
    moveSpeed = 85
    rotationSpeed = 100

    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed

    if kp.getKey("UP"):
        fb = moveSpeed
    elif kp.getKey("DOWN"):
        fb = -moveSpeed

    if kp.getKey("w"):
        ud = liftSpeed
    elif kp.getKey("s"):
        ud = -liftSpeed

    if kp.getKey("d"):
        yv = rotationSpeed
    elif kp.getKey("a"):
        yv = -rotationSpeed

    if kp.getKey("q"):
        Drone.land()
        time.sleep(3)
    elif kp.getKey("e"):
        Drone.takeoff()

    if kp.getKey("z"):
        # Screen Shot Image From The Camera Display
        cv2.imwrite(f"tellopy/Resources/Images/{time.time()}.jpg", img)
        time.sleep(0.3)

    if kp.getKey("f"):
        # Trigger Flip
        flip("f")

    if kp.getKey("g"):
        # Trigger Flip
        flip("b")

    if kp.getKey("h"):
            # Trigger Flip
        flip("r")

    if kp.getKey("j"):
            # Trigger Flip
        flip("l")

    if kp.getKey("p"):
        follow_face_mode = not follow_face_mode  # Toggle follow_face_mode on/off

    return [lr, fb, ud, yv]

def flip(Richtung): Drone.flip(Richtung)

# Initialize Keyboard Input
kp.init()

# Start Connection With Drone
Drone = tello.Tello()
Drone.connect()

# Start Camera Display Stream
Drone.streamon()

while True:
    # Get The Return Value And Stored It On Variable
    keyValues = getKeyboardInput()

    # Get Frame From Drone Camera
    img = Drone.get_frame_read().frame
    img = cv2.resize(img, (720, 480))

    # Perform Face Recognition
    faces = detect_faces(img)

    if follow_face_mode and len(faces) > 0:
        # Calculate differences
        horizontal_diff, vertical_diff = calculate_differences(img, faces)
        # Adjust the drone's movement to follow the face
        keyValues = follow_face(horizontal_diff, vertical_diff)

    # Control The Drone
    Drone.send_rc_control(keyValues[0], keyValues[1], keyValues[2], keyValues[3])


    # Show The Frame
    cv2.imshow("DroneCapture", img)
    cv2.waitKey(1)


    battery_percentage = Drone.get_battery()
    print(f"Batteriestatus: {battery_percentage}%")
