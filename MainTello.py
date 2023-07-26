from djitellopy import tello
import time
import KeyboardTelloModule as kp
import cv2

# Function to perform face recognition
def detect_faces(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return image

def getKeyboardInput():
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
    # Control The Drone
    Drone.send_rc_control(keyValues[0], keyValues[1], keyValues[2], keyValues[3])
    # Get Frame From Drone Camera
    img = Drone.get_frame_read().frame
    img = cv2.resize(img, (720, 480))

    # Perform Face Recognition
    img_with_faces = detect_faces(img)

    # Show The Frame
    cv2.imshow("DroneCapture", img_with_faces)
    cv2.waitKey(1)

    battery_percentage = Drone.get_battery()
    print(f"Batteriestatus: {battery_percentage}%")
