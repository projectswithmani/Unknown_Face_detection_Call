import cv2
import face_recognition
import serial
import time

# Initialize serial communication
try:
    arduino = serial.Serial('COM12', 9600)  # Change COM port if needed
    time.sleep(2)  # Allow connection time
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    arduino = None

# Load and encode reference images
try:
    ref_img1 = face_recognition.load_image_file(r"E:\Codeplayground\Servo pyt\m3.jpg")
    ref_img2 = face_recognition.load_image_file(r"E:\Codeplayground\Servo pyt\virat.jpeg")

    ref_enc1 = face_recognition.face_encodings(ref_img1)[0]
    ref_enc2 = face_recognition.face_encodings(ref_img2)[0]

except Exception as e:
    print(f"Error loading reference images: {e}")
    ref_enc1 = ref_enc2 = None

# Start video capture
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

frame_count = 0
last_buzzer_state = False  # Prevents continuous buzzing

# Function to process frames
def process_frame(frame):
    global last_buzzer_state

    small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)  # Reduce size to increase speed
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    match_found = False

    for face_encoding, face_location in zip(face_encodings, face_locations):
        match1 = face_recognition.compare_faces([ref_enc1], face_encoding, tolerance=0.5)
        match2 = face_recognition.compare_faces([ref_enc2], face_encoding, tolerance=0.5)

        if True in match1 or True in match2:
            match_found = True
            top, right, bottom, left = [v * 2 for v in face_location]  # Scale back up
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, "Access Granted", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # If an unregistered face is detected
    if not match_found and len(face_encodings) > 0:
        if not last_buzzer_state and arduino:
            arduino.write(b'2')  # Send signal to activate buzzer
            print("🚨 Unknown Face Detected - Buzzer Activated")
            last_buzzer_state = True
    else:
        last_buzzer_state = False  # Reset state when no face is detected

    return frame

# Main loop
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_count % 10 == 0:  # Process every 10th frame for smoothness
            try:
                frame = process_frame(frame)
            except Exception as e:
                print(f"Error in face processing: {e}")

        frame_count += 1
        cv2.imshow('Face Recognition', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    cap.release()
    cv2.destroyAllWindows()
    if arduino:
        arduino.close()

