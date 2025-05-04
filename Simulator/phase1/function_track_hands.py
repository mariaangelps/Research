import cv2
import mediapipe as mp

def track_hands(image, mp_hands, hands):
    # Convert image to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Process the image and find hand landmarks
    results = hands.process(image)

    # Initialize list of hand coordinates
    hand_coords = []

    # Loop through detected hands
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Get the coordinates of the wrist landmark
            x = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y
            y = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x

            # Normalize the coordinates to be between 0 and 1
            x_pix = x * image.shape[0]
            y_pix = y * image.shape[1]

            # Append the normalized coordinates to the list of hand coordinates
            hand_coords.append((x_pix, y_pix))

    return hand_coords