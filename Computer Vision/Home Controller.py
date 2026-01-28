import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
import numpy as np
import time

# --- MQTT Setup ---
MQTT_BROKER = "broker.hivemq.com"
TOPIC_GROUND = "home/ground_floor"
TOPIC_FIRST = "home/first_floor"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) 
client.connect(MQTT_BROKER, 1883, 60)

# --- Mediapipe Setup ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# --- Image Auth Setup ---
ref_path = r"C:\Users\HP\OneDrive\Desktop\Computer Vision\Face_ID\persons\Mazen_Sarhan.jpg"
ref_img = cv2.imread(ref_path)
MATCH_THRESHOLD = 30 
COOLDOWN_DURATION = 3.0 

orb = cv2.ORB_create(nfeatures=1500)
kp1, des1 = orb.detectAndCompute(ref_img, None)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

cap = cv2.VideoCapture(0)
authorized = False
# الحالات الحالية (نبدأ بـ OFF)
states = {TOPIC_FIRST: "OFF", TOPIC_GROUND: "OFF"}
last_action_time = 0 

print(f"🔒 النظام مقفل.. بانتظار صورة البيت...")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape
    current_time = time.time()
    
    if not authorized:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp2, des2 = orb.detectAndCompute(gray, None)
        matches_count = 0
        if des2 is not None:
            matches = sorted(bf.match(des1, des2), key=lambda x: x.distance)
            matches_count = len([m for m in matches if m.distance < 45])
            if matches_count > MATCH_THRESHOLD:
                authorized = True
                print("✅ تم الفتح!")
        cv2.putText(frame, f"LOCKED - {matches_count}/{MATCH_THRESHOLD}", (20, 50), 1, 1.5, (0,0,255), 2)
    else:
        # رسم الواجهة
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 255), 2)
        cv2.putText(frame, f"1st: {states[TOPIC_FIRST]}", (10, 40), 1, 1.5, (255,255,0), 2)
        cv2.putText(frame, f"Ground: {states[TOPIC_GROUND]}", (10, h-20), 1, 1.5, (255,255,0), 2)
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)
        
        can_action = (current_time - last_action_time) > COOLDOWN_DURATION

        if results.multi_hand_landmarks and can_action:
            for hand_lms in results.multi_hand_landmarks:
                y_tip = hand_lms.landmark[8].y * h 
                
                # تحديد التوبيك بناءً على مكان الإيد
                target_topic = TOPIC_FIRST if y_tip < h // 2 else TOPIC_GROUND
                
                # عكس الحالة (Toggle)
                new_state = "ON" if states[target_topic] == "OFF" else "OFF"
                
                client.publish(target_topic, new_state)
                states[target_topic] = new_state
                last_action_time = current_time
                
                icon = "🟢" if new_state == "ON" else "🔴"
                print(f"{icon} [SWITCH] {target_topic} is now {new_state}")
                
                mp_drawing.draw_landmarks(frame, hand_lms, mp_hands.HAND_CONNECTIONS)

        # عرض عداد الـ Cooldown
        if not can_action:
            wait_t = round(COOLDOWN_DURATION - (current_time - last_action_time), 1)
            cv2.putText(frame, f"WAIT: {wait_t}s", (w-180, h//2+10), 1, 1.5, (0, 165, 255), 2)

    cv2.imshow("Smart Switch System", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
