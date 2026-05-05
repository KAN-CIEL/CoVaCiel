# -*- coding: utf-8 -*-
import os
# --- TECHNIQUE 1 : Forcer la désactivation du buffer réseau OpenCV ---
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
import json
import cv2
import numpy as np
import time
import threading
from typing import Tuple, Optional

class VisionConfig:
    STREAM_URL = "rtsp://127.0.0.1:8554/cam"
    PROCESS_WIDTH = 160 
    MIN_CONTOUR_AREA = 800 

    RED_LOWER_1 = np.array([0, 100, 100])
    RED_UPPER_1 = np.array([10, 255, 255])
    RED_LOWER_2 = np.array([160, 100, 100])
    RED_UPPER_2 = np.array([180, 255, 255])
    
    GREEN_LOWER = np.array([40, 70, 70])
    GREEN_UPPER = np.array([80, 255, 255])

class ThreadedCamera:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        
        self.lock = threading.Lock()
        self.new_frame_available = False
        
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while not self.stopped:
            try:
                if self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        with self.lock:
                            self.ret = ret
                            self.frame = frame.copy()
                            self.new_frame_available = True
                else:
                    self.ret = False
            except Exception as e:
                self.ret = False
            
            time.sleep(0.005)

    def read(self):
        with self.lock:
            self.new_frame_available = False
            return self.ret, self.frame

    def release(self):
        self.stopped = True
        time.sleep(0.1)
        if self.cap:
            self.cap.release()

class ColorProcessor:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detecter_couleur(self, hsv_frame: np.ndarray, lower_bounds: list, upper_bounds: list, min_area: int) -> Tuple[Optional[str], Optional[tuple], np.ndarray]:
        mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        
        for lower, upper in zip(lower_bounds, upper_bounds):
            temp_mask = cv2.inRange(hsv_frame, lower, upper)
            mask = cv2.bitwise_or(mask, temp_mask)

        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return "null", None, mask_clean

        plus_gros_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(plus_gros_contour) < min_area:
            return "null", None, mask_clean

        M = cv2.moments(plus_gros_contour)
        if M["m00"] == 0:
            return "null", None, mask_clean
            
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        largeur_image = hsv_frame.shape[1]
        position = "GAUCHE" if cx < (largeur_image // 2) else "DROITE"

        return position, (cx, cy), mask_clean

def on_trackbar(val):
    pass

class CovaCielVision:
    def __init__(self):
        self.config = VisionConfig()
        self.processor = ColorProcessor()
        self.cam = None
        
        cv2.namedWindow("Calibration Couleurs", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Calibration Couleurs", 400, 200)
        
        cv2.createTrackbar("Vert Min Hue", "Calibration Couleurs", 40, 90, on_trackbar)
        cv2.createTrackbar("Vert Max Hue", "Calibration Couleurs", 80, 90, on_trackbar)
        
        self._connect_camera()

    def _connect_camera(self):
        print(f"[SYSTEM] Connexion 0-Latence en cours...")
        if self.cam is not None:
            self.cam.release()
        self.cam = ThreadedCamera(self.config.STREAM_URL)

    def run(self):
        print("[SYSTEM] Moteur temps réel DÉMARRÉ.")
        fps_timer = time.time()
        frame_count = 0
        
        while True:
            if not self.cam.new_frame_available:
                time.sleep(0.005)
                continue
                
            ret, frame = self.cam.read()
            
            if not ret or frame is None:
                time.sleep(1)
                self._connect_camera()
                continue

            ratio = self.config.PROCESS_WIDTH / frame.shape[1]
            dim = (self.config.PROCESS_WIDTH, int(frame.shape[0] * ratio))
            frame_res = cv2.resize(frame, dim)

            v_min_h = cv2.getTrackbarPos("Vert Min Hue", "Calibration Couleurs")
            v_max_h = cv2.getTrackbarPos("Vert Max Hue", "Calibration Couleurs")
            self.config.GREEN_LOWER[0] = v_min_h
            self.config.GREEN_UPPER[0] = v_max_h

            hsv = cv2.cvtColor(frame_res, (5, 5), 0), cv2.COLOR_BGR2HSV)

            pos_vert, coord_vert, mask_v = self.processor.detecter_couleur(
                hsv, [self.config.GREEN_LOWER], [self.config.GREEN_UPPER], self.config.MIN_CONTOUR_AREA
            )
            pos_rouge, coord_rouge, mask_r = self.processor.detecter_couleur(
                hsv, [self.config.RED_LOWER_1, self.config.RED_LOWER_2], 
                [self.config.RED_UPPER_1, self.config.RED_UPPER_2], self.config.MIN_CONTOUR_AREA
            )
            
            # --- ECRITURE PROPRE ET ALIGNÉE DU FICHIER JSON ---
            if not pos_vert: pos_vert = "null"
            if not pos_rouge: pos_rouge = "null"

            data_ia = {
                "vert": pos_vert,
                "rouge": pos_rouge
            }

            try:
                with open("vision_data.json", "w") as f:
                    json.dump(data_ia, f)
            except Exception as e:
                pass

            # --- RENDU VISUEL ---
            overlay = frame_res.copy()
            cv2.rectangle(overlay, (0, 0), (frame_res.shape[1], frame_res.shape[0]), (0, 0, 0), -1)
            frame_final = cv2.addWeighted(frame_res, 0.4, overlay, 0.6, 0)

            mask_total = cv2.bitwise_or(mask_v, mask_r)
            zones_colorees = cv2.bitwise_and(frame_res, frame_res, mask=mask_total)
            
            fond_sombre_nettoye = cv2.bitwise_and(frame_final, frame_final, mask=cv2.bitwise_not(mask_total))
            frame_final = cv2.add(fond_sombre_nettoye, zones_colorees)

            milieu_x = self.config.PROCESS_WIDTH // 2
            cv2.line(frame_final, (milieu_x, 0), (milieu_x, frame_final.shape[0]), (255, 255, 255), 1)

            def draw_text(img, text, pos, bg_color):
                cv2.putText(img, text, pos, cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(img, text, pos, cv2.FONT_HERSHEY_DUPLEX, 0.6, bg_color, 1)

            if pos_vert != "null":
                cv2.circle(frame_final, coord_vert, 10, (0, 255, 0), 2)
                draw_text(frame_final, f"VERT ({pos_vert})", (coord_vert[0]-30, coord_vert[1]-15), (150, 255, 150))
                
            if pos_rouge != "null":
                cv2.circle(frame_final, coord_rouge, 10, (0, 0, 255), 2)
                draw_text(frame_final, f"ROUGE ({pos_rouge})", (coord_rouge[0]-30, coord_rouge[1]-15), (150, 150, 255))

            frame_count += 1
            if time.time() - fps_timer > 1.0:
                print(f"\r[IA] FPS Réel: {frame_count:^3} | Vert: {pos_vert:^6} | Rouge: {pos_rouge:^6} ", end="", flush=True)
                frame_count = 0
                fps_timer = time.time()

            #cv2.imshow("Vision Dynamique", frame_final)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    app = CovaCielVision()
    app.run()
