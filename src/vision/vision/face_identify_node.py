#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# --- CONFIG FISSA (semplice) ---
CAM_DEV = 0
WIDTH, HEIGHT, FPS = 1280, 720, 30
FOURCC = "MJPG"

# MediaPipe short-range + min detection score richiesto
MP_MODEL = 0
MIN_CONF = 0.79

# Recognition
COSINE_THRESHOLD = 0.45
VOTE_FRAMES = 7
MIN_FACE_PX = 60

EMB_SIZE = (112, 112)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def l2_normalize(x, eps=1e-12):
    n = float(np.linalg.norm(x))
    return x if n < eps else x / n


def preprocess_for_arcface(face_bgr):
    face_rs = cv2.resize(face_bgr, EMB_SIZE, interpolation=cv2.INTER_AREA)
    face_rgb = cv2.cvtColor(face_rs, cv2.COLOR_BGR2RGB).astype(np.float32)
    face_rgb = (face_rgb - 127.5) / 128.0
    return face_rgb[None, :, :, :]  # NHWC


def load_db_mean(path, dim):
    if not os.path.exists(path):
        return [], np.empty((0, dim), dtype=np.float32)

    data = np.load(path, allow_pickle=True)
    names = data["names"].tolist()
    sums = data["sums"].astype(np.float32)

    if sums.ndim != 2 or sums.shape[1] != dim:
        raise RuntimeError(f"DB incompatibile: sums.shape={sums.shape}, expected_dim={dim}")

    means = np.zeros_like(sums)
    for i in range(sums.shape[0]):
        means[i] = l2_normalize(sums[i])

    return names, means


class FaceIdentifyNode(Node):
    def __init__(self):
        super().__init__("face_identify_node")

        # Publisher: una stringa "name cx cy"
        self.pub = self.create_publisher(String, "face_id", 10)

        # Percorsi dal package share
        share = get_package_share_directory("vision")
        model_path = os.path.join(share, "models", "arcface.onnx")
        db_path = os.path.join(share, "faces_db", "db_mean.npz")

        # ONNXRuntime + MediaPipe
        import onnxruntime as ort
        import mediapipe as mp

        ort.set_default_logger_severity(3)
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Modello ONNX non trovato: {model_path}")
        if not os.path.exists(db_path):
            raise FileNotFoundError(f"DB non trovato: {db_path}")

        self.sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.out_name = self.sess.get_outputs()[0].name

        dummy = np.zeros((1, 112, 112, 3), dtype=np.float32)
        emb_dim = self.sess.run([self.out_name], {self.in_name: dummy})[0].reshape(-1).shape[0]
        self.names, self.mean_embs = load_db_mean(db_path, emb_dim)

        self.vote_win = deque(maxlen=VOTE_FRAMES)

        # Camera
        self.cap = cv2.VideoCapture(CAM_DEV, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)

        if not self.cap.isOpened():
            raise RuntimeError("Camera non disponibile")

        # Face detector
        self.mp_fd = mp.solutions.face_detection
        self.detector = self.mp_fd.FaceDetection(
            model_selection=MP_MODEL,
            min_detection_confidence=MIN_CONF
        )

        # Timer ~30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

        self.get_logger().info("vision/face_identify_node avviato. Pubblico su topic: /face_id (std_msgs/String)")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        h, w = frame.shape[:2]
        res = self.detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if not res.detections:
            return

        # Prendo la detection 0 (puoi migliorare scegliendo la migliore se vuoi)
        d = res.detections[0]
        bb = d.location_data.relative_bounding_box

        x1 = clamp(int(bb.xmin * w), 0, w - 1)
        y1 = clamp(int(bb.ymin * h), 0, h - 1)
        x2 = clamp(int((bb.xmin + bb.width) * w), 0, w - 1)
        y2 = clamp(int((bb.ymin + bb.height) * h), 0, h - 1)

        bw, bh = x2 - x1, y2 - y1
        cx, cy = int(x1 + bw / 2), int(y1 + bh / 2)

        name = "UNKNOWN"
        if bw >= MIN_FACE_PX and bh >= MIN_FACE_PX and len(self.names) > 0:
            face = frame[y1:y2, x1:x2]
            if face.size > 0:
                inp = preprocess_for_arcface(face)
                emb = self.sess.run([self.out_name], {self.in_name: inp})[0].reshape(-1)
                emb = l2_normalize(emb)

                sims = self.mean_embs @ emb
                idx = int(np.argmax(sims))
                if float(sims[idx]) >= COSINE_THRESHOLD:
                    self.vote_win.append(self.names[idx])
                    if len(self.vote_win) == VOTE_FRAMES:
                        name = max(set(self.vote_win), key=self.vote_win.count)

        msg = String()
        msg.data = f"{name} {cx} {cy}"
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.detector.close()
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FaceIdentifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
