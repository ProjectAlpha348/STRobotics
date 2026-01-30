#!/usr/bin/env python3
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from ament_index_python.packages import get_package_share_directory


def open_camera(camera_index: int, camera_device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = None

    if camera_device:
        cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)

    if cap is None or not cap.isOpened():
        cap = cv2.VideoCapture(int(camera_index), cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError(f"Impossibile aprire camera (index={camera_index}, device='{camera_device}')")

    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))

    # riduce buffering su molte webcam
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # ---------------- Params
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_device", "")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("target_class", -1)          # -1 tutte, 0 person, ...
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("rate_hz", 30.0)

        self.declare_parameter("model_relpath", "models/yolo/yolov8n.pt")

        # ---- NEW: OpenCV window (showimage)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "Tommy - Vision (YOLO)")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_device = str(self.get_parameter("camera_device").value)
        self.w = int(self.get_parameter("image_width").value)
        self.h = int(self.get_parameter("image_height").value)

        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.iou_thres = float(self.get_parameter("iou_thres").value)
        self.target_class = int(self.get_parameter("target_class").value)

        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        if self.rate_hz <= 0:
            self.rate_hz = 30.0

        # ---- NEW
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self._window_open = False

        # ---------------- Model path
        share = get_package_share_directory("robot_head")
        model_relpath = str(self.get_parameter("model_relpath").value).lstrip("/")
        self.model_path = os.path.join(share, model_relpath)
        if not os.path.exists(self.model_path):
            raise RuntimeError(f"Modello YOLO non trovato: {self.model_path}")

        # ---------------- ROS pubs/subs
        self.pub_det = self.create_publisher(Detection2DArray, "/tommy/vision/detections", 10)
        self.pub_img = self.create_publisher(Image, "/tommy/vision/image_annotated", qos_profile_sensor_data)
        self.pub_fps = self.create_publisher(Float32, "/tommy/vision/fps", 10)

        self.pub_state = self.create_publisher(String, "/tommy/vision/state", 10)

        self.sub_enable = self.create_subscription(Bool, "/tommy/vision/enable", self._on_enable, 10)
        self.sys_sub = self.create_subscription(String, "/robot_head/system/cmd", self._on_system_cmd, 10)

        self.bridge = CvBridge()

        # ---------------- State
        self.enabled = True  # default
        self._state = "idle"
        self._publish_state("idle")

        # ---------------- YOLO load
        from ultralytics import YOLO
        self.get_logger().info(f"Carico YOLO: {self.model_path}")
        self.model = YOLO(self.model_path)

        # ---------------- Camera (aperta solo se enabled)
        self.cap = None
        if self.enabled:
            try:
                self._ensure_camera_open()
                self._ensure_window_open()
                self._publish_state("running")
            except Exception as e:
                self.get_logger().warning(f"Camera non disponibile all'avvio: {e}")
                self._publish_state("error")

        self.last_t = time.perf_counter()
        self.fps_ema = 0.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info("vision_node avviato.")

    # ----------------- State helpers
    def _publish_state(self, s: str):
        self._state = s
        m = String()
        m.data = self._state
        self.pub_state.publish(m)

    # ----------------- Window helpers (NEW)
    def _ensure_window_open(self):
        if not self.show_window:
            return
        if self._window_open:
            return
        # Se non c'è display (headless), imshow potrebbe fallire: gestiamo con try/except.
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self._window_open = True
        except Exception as e:
            self.get_logger().warning(f"Impossibile creare finestra OpenCV (headless?): {e}")
            self._window_open = False

    def _close_window(self):
        if not self._window_open:
            return
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass
        self._window_open = False

    # ----------------- Camera helpers
    def _ensure_camera_open(self):
        # chiamare SOLO quando enabled=True
        if self.cap is not None and self.cap.isOpened():
            return
        self.get_logger().info(
            f"Apro camera: index={self.camera_index}, device='{self.camera_device}', {self.w}x{self.h}"
        )
        self.cap = open_camera(self.camera_index, self.camera_device, self.w, self.h)

    def _release_camera(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.cap = None

    # ----------------- Callbacks
    def _on_enable(self, msg: Bool):
        new_enabled = bool(msg.data)
        if new_enabled == self.enabled:
            return

        self.enabled = new_enabled
        self.get_logger().info(f"Vision enable = {self.enabled}")

        if not self.enabled:
            # DISATTIVAZIONE REALE: rilascio immediato della webcam + chiusura finestra
            self._release_camera()
            self._close_window()
            self._publish_state("idle")
        else:
            # riattivazione: tenta riapertura + riapri finestra
            try:
                self._ensure_camera_open()
                self._ensure_window_open()
                self._publish_state("running")
            except Exception as e:
                self.get_logger().warning(f"Impossibile riaprire camera: {e}")
                self._publish_state("error")

    def _on_system_cmd(self, msg: String):
        if msg.data.strip().lower() == "shutdown":
            self.get_logger().info("Shutdown richiesto: stop timer, rilascio camera, chiusura nodo.")
            try:
                if self.timer is not None:
                    self.timer.cancel()
            except Exception:
                pass

            self.enabled = False
            self._release_camera()
            self._close_window()

            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

            self.destroy_node()
            rclpy.shutdown()

    # ----------------- Main loop
    def _tick(self):
        # GUARD RAIL: se disabilitato, NON fare nulla e soprattutto NON riaprire la camera
        if not self.enabled:
            return

        # enabled=True da qui in poi

        # se camera non aperta, prova ad aprire
        if self.cap is None or not self.cap.isOpened():
            try:
                self._ensure_camera_open()
                self._ensure_window_open()
            except Exception as e:
                self.get_logger().warn(f"Camera non disponibile: {e}")
                self._publish_state("error")
                return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Frame non disponibile (rilascio camera, stato=error).")
            self._release_camera()
            self._close_window()
            self._publish_state("error")
            return

        if self._state != "running":
            self._publish_state("running")

        results = self.model.predict(
            source=frame,
            conf=self.conf_thres,
            iou=self.iou_thres,
            verbose=False
        )

        det_msg = self._to_detection2d_array(results, frame.shape[1], frame.shape[0])
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.header.frame_id = "camera_frame"
        self.pub_det.publish(det_msg)

        annotated = None
        if self.publish_annotated:
            annotated = results[0].plot()

            # publish su topic solo se richiesto
            if self.pub_img.get_subscription_count() > 0:
                img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                img_msg.header = det_msg.header
                self.pub_img.publish(img_msg)

        # ---- NEW: showimage locale (sempre quando enabled + show_window)
        if self.show_window and self._window_open:
            try:
                to_show = annotated if annotated is not None else frame
                cv2.imshow(self.window_name, to_show)
                # indispensabile per far “respirare” la finestra
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warning(f"imshow/waitKey fallita (chiudo finestra): {e}")
                self._close_window()

        # FPS EMA
        now = time.perf_counter()
        dt = now - self.last_t
        self.last_t = now
        inst_fps = (1.0 / dt) if dt > 0 else 0.0
        self.fps_ema = (0.9 * self.fps_ema) + (0.1 * inst_fps)

        m = Float32()
        m.data = float(self.fps_ema)
        self.pub_fps.publish(m)

    def _to_detection2d_array(self, results, img_w: int, img_h: int) -> Detection2DArray:
        out = Detection2DArray()

        if not results:
            return out

        r0 = results[0]
        if r0.boxes is None:
            return out

        boxes = r0.boxes
        xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else boxes.xyxy
        conf = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else boxes.conf
        cls = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else boxes.cls

        for (x1, y1, x2, y2), score, c in zip(xyxy, conf, cls):
            c_id = int(c)
            if self.target_class >= 0 and c_id != self.target_class:
                continue

            x1 = float(max(0.0, min(x1, img_w - 1)))
            y1 = float(max(0.0, min(y1, img_h - 1)))
            x2 = float(max(0.0, min(x2, img_w - 1)))
            y2 = float(max(0.0, min(y2, img_h - 1)))

            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            cx = x1 + (w / 2.0)
            cy = y1 + (h / 2.0)

            det = Detection2D()
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(c_id)
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)

            out.detections.append(det)

        return out

    def destroy_node(self):
        self.enabled = False
        self._release_camera()
        self._close_window()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

