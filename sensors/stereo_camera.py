"""Dual IMX219 stereo capture on Raspberry Pi 5 using Picamera2 (libcamera).

The Pi 5 has two CSI ports (CAM0 / CAM1), so each IMX219 is opened as its own camera.
Note: the two cameras are NOT hardware-synchronised. read() returns each frame's
sensor timestamp so you can see how far apart left and right were captured.
"""
from picamera2 import Picamera2


class StereoCamera:
    def __init__(self, width=640, height=480, fps=30, left_index=0, right_index=1):
        found = Picamera2.global_camera_info()
        if len(found) < 2:
            raise RuntimeError(
                f"Need 2 cameras, found {len(found)}: {found}. "
                "Check both ribbon cables (Pi 5 uses the smaller 22-pin connector) "
                "and run `rpicam-hello --list-cameras`."
            )

        self.cams = []
        for idx in (left_index, right_index):
            cam = Picamera2(idx)
            config = cam.create_video_configuration(
                # "RGB888" in Picamera2 is stored as B,G,R per pixel -> correct colours in OpenCV
                main={"size": (width, height), "format": "RGB888"},
                controls={"FrameRate": fps},
            )
            cam.configure(config)
            self.cams.append(cam)

        for cam in self.cams:
            cam.start()

    def read(self):
        """Return (left_bgr, right_bgr, left_stamp_ns, right_stamp_ns)."""
        frames, stamps = [], []
        for cam in self.cams:
            req = cam.capture_request()
            try:
                frames.append(req.make_array("main"))
                stamps.append(req.get_metadata().get("SensorTimestamp", 0))
            finally:
                req.release()
        return frames[0], frames[1], stamps[0], stamps[1]

    def close(self):
        for cam in self.cams:
            cam.stop()
            cam.close()
