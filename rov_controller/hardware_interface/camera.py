import cv2
import time
import logging

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')

# --- Example GStreamer Pipelines for Raspberry Pi CSI Cameras ---
# These can vary based on Pi version, camera module (v1, v2, HQ), and installed libraries.
# You might need to adjust `sensor-id` or other parameters.

# For libcamera stack (newer Raspberry Pi OS - Bullseye and later)
# This uses the `libcamerasrc` GStreamer element.
# `! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1`
RPI_CSI_LIBCAMERA_PIPELINE = (
    "libcamerasrc ! "
    "video/x-raw,width={width},height={height},framerate={fps}/1 ! "
    "videoconvert ! "
    "appsink drop=true max-buffers=1"  # drop=true and max-buffers=1 for latest frame
)
# A more complete example with BGR format for OpenCV:
RPI_CSI_LIBCAMERA_PIPELINE_BGR = (
    "libcamerasrc ! "
    # NV12 is a common CSI output
    "video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
)


# For older MMAL stack (legacy Raspberry Pi OS - Buster or older with legacy camera enabled)
# This uses `nvarguscamerasrc` (NVIDIA on Jetson often) or `v4l2src` (sometimes for CSI on Pi if configured that way)
# The more direct approach for MMAL was often through picamera library, but OpenCV can use GStreamer.
# A common GStreamer pipeline for CSI using V4L2 (if the CSI camera appears as /dev/videoX):
# `v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! appsink`
# However, for CSI direct access on older Pi OSes, `picamera` library was often easier than crafting a GStreamer pipeline for OpenCV.
# If using `libcamera` is an option (newer OS), it's generally preferred for CSI.

class CameraControllerCV:
    """
    Manages a camera (USB or CSI via GStreamer on Raspberry Pi) using OpenCV.
    """

    def __init__(self, camera_id_or_pipeline: int | str = 0,
                 width: int = 640, height: int = 480, fps: int = 30,
                 api_preference=cv2.CAP_ANY):
        """
        Initializes the camera.

        Args:
            camera_id_or_pipeline (int | str):
                - For USB cameras: Integer ID (e.g., 0, 1).
                - For CSI cameras (Raspberry Pi): A GStreamer pipeline string.
                  You can use RPI_CSI_LIBCAMERA_PIPELINE_BGR.format(width=..., height=..., fps=...)
                  or provide your own.
            width (int): Desired capture width.
            height (int): Desired capture height.
            fps (int): Desired capture frames per second.
            api_preference: OpenCV backend to prefer (e.g., cv2.CAP_V4L2, cv2.CAP_GSTREAMER).
                            cv2.CAP_ANY lets OpenCV decide.
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.camera_id_or_pipeline = camera_id_or_pipeline
        self.cap = None
        self.width = width
        self.height = height
        self.fps = fps
        self.api_preference = api_preference

        self.connect()

    def _get_gst_pipeline(self) -> str | None:
        """
        Returns a GStreamer pipeline string if self.camera_id_or_pipeline is a known key
        or if it's already a GStreamer pipeline string.
        """
        if isinstance(self.camera_id_or_pipeline, str):
            if "!" in self.camera_id_or_pipeline:  # Likely already a GStreamer pipeline
                return self.camera_id_or_pipeline
            # Add known keys for predefined pipelines if desired
            # elif self.camera_id_or_pipeline.upper() == "CSI_LIBCAMERA":
            #     return RPI_CSI_LIBCAMERA_PIPELINE_BGR.format(width=self.width, height=self.height, fps=self.fps)
        return None  # Not a GStreamer pipeline string or known key

    def connect(self):
        """
        Establishes the camera connection.
        """
        self.logger.info(
            f"Attempting to open camera: {self.camera_id_or_pipeline}")

        gst_pipeline = self._get_gst_pipeline()

        try:
            if gst_pipeline:
                self.logger.info(f"Using GStreamer pipeline: {gst_pipeline}")
                self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            elif isinstance(self.camera_id_or_pipeline, int):
                self.logger.info(
                    f"Using camera index: {self.camera_id_or_pipeline} with API: {self.api_preference}")
                self.cap = cv2.VideoCapture(
                    self.camera_id_or_pipeline, self.api_preference)
            else:
                self.logger.error(
                    f"Invalid camera_id_or_pipeline type: {type(self.camera_id_or_pipeline)}. Must be int or GStreamer string.")
                return

            if not self.cap.isOpened():
                self.logger.error(
                    f"Failed to open camera: {self.camera_id_or_pipeline}")
                self.cap = None
                return

            # Try to set camera properties
            # Note: Not all cameras/backends support setting all properties,
            # and some properties must be set before opening via the GStreamer pipeline itself.
            if self.cap:
                if self.width:
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                if self.height:
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                if self.fps:
                    self.cap.set(cv2.CAP_PROP_FPS, self.fps)

                # Verify what was actually set (optional, reading can be slow)
                actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
                self.logger.info(f"Camera opened. Requested: {self.width}x{self.height} @ {self.fps}fps. "
                                 f"Actual: {int(actual_width)}x{int(actual_height)} @ {actual_fps:.2f}fps.")
                self.width = int(actual_width)  # Update with actual values
                self.height = int(actual_height)

        except Exception as e:
            self.logger.error(f"Exception during camera connection: {e}")
            if self.cap:
                self.cap.release()
            self.cap = None

    def is_opened(self) -> bool:
        """Checks if the camera is opened and ready."""
        return self.cap is not None and self.cap.isOpened()

    def read_frame(self) -> tuple[bool, cv2.typing.MatLike | None]:
        """
        Reads a frame from the camera.

        Returns:
            tuple[bool, MatLike | None]: (success, frame).
            'success' is True if a frame was read, False otherwise.
            'frame' is the captured OpenCV frame (NumPy array) if successful, else None.
        """
        if not self.is_opened():
            # self.logger.warning("Attempted to read frame, but camera is not open.")
            return False, None

        assert self.cap is not None
        ret, frame = self.cap.read()
        if not ret:
            # self.logger.warning("Failed to retrieve frame from camera.")
            return False, None
        return True, frame

    def get_properties(self) -> dict:
        """
        Gets current camera properties.
        Note: Reading properties frequently can sometimes be slow or unreliable
              depending on the OpenCV backend and camera.
        """
        if not self.is_opened():
            return {
                "width": self.width,  # Return desired/last known if not open
                "height": self.height,
                "fps": self.fps,
                "is_opened": False
            }

        assert self.cap is not None
        return {
            "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": self.cap.get(cv2.CAP_PROP_FPS),
            "brightness": self.cap.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": self.cap.get(cv2.CAP_PROP_CONTRAST),
            "saturation": self.cap.get(cv2.CAP_PROP_SATURATION),
            "hue": self.cap.get(cv2.CAP_PROP_HUE),
            "gain": self.cap.get(cv2.CAP_PROP_GAIN),
            "exposure": self.cap.get(cv2.CAP_PROP_EXPOSURE),
            "is_opened": True
        }

    def release(self):
        """
        Releases the camera resource.
        """
        if self.cap is not None:
            self.logger.info("Releasing camera.")
            self.cap.release()
            self.cap = None
        self.logger.info("Camera released.")

    def __del__(self):
        """Ensures camera is released when the object is deleted."""
        self.release()


# Example Usage
if __name__ == "__main__":
    # --- Option 1: Try to use a generic USB camera (e.g., /dev/video0) ---
    print("Attempting to open USB camera (index 0)...")
    # For USB, you can specify desired resolution, though it might not always be respected perfectly.
    usb_cam = CameraControllerCV(
        camera_id_or_pipeline=0, width=1280, height=720, fps=30)

    if usb_cam.is_opened():
        print("USB Camera opened successfully.")
        print(f"Properties: {usb_cam.get_properties()}")
        try:
            for i in range(60):  # Read 60 frames
                success, frame = usb_cam.read_frame()
                if success:
                    # cv2.imshow("USB Camera Frame", frame) # Requires a GUI environment
                    if i % 10 == 0:  # Log every 10 frames
                        print(f"Read USB frame {i+1}, shape: {frame.shape}")
                    # if cv2.waitKey(1) & 0xFF == ord('q'): # Allows q to quit if imshow is active
                    #     break
                    time.sleep(1/30)  # Simulate processing delay
                else:
                    print("Failed to read frame from USB camera.")
                    break
        finally:
            # cv2.destroyAllWindows() # If using imshow
            usb_cam.release()
            print("USB Camera released.")
    else:
        print("Failed to open USB camera.")

    print("\n" + "="*30 + "\n")

    # --- Option 2: Try to use a Raspberry Pi CSI camera via libcamera GStreamer pipeline ---
    # This part will likely only work on a Raspberry Pi with a CSI camera and libcamera correctly configured.
    print("Attempting to open Raspberry Pi CSI camera (using example libcamera pipeline)...")
    csi_width = 1280
    csi_height = 720
    csi_fps = 30

    # Note: platform.machine() can return 'aarch64' or 'armv7l' on Raspberry Pi
    # This is just an example; robust platform detection might be more involved.
    # if "arm" in platform.machine().lower() or "aarch64" in platform.machine().lower():
    # This example pipeline should be modified if your setup requires different parameters or elements.
    csi_pipeline = RPI_CSI_LIBCAMERA_PIPELINE_BGR.format(
        width=csi_width, height=csi_height, fps=csi_fps)

    # To test this part, you'd uncomment it and run on a Pi.
    # For non-Pi systems, this will likely fail to open.
    # csi_cam = CameraControllerCV(camera_id_or_pipeline=csi_pipeline,
    #                              width=csi_width, height=csi_height, fps=csi_fps,
    #                              api_preference=cv2.CAP_GSTREAMER)

    # if csi_cam.is_opened():
    #     print("CSI Camera opened successfully.")
    #     print(f"Properties: {csi_cam.get_properties()}")
    #     try:
    #         for i in range(60): # Read 60 frames
    #             success, frame = csi_cam.read_frame()
    #             if success:
    #                 # cv2.imshow("CSI Camera Frame", frame)
    #                 if i % 10 == 0:
    #                     print(f"Read CSI frame {i+1}, shape: {frame.shape}")
    #                 # if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 #     break
    #                 time.sleep(1/30)
    #             else:
    #                 print("Failed to read frame from CSI camera.")
    #                 break
    #     finally:
    #         # cv2.destroyAllWindows()
    #         csi_cam.release()
    #         print("CSI Camera released.")
    # else:
    #     print("Failed to open CSI camera (or not on a compatible Raspberry Pi setup).")
    print("Skipping CSI camera test for this example run unless explicitly uncommented and on a Pi.")
