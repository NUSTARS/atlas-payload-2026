from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import time

app = Flask(__name__)


# ----- One global camera instance -----
picam2 = Picamera2(camera_num=0)
config = picam2.create_video_configuration(
    main={"size": (1920, 1080), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
time.sleep(1)  # camera warm-up


def generate_frames():
    while True:
        frame = picam2.capture_array()
        ret, jpeg = cv2.imencode(".jpg", frame)
        if not ret:
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )


@app.route("/")
def index():
    # Simple HTML page that shows the video stream
    return """
    <html>
      <head>
        <title>Pi Camera Stream</title>
      </head>
      <body>
        <img src="/video_feed" width="1920" height="1080" />
        <button onclick="">Start Video</button>
      </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    # Important: avoid reloader so it doesn't try to open the camera twice
    app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)