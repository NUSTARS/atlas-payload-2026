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
        <img id="video" width="1920" height="1080" />
        <br><br>


        <br><br>
        <label for="exposure">Exposure (µs):</label>
        <input type="range"
               id="exposure"
               min="100"
               max="100000"
               value="10000"
               step="100"
               oninput="setExposure(this.value)">

        <span id="expValue">10000</span>

        <script>
          function startVideo() {
            document.getElementById("video").src = "/video_feed";
          }

          function setExposure(value) {
            document.getElementById("expValue").innerText = value;
            fetch("/set_exposure/" + value);
          }
        </script>
      </body>
    </html>
    """


@app.route("/set_exposure/<int:value>")
def set_exposure(value):
    # Clamp exposure to a safe range
    value = max(100, min(value, 100000))
    picam2.set_controls({"ExposureTime": value})
    return f"Exposure set to {value}"



@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    # Important: avoid reloader so it doesn't try to open the camera twice
    app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)