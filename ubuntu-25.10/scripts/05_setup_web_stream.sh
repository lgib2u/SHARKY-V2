#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Ubuntu 25.10: setting up web stream (Flask + Picamera2)..."

APP_DIR="/opt/sharky-webcam"
SERVICE_NAME="sharky_webcam.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

echo "Attempting to install Flask + Picamera2..."
remote_sudo "apt update"
if ! remote_sudo "apt install -y python3-flask python3-picamera2"; then
  echo "python3-picamera2 not available on this Ubuntu build; skipping web stream setup."
  exit 0
fi

echo "Verifying Picamera2 import..."
if ! remote "python3 -c 'from picamera2 import Picamera2'"; then
  echo "Picamera2 unavailable at runtime; skipping web stream setup."
  exit 0
fi

echo "Deploying web app to ${APP_DIR} ..."
remote_sudo "mkdir -p ${APP_DIR}"
remote "cat > /tmp/app.py <<'PYEOF'
import io
import threading
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

INDEX_HTML = '''
<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Sharky Camera</title>
    <style>
      body { font-family: system-ui, sans-serif; margin: 0; background: #111; color: #eee; }
      header { padding: 12px 16px; background: #222; border-bottom: 1px solid #333; }
      main { padding: 12px 16px; display: flex; justify-content: center; }
      img { width: 100%; max-width: 800px; background: #000; }
      .meta { opacity: .8; font-size: 12px; margin-top: 8px; text-align: center; }
      a { color: #8cc0ff; }
    </style>
  </head>
  <body>
    <header><strong>Sharky</strong> — Camera Stream</header>
    <main>
      <div>
        <img src="/stream.mjpg" alt="Stream">
        <div class="meta">MJPEG via Picamera2 · <a href="/snapshot.jpg">snapshot</a></div>
      </div>
    </main>
  </body>
</html>
'''

app = Flask(__name__)
picam2 = Picamera2()
# Keep it light
video_config = picam2.create_video_configuration(main={'size': (640, 480)})
picam2.configure(video_config)

class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = bytes(buf)
            self.condition.notify_all()
        return len(buf)

output = StreamingOutput()

def start_camera_once():
    if not picam2.started:
        picam2.start_recording(MJPEGEncoder(), FileOutput(output))

@app.route('/')
def index():
    start_camera_once()
    return render_template_string(INDEX_HTML)

@app.route('/stream.mjpg')
def stream_mjpg():
    start_camera_once()
    def gen():
        while True:
            with output.condition:
                output.condition.wait()
                frame = output.frame
            yield (b'--frame\\r\\n'
                   b'Content-Type: image/jpeg\\r\\n'
                   b'Content-Length: ' + str(len(frame)).encode() + b'\\r\\n\\r\\n' +
                   frame + b'\\r\\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot.jpg')
def snapshot():
    start_camera_once()
    with output.condition:
        output.condition.wait(timeout=1.0)
        frame = output.frame
    return Response(frame, mimetype='image/jpeg')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, threaded=True)
PYEOF"
remote_sudo "mv /tmp/app.py ${APP_DIR}/app.py && chown -R ${USER}:${USER} ${APP_DIR}"

echo "Creating systemd service ${SERVICE_NAME} ..."
remote "cat > /tmp/${SERVICE_NAME} <<EOF
[Unit]
Description=Sharky Web Camera (Flask + Picamera2)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=${APP_DIR}
Environment=LIBCAMERA_LOG_LEVELS=*:3
ExecStart=/usr/bin/python3 ${APP_DIR}/app.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF"
remote_sudo "mv /tmp/${SERVICE_NAME} ${SERVICE_PATH} && chown root:root ${SERVICE_PATH} && chmod 644 ${SERVICE_PATH}"
remote_sudo "systemctl daemon-reload && systemctl enable --now ${SERVICE_NAME}"

echo "Web stream available on:"
echo "  http://localhost:8000/ (LAN or via hostname)"
echo "  http://10.42.0.1:8000/ (AP default via NetworkManager)"


