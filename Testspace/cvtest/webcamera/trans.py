from flask import Flask, render_template, Response
import cv2

# 调用摄像头类
class VideoCamera(object):
    def __init__(self):
        self.cap = cv2.VideoCapture(0) 
    
    def __del__(self):
        self.cap.release()
    
    def get_frame(self):
        flag, img = self.cap.read()
        if flag:
            return img

def gen(camera):
    while True:
        frame = camera.get_frame()
        ret, buffer = cv2.imencode('.jpg',frame)
        frame = buffer.tobytes()  
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

            
# 网页框架
app = Flask(__name__)

# 主页函数调用
@app.route('/') 
def index():
    return render_template('index.html')

# 视频页
@app.route('/video_feed')
def video_feed():
    return Response(gen(VideoCamera()), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run()