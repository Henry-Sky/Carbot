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
            ret,img_encode = cv2.imencode('.jpg', img)
            return ret,img_encode.tobytes(),codeinfo
        else:
            return False
    
# 二维码识别函数
def qrcheck(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    qrcoder = cv2.QRCodeDetector()
    codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(gray_img)
    return codeinfo
            
# 网页框架
app = Flask(__name__)

# 主页函数调用
@app.route('/') 
def index():
    return render_template('index.html')
 
def gen(camera):
    while True:
        flag,frame,codeinfo = camera.get_frame()
        if codeinfo != "":
            print("识别二维码:",codeinfo)
        if flag:
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        else:
            continue

# 视频页
@app.route('/video_feed')
def video_feed():
    return Response(gen(VideoCamera()), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run()