from flask import Flask, redirect, url_for, request,render_template,Response
import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
from cv2 import *
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

Motor_In1 = 29
Motor_In2 = 31
Motor_In3 = 33
Motor_In4 = 35
GPIO.setup(Motor_In1,GPIO.OUT)   # Set pin function as output
GPIO.setup(Motor_In2,GPIO.OUT)   # Set pin function as output
GPIO.setup(Motor_In3,GPIO.OUT)   # Set pin function as output
GPIO.setup(Motor_In4,GPIO.OUT)   # Set pin function as output

GPIO.output(Motor_In1, False)
GPIO.output(Motor_In2, False)
GPIO.output(Motor_In3, False)
GPIO.output(Motor_In4, False)

video_capture = cv2.VideoCapture(0)

Url_Address = "192.168.0.102"
app = Flask(__name__)


def generate_frames():
    while True:
        result, output = video_capture.read()
        cv2.imshow('frame', output)
        ret, buffer = cv2.imencode('.jpg', output)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
@app.route('/video_feed')
def video_feed():
    print ("Error: unable to fecth data")
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/Forward',methods = ['POST', 'GET'])
def Forward():
   try:
         print ("forward")      
         GPIO.output(Motor_In1, True)
         GPIO.output(Motor_In2, False)
         GPIO.output(Motor_In3, True)
         GPIO.output(Motor_In4, False)
         time.sleep(0.1)
         return render_template("temp.html",HTML_address=Url_Address)
   except:
      print ("Error: unable to fecth data")

@app.route('/Backward',methods = ['POST', 'GET'])
def Backward():
   try:
        print ("backward")  
        GPIO.output(Motor_In1, False)
        GPIO.output(Motor_In2, True)
        GPIO.output(Motor_In3, False)
        GPIO.output(Motor_In4, True)
        time.sleep(0.1)
        return render_template("temp.html",HTML_address=Url_Address)
   except:
      print ("Error: unable to fecth data")

@app.route('/left',methods = ['POST', 'GET'])
def left():
   try:
        print ("left")
        GPIO.output(Motor_In1, True)
        GPIO.output(Motor_In2, False)
        GPIO.output(Motor_In3, False)
        GPIO.output(Motor_In4, True)
        time.sleep(0.1)
        return render_template("temp.html",HTML_address=Url_Address)
   except:
      print ("Error: unable to fecth data")

@app.route('/right',methods = ['POST', 'GET'])
def right():
   try:
        print ("right")      
        GPIO.output(Motor_In1, False)
        GPIO.output(Motor_In2, True)
        GPIO.output(Motor_In3, True)
        GPIO.output(Motor_In4, False)
        time.sleep(0.1)
        return render_template("temp.html",HTML_address=Url_Address) 
   except:
      print ("Error: unable to fecth data")

@app.route('/stop',methods = ['POST', 'GET'])
def stop():
   try:
        print ("stop")     
        GPIO.output(Motor_In1, False)
        GPIO.output(Motor_In2, False)
        GPIO.output(Motor_In3, False)
        GPIO.output(Motor_In4, False)
        time.sleep(0.1)
        return render_template("temp.html",HTML_address=Url_Address)  
   except:
      print ("Error: unable to fecth data")

@app.route('/')
def login():
   return render_template("temp.html",HTML_address=Url_Address)      

      
if __name__ == '__main__':
   app.run(Url_Address,8080,threaded=True)



