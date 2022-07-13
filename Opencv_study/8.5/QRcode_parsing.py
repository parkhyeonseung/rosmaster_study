import time
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from PIL import Image, ImageDraw, ImageFont

def decodeDisplay(image, font_path):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    barcodes = pyzbar.decode(gray)
    
    for barcode in barcodes:
        (x,y,w,h) = barcode.rect
        cv2.rectangle(image, (x,y),(x+w,y+h),(255,0,0),5)
        encoding = 'UTF-8'
        
        barcodeData = barcode.data.decode(encoding)
        barcodeType = barcode.type
        
        pillimg = Image.fromarray(image)
        draw = ImageDraw.Draw(pillimg)
        
        # fontStyle = ImageFont.truetype(font_path, size=12, encoding=encoding)
        fontStyle = ImageFont.load_default()
        draw.text((x,y -25), str(barcode.data, encoding),fill=(255,0,0),font=fontStyle)
        
        image = cv2.cvtColor(np.array(pillimg),cv2.COLOR_RGB2BGR)
        
        print("[INFO] found {} barcode: {}".format(barcodeType,barcodeData))
    return image

if __name__ =="__main__":
    font_path = "../font/Block_Simplified.TTF"
    capture = cv2.VideoCapture(0)
    cv_edition = cv2.__version__
    if cv_edition[0] == '3': capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print('capture get FPS : ', capture.get(cv2.CAP_PROP_FPS))
    
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv2.waitKey(10) & 0xFF
        frame = decodeDisplay(frame, font_path)
        end = time.time()
        fps = 1 / (end-start)
        text = "FPS : " + str(int(fps))
        cv2.putText(frame, text, (30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.6,(100,200,200),1)
        cv2.imshow('frame',frame)
        if action == ord('q') or action == 113: break
    
    capture.release()
    cv2.destroyAllWindows()