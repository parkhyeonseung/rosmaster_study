import os
import qrcode
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt

def add_logo(img,logo_path):
    icon = Image.open(logo_path)
    img_w,img_h = img.size
    
    icon_w,icon_h = icon.size
    
    factor = 6
    size_w = int(img_w/factor)
    size_h = int(img_h/factor)
    
    if icon_w > size_w : icon_w = size_w
    if icon_h > size_h : icon_h = size_h
    
    icon = icon.resize((icon_w,icon_h),Image.ANTIALIAS)
    
    w = int((img_w - icon_w)/2)
    h = int((img_h - icon_h)/2)
    
    img.paste(icon, (w,h), mask=None)
    return img

def Create_QRcode(data, file_name, logo_path):
    my_file = Path(logo_path)
    qr = qrcode.QRCode(
        version = 1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=5,
        border=4,
    )
    
    qr.add_data(data)
    qr.make(fit=True)
    
    img = qr.make_image(fill_color="green", back_color="white")
    if my_file.is_file(): img = add_logo(img, logo_path)
    img.save(file_name)

    return img

if __name__ =="__main__":
    file_path = os.getcwd()
    logo_path = file_path+ "/opencv_img.png"
    out_img = file_path + "/qrcode.jpg"
    input = input("information enter:")
    Create_QRcode(input, out_img,logo_path)
    