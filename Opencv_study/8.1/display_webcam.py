import cv2
if __name__ == '__main__':
    frame = cv2.VideoCapture(0)
    while frame.isOpened():
        ret,img = frame.read()
        cv2.imshow('frame',img)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    frame.release()
    cv2.destroyAllWindows()