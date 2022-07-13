import cv2 
if __name__ =='__main__':
	img = cv2.imread('opencv_img.png')
	while True:
		cv2.imshow('frame',img)
		action = cv2.watiKey(1) & 0xFF
		if action == ord('q') or action ==113:
			break
	img.release()
	cv2.destroyAllWindows()