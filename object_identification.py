import cv2
import imutils
import numpy as np

def read_image_from_camera():
	

def image_processing(image):
	# convert black squares to white
	THRESH = 40
	idx = np.where((image <= [THRESH, THRESH, THRESH]).all(axis=2))
	thresh_image = image
	thresh_image[idx] = [255, 255, 255]
	
	# threshold image to make bgd black
	ret, thresh = cv2.threshold(thresh_image, 200, 250, type=cv2.THRESH_BINARY)
	thresh_image = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)

	# find contour with maximum area
	cont, _ = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	areas = [cv2.contourArea(c) for c in cont]

	max_area, max_cont = max(areas), cont[np.argmax(areas)]

	# # create bounding box for largest contour
	# x, y, w, h = cv2.boundingRect(max_cont)
	# cv2.imshow("Result", cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2))
	# cv2.waitKey(0)

	print(max_area)

if __name__ == '__main__':
	image = cv2.imread("/Users/dhruvlaad/IIT/ID6100/roslab/ref.jpg")
	experimental(image)