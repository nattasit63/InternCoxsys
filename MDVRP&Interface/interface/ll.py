import cv2
  
originalImage = cv2.imread('/home/natta/interface_ws/src/full_interface/config/map_demo.pgm')
grayImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)
  
(thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
 
cv2.imshow('Black white image', blackAndWhiteImage)
cv2.imshow('Original image',originalImage)
# cv2.imshow('Gray image', grayImage)
  
cv2.waitKey(0)
cv2.destroyAllWindows()