#!/usr/bin/env python

import numpy as np
import cv2
import time


count = 1
# init camera
cap = cv2.VideoCapture(4)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

while(cap.isOpened()):
  # capture img
  ret, img = cap.read()
  cv2.imshow("img", img)
  # print(img.shape)

  key = cv2.waitKey(1)
  if key == 27:  # 按esc键退出
    print("ESC Break...")
    cv2.destroyAllWindows()
    break
  if key == ord(' '):   # 按空格键保存
    filename = '/home/zxc/Desktop/scripts/images/'+str(count)+'.png'
    cv2.imwrite(filename, img)
    print("Saved", count)
    count = count + 1

# release
cap.release()
