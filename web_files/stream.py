import cv2
import imutils

# 即時預覽Camera
def take_stream():
  #cap = cv2.VideoCapture(1)
  cap = cv2.VideoCapture(0)

  #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
  #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

  while True:
      ret, frame = cap.read()
      frame = imutils.resize(frame, 320)
      #cv2.putText(frame, "press 'q' to quit ", (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
      cv2.imshow("preview", frame)
      if cv2.waitKey(1) & 0xFF == ord("q"):
          break

  cap.release()
  cv2.destroyAllWindows()

take_stream()
