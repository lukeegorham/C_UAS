# import the opencv library
import cv2


# define a video capture object
vid = cv2.VideoCapture('/dev/video2')
# NOTE:
#     the following is true on Raspberry Pi!
#     video2 is black/white with lots of dots??? - uses second camera from the right
#     video4 is color - uses rightmost camera
#     No other video works
#     On NVIDIA video2 is color camera

while(True):
    
    # Capture the video frame
    # by frame
    ret, frame = vid.read()

    # Display the resulting frame
    if not ret:
        continue
    
    cv2.imshow('frame', frame)
    
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
