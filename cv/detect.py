import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import keras


#model = keras.models.load_model('convNN.model')
#print(model)
IMG_SIZE = 80

class DetectCrocodile:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

    def image_callback(self, image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "mono8")
            cv2.imshow("Camera", self.cv_image)
            #self.crocodile()
        except CvBridgeError as e:
            print (e)

        cv2.waitKey(3)

    # def crocodile(self):
    #     image = cv2.resize(self.cv_image, (IMG_SIZE, IMG_SIZE))
    #     blur = cv2.GaussianBlur(image,(3,3),0)
    #     ready_to_predict = np.array(blur).reshape(-1, IMG_SIZE, IMG_SIZE, 1)
    #     ready_to_predict = ready_to_predict/255.0
    #     prediction = model.predict(ready_to_predict)
    #     #Hello, Bayes
    #     print(prediction)
    #     if prediction[0][0] > 0.5:
    #         print("CROCODILE")
    #     else:
    #         print("NOT A CROCODILE")

# run with KERAS_BACKEND=theano python detect.py
def main():
    detecter = DetectCrocodile()
    rospy.init_node('DetectCrocodile', anonymous=True)
    rate = rospy.Rate(20)
    while True:
        #print('Entrou no while')
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting Down')
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

#Step 1 -

#Step 2 - clear image with some source of absolute_import and convert to hsf

#Step 3 - Define filters: color, brightness, lines, edges, whatever

#Step 4 - Fit an square around the crocodile

#Step 5 - Cut square image

#Step 6 - Apply neural network to evaluate probability that it is an actual crocodile
