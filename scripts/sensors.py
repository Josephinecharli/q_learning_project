import keras_ocr
import cv2

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Sensing(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("Move_Robot")

        # download pre-trained model
        self.pipeline = keras_ocr.pipeline.Pipeline()

        # Once you have the pipeline, you can use it to recognize characters,
        # images is a list of images in the cv2 format
        self.images = [img1, img2, ...]

        # call the recognizer on the list of images
        prediction_groups = pipline.recognize(images)

        # prediction_groups is a list of predictions for each image
        # prediction_groups[0] is a list of tuples for recognized characters for img1
        # the tuples are of the formate (word, box), where word is the word
        # recognized by the recognizer and box is a rectangle in the image where the recognized words reside


        #Functionality:
        # need a run function
        

        # pick dumbell by color



    def run(self):
        continue


if __name__ == "__main__":
    node = Move_Robot()
    rospy.spin()
