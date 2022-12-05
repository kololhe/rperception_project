
from detection_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Point
import rospy

class pixel_extractor:
    def __init__(self) :
        self.msg = Point(0,0,0)
        self.Pub = rospy.Publisher('/central_pixel', Point, queue_size=10)

    def run(self) :
        rospy.init_node('message_combiner_node')

        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.center_pixel)

        rospy.spin()

    def center_pixel(self, msg):
        xmin = msg.bounding_boxes[0].xmin
        xmax = msg.bounding_boxes[0].xmax
        central_pixel = -((xmin+xmax)/2 - 1980/2)
        print(central_pixel)
        self.msg.x = central_pixel
        self.Pub.publish(self.msg) 
        
if __name__ == '__main__' :

    mc = pixel_extractor()
    mc.run()
