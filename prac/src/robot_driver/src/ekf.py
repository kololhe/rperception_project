
from geometry_msgs.msg import Point
import rospy
import numpy as np
from numpy.linalg import inv

global X, P, Rimu, Rcam, I, d, alpha, dt

X = np.matrix([[0],[1],[0]])
P = np.matrix([[0.01,0,0],[0,0.01,0],[0,0,0.01]])
Rimu = np.matrix([0.05])
Rcam = np.matrix([0.05])
I = np.eye(3)
d = 2.5
alpha = 0.0005363
dt = 1/250/2

def predict(x,P,dt):
        F = np.matrix([[1,dt,0],[0,1,0],[0,0,1]])
        #Q = np.matrix([[25*dt**4, 50*dt**3, 0],[50*dt**3, 100*dt**4, 0], [0,0,0]])
        Q = np.matrix([[0.005, 0.005, 0],[0.005, 0.005, 0], [0,0,0]])
        xkp = F@X
        Pkp = F @ P @ F.T + Q
        return xkp, Pkp
        print("predicted state")



class message_combiner:
    def __init__(self) :
        self.msg = Point(0, 0, 0)
        self.Pub = rospy.Publisher("/ekf_estimate", Point, queue_size=10)

    def run(self) :
        rospy.init_node("ekf", anonymous=True)

        rospy.Subscriber('/bot_imu', Point, self.imu)
        #rospy.Subscriber('/central_pixel', Point, self.cam)

        rospy.spin()

    def imu(self, msg):
        global X, P, dt, Rimu
        z = msg.x
        xpred,Ppred = predict(X,P,dt)
        H = np.matrix([0,0,1])
        residual = z-H@xpred
        res_cov = H@Ppred@H.T + Rimu
        rc = res_cov[0]
        rcinv = 1/rc
        K = Ppred@H.T
        K = K*rcinv
        X = xpred + K@residual
        P = (I - K@H)@Ppred
        

        self.msg.x = X[0]
        self.msg.y = X[1]
        self.msg.z = X[2]
        self.Pub.publish(self.msg) 
        print("got imu data")
        
    def cam(self, msg) :
        global X, P, dt, Rcam
        z = msg.x
        print("got camera data")       
    

if __name__ == '__main__' :

    mc = message_combiner()
    mc.run()
