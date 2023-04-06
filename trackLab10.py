#!/usr/bin/python3
#                 trackLab10.py
#                 Recenter coordinates on LEDs
#                 Updates templates with each Normalized Correlation iteration
#                 Run motor and stop
import rospy
import time
from motor_control import Motor 
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
NT = 5                      # template offset
NS = 25                     # search size
print("type ", type(NT))
def distance(px1,px2):
    m = 2091.17
    b = -0.53
    delta = px2 - px1
    if delta > 2:
        return b + m/(px2 - px1)
    else:
        return -1.

def get_templ(img, NT, NS, Xcor, Ycor):
    Xc = np.rint(Xcor).astype(int)
    Yc = np.rint(Ycor).astype(int)
    Xc1 = Xc - NT
    Xc2 = Xc + NT + 1
    Yc1 = Yc - NT
    Yc2 = Yc + NT + 1
    Xs1 = Xc - NS
    Xs2 = Xc + NS + 1
    Ys1 = Yc - NS
    Ys2 = Yc + NS + 1
    templ = img[Yc1:Yc2,Xc1:Xc2]
    scrn  = img[Ys1:Ys2,Xs1:Xs2]
    return templ,scrn


class TakePhoto:
    def __init__(self):
        self.image_received = False

        img_topic = "/raspicam_node/image/compressed"
        img_topic = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback, queue_size = 10)

    def callback(self, data):
        self.image_received = True
        np_arr = np.frombuffer(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.img.astype(np.uint8)
    def get_img(self):
        if self.image_received == False:
           print("None")
           return
        return self.img
    def save_img(self, img_title):
        if self.image_received == False:
            print("None")
            return
        cv2.imwrite(img_title, self.img)
    def disp_img(self, img_title):
    	if self.image_received == False:
    	     print("None")
    	     return
    	cv2.imshow(img_title, self.img)

class CmdListener:
    def __init__(self):
        self.cmd = ""
        self.cmd_sub = rospy.Subscriber("/master_cmd", String, self.callback)
    def callback(self, cmd):
        self.cmd = cmd.data
    def get_msg(self):
        return self.cmd
    def setup_order(self):
        while self.get_msg() == "HIGH":
                pass
        print("High ")
        while self.get_msg() == "LOW":
                pass
        print("Low ")

if __name__ == "__main__":
    ipr= 0
    rospy.init_node("LED_Tracking")
    feedback_pub = rospy.Publisher("/feedback", String, queue_size = 1)
    camera = TakePhoto()
    cmd_listener = CmdListener()
    motor = Motor()
    rate = rospy.Rate(10)
    NX = 40                             # number of pixels to process, 30 works
    led_on = None
    led_off = None
    time.sleep(4)
    feedback_pub.publish(str(0))
    motor.stop()
    #cmd_listener.setup_order()
    print("Beginning cmd_listener ",cmd_listener.get_msg())
    while not rospy.is_shutdown():
      while ipr == 0:
        cmd_listener.setup_order()         #syncronize blinking
        if cmd_listener.get_msg() == "HIGH":
            time.sleep(0.45)
            led_on = camera.get_img()
            print("***HIGH***")
            while cmd_listener.get_msg() == "HIGH":
                pass
        if cmd_listener.get_msg() == "LOW":
            time.sleep(0.45)
            led_off = camera.get_img()
            print("***LOW***")
            while cmd_listener.get_msg() == "LOW":
                pass

        if (led_on is None or led_off is None):
            diff = np.zeros([240,320], dtype=np.int32)
            if(led_on == None):
                print("led_on ", led_on)
            if(led_off == None):
                print("led_off ", led_off)
            break
        else:
            diff = cv2.absdiff(led_on, led_off)
            diffB = 2*cv2.blur(diff, (3,3))    #Blured image
            mx_diffB = np.amax(diffB)
            cv2.imshow("Diff", diffB)
            idx = np.argmax(diffB, axis=0)		#x column max vector
            val = np.amax(diffB, axis=0)		#max value of image
            ival = np.argsort(val)[::-1]		#sort max column values, decending
            diffV = np.reshape(diffB, [320*240])
            AI = np.flip(np.argsort(diffB,axis=None))[0:NX] 
            AZ = np.zeros([NX,5], dtype=np.int32)     #pixel Value, linear Coordinate, X Coordinate, Y Coordinate, Cluster
            AZ[:,0] = diffV[AI]
            AZ[:,1] = AI
            AZ[:,2] = AI/320
            AZ[:,3] = AI%320
            AZ2i = np.argsort(AZ[:,3])
            AZZ = AZ[AZ2i,:]
            clus = 1
            pt0 = 0;
            for i in range(NX):
                 if( (AZZ[i,3] - AZZ[pt0,3]) < 2): #look for a  break in the x coordinates
                     AZZ[i,4] = clus 
                 else:                             #found a break in the x values (except final cluster)
                     clus += 1
                     AZZ[i,4] = clus
                 pt0 = i
            
            print(AZZ)
            
            led_on = 0
            led_off = 0
            
            i+=1
            # Calculate the X and Y average pixel values of each cluster
            #Clust values                cluster, X, Y, xmin, xmax, ymin, ymax
            Clust = np.zeros((clus,7), dtype=np.double)
            for i in range(clus):
               Clust[i,0] = i+1
               clus_ind = np.where(AZZ[:,4] == [i+1] )[0]
               print(i+1)
               print(clus_ind)
               Clust[i,1] = np.dot( AZZ[clus_ind,3],AZZ[clus_ind,0])/np.sum(AZZ[clus_ind,0])
               Clust[i,2] = np.dot( AZZ[clus_ind,2],AZZ[clus_ind,0])/np.sum(AZZ[clus_ind,0])
               Clust[i,3] = np.amin(AZZ[clus_ind,3])
               Clust[i,4] = np.amax(AZZ[clus_ind,3])
               Clust[i,5] = np.amin(AZZ[clus_ind,2])
               Clust[i,6] = np.amax(AZZ[clus_ind,2])
               print("X",Clust[i,1], " Xmin",Clust[i,3], "Xmax", Clust[i,4])
               print("Y",Clust[i,2], " Ymin",Clust[i,5], "Ymax", Clust[i,6])
               print()
            
            dist = distance(Clust[0,1],Clust[2,1])
            print("---------------------------------------------------------")
            print("Left Front LED Coordinate ", Clust[0,1],", ",Clust[0,2])
            print("Right Front LED Coordinate", Clust[2,1],", ",Clust[2,2])
            print("Rear LED Coordinate       ", Clust[1,1],", ",Clust[1,2])
            print("Distance ", dist)
            print("---------------------------------------------------------")
            
            if(clus == 3):
                feedback_pub.publish(str(1))       #turn off blinking
                cv2.waitKey(0)
                ipr = 1
                if dist < 0:
                     ipr = 0
                     print("distance < 0, Blinking Mode set")
                     feedback_pub.publish(str(0))
                     time.sleep(2)
                else:
                     cv2.waitKey(0)
                     print("Finished Blinking")
                     break   
            print("Wrong #1")
        rate.sleep()
        print("Wrong #2")
      else:                          # i > 0
        print("Not blinking")
        while camera.image_received == False:
                pass
        img_on = camera.get_img()
        if ipr == 1:
            print("ipr = ", ipr)
            templ1, scrn1 = get_templ(img_on, NT,NS, Clust[0,1], Clust[0,2])
            print("size templ1 ", templ1.shape)
            templ2, scrn2 = get_templ(img_on, NT,NS, Clust[2,1], Clust[2,2])
            templ3, scrn3 = get_templ(img_on, NT,NS, Clust[1,1], Clust[1,2])
        else:
            print("ipr = ", ipr)
            templX, scrn1 = get_templ(img_on, NT,NS, Clust[0,1], Clust[0,2])
            templX, scrn2 = get_templ(img_on, NT,NS, Clust[2,1], Clust[2,2])
            templX, scrn3 = get_templ(img_on, NT,NS, Clust[1,1], Clust[1,2])

        cv2.imshow("templ1", templ1)
        cv2.imshow("scrn1", scrn1)
        cv2.imshow("templ2", templ2)
        cv2.imshow("scrn2", scrn2)
        if ipr == 4:
             motor.move_with_coor(.07,0.)
        if ipr >= 4:
            if(dist < 24.):
                motor.move_with_coor(0., 0.)
                break
        thr = 0.85
        resulta2= cv2.matchTemplate(scrn1,  templ1, cv2.TM_CCOEFF_NORMED)
        (minVal, maxValaa, minLoc, maxLocaa2) = cv2.minMaxLoc(resulta2)
        print("* in screen area maxValaa, maxLocaa2 ", maxValaa, maxLocaa2)
        maxLocaa = (np.rint(Clust[0,1])-NS+ maxLocaa2[0], np.rint(Clust[0,2])-NS+maxLocaa2[1])

        resultb2 = cv2.matchTemplate(scrn2, templ2, cv2.TM_CCOEFF_NORMED)
        (minVal, maxValbb, minLoc, maxLocbb2) = cv2.minMaxLoc(resultb2)
        print("* in screen area maxValbb, maxLocbb2 ", maxValbb, maxLocbb2)
        maxLocbb = (np.rint(Clust[2,1])-NS+ maxLocbb2[0], np.rint(Clust[2,2])-NS+maxLocbb2[1])
        resultc2 = cv2.matchTemplate(scrn3, templ3, cv2.TM_CCOEFF_NORMED)
        (minVal, maxValcc, minLoc, maxLoccc2) = cv2.minMaxLoc(resultc2)
        print("* in screen area maxValcc, maxLoccc2 ", maxValcc, maxLoccc2)
        maxLoccc = (np.rint(Clust[1,1])-NS+ maxLoccc2[0], np.rint(Clust[1,2])-NS+maxLoccc2[1])


        print("* maxValaa, maxLocaa **",maxValaa, maxLocaa)
#        print("All values in resulta ", np.nonzero(resulta > thr))
        print("*******")
        print("** maxValbb, maxLocbb **",maxValbb, maxLocbb)
#        print("All values in resultb ", np.nonzero(resultb > thr))
        print("*******")
        print("*** maxValcc, maxLoccc **",maxValcc, maxLoccc)
#        print("All values in resultc ", np.nonzero(resultc > thr))
        distOLD = dist
        dist = distance(maxLocaa[0],maxLocbb[0])
        print("distance ", dist)
        print("***************************")
        if dist < 0:
             ipr = 0
             feedback_pub.publish(str(0))
             time.sleep(2)
             break
#       GENERATE NEW TEMPLATES BASED ON NEW LED COORDINATES
        Clust[0,1] = maxLocaa[0] + NT
        Clust[0,2] = maxLocaa[1] + NT
        Clust[2,1] = maxLocbb[0] + NT
        Clust[2,2] = maxLocbb[1] + NT
        Clust[1,1] = maxLoccc[0] + NT
        Clust[1,2] = maxLoccc[1] + NT

        templ1, scrn1 = get_templ(img_on, NT, NS, Clust[0,1], Clust[0,2])
        (minVal, maxVal, minLoc, maxLoca) = cv2.minMaxLoc(templ1)
        print("template A max Location ", maxLoca)
        Clust[0,1] += maxLoca[0] - NT
        Clust[0,2] += maxLoca[1] - NT
        templ1, scrn1 = get_templ(img_on, NT,NS, Clust[0,1], Clust[0,2])
        (minVal, maxVal, minLoc, maxLoca) = cv2.minMaxLoc(templ1)
        print("template A max Location ", maxLoca)

        templ2, scrn2 = get_templ(img_on, NT,NS, Clust[2,1], Clust[2,2])
        (minVal, maxVal, minLoc, maxLocb) = cv2.minMaxLoc(templ2)
        print("template B max Location ", maxLocb)
        Clust[2,1] += maxLocb[0] - NT
        Clust[2,2] += maxLocb[1] - NT
        templ2, scrn2 = get_templ(img_on, NT,NS, Clust[2,1], Clust[2,2])
        (minVal, maxVal, minLoc, maxLocb) = cv2.minMaxLoc(templ2)
        print("template B max Location ", maxLocb)

        templ3, scrn3 = get_templ(img_on, NT,NS, Clust[1,1], Clust[1,2])
        (minVal, maxVal, minLoc, maxLocc) = cv2.minMaxLoc(templ3)
        print("template C max Location ", maxLocc)
        Clust[1,1] += maxLocc[0] - NT
        Clust[1,2] += maxLocc[1] - NT
        templ3, scrn3 = get_templ(img_on, NT,NS, Clust[1,1], Clust[1,2])
        (minVal, maxVal, minLoc, maxLocc) = cv2.minMaxLoc(templ3)
        print("template C max Location ", maxLocc)


        ipr += 1
        if dist < 0:
             ipr = 0
        if abs(distOLD - dist) > 6.:
             ipr = 0			#stop and go back to blink
        cv2.imshow("Diff", diffB)
        if(ipr > 50):
            motor.stop()
            break
        time.sleep(.5)
