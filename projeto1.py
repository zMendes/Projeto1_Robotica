#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import visao_module
import cormodule
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8



bridge = CvBridge()
w_180 =  math.pi/3
cv_image = None
media = []
centro = []
atraso = 1.5E9 
area = 0.0 
isCar =  False


check_delay = False 

def scanner(dado):
	global scann
	scann = dado.ranges


def scaneou(dado):
	global crash
	crash = dado.data


def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro


    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs

    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados =  visao_module.processa(cv_image)
        media, centro, area =  cormodule.identifica_cor(cv_image)
        for i in resultados:
            if i[0] == "car":
                global isCar
                isCar = True

        depois = time.clock()

        
    except CvBridgeError as e:
        print('ex', e)
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/kamera"
    


    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scanner)
    bump = rospy.Subscriber("/bumper", UInt8, scaneou)

    global scann
    scann = [2]
    global isCar
    isCar == False
    global crash
    crash = None

    try:

        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xFF

            vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0))        
            if len(media) != 0 : 
                if centro[1] - media[0] > 30:
                    vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0.5))
                elif centro[1] - media[0] < -30: 
                    vel = Twist(Vector3(0.07,0,0), Vector3(0,0,-0.5))
                else: 
                    vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0))
                media= []
    
            if isCar == True:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.6))
                velocidade_saida.publish(vel)
                isCar = False
            
            for i in range(len(scann)):
                if i <= 89:
                    if scann[i] != np.inf and scann[i]> 0.13 and  scann[i]< 0.25:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-w_180))

                if i >= 270:
                    if scann[i] != np.inf and scann[i]> 0.13 and  scann[i]< 0.25:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,w_180))    

                
            if crash ==1 or crash ==2:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                rospy.sleep(1.0)
                vel = Twist(Vector3(-0.07,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                rospy.sleep(3.0)
                if crash ==1:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
                    velocidade_saida.publish(vel)
                    rospy.sleep(2.0)
                elif crash ==2:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
                    velocidade_saida.publish(vel)
                    rospy.sleep(2.0)
                    
                crash = None

            if crash ==3 or crash ==4:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                rospy.sleep(1.0)
                vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                rospy.sleep(3.0)
                if crash ==3:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
                    velocidade_saida.publish(vel)
                    rospy.sleep(2.0)
                elif crash ==4:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
                    velocidade_saida.publish(vel)
                    rospy.sleep(2.0)
                    
                crash = None

            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


