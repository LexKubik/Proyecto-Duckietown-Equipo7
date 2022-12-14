#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from duckietown_msgs.msg import Twist2DStamped #
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist, Point
from sensor_msgs.msg import Image, Joy #importar mensajes a ROS tipo Image y Joy
import cv2 #importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np #importar libreria numpy


class Proyecto(object):
	def __init__(self, args):
		super(Proyecto, self).__init__()
		self.args = args
		#Para la mascara, deteccion y calculo de posicion del semaforo
		self.subCam = rospy.Subscriber('/duckiebot/camera_node/image/rect', Image, self.deteccion)
		self.pubMask = rospy.Publisher("/duckiebot/mask", Image, queue_size=10)
		self.pubDetect = rospy.Publisher("/duckiebot/detecciones", Image, queue_size = 10)
		self.pubPosicion = rospy.Publisher("/duckiebot/Posicion", Point, queue_size = 10)
		self.bridge = CvBridge()
		self.minim = 1000
	
		#Para controlar el duckiebot (joystick)
		self.subJoy = rospy.Subscriber("/duckiebot/joy",Joy,self.joystick)
		self.pubJoy = rospy.Publisher("/duckiebot/possible_cmd", Twist2DStamped,queue_size=1)
		self.msg_joy = Twist2DStamped()

		#Para detencion por deteccion (controller)
		self.subPosicion = rospy.Subscriber('/duckiebot/Posicion', Point, self.detencion)
		#self.subCmd = rospy.Subscriber("/duckiebot/possible_cmd",Twist2DStamped, self.joy_aux)
		self.pubWheels = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd",Twist2DStamped,queue_size=1)
		self.msg_control = Twist2DStamped()
		
		#Contador
		self.count=3
#####

	def deteccion(self,msg): #deteccion del semaforo en rojo
		image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		image_out_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		#deteccion de color rojo
		lower_limit_1 = np.array([0, 100, 100])
		upper_limit_1 = np.array([10,255, 255])
		
		lower_limit_2 = np.array([160, 100, 100])
                upper_limit_2 = np.array([169,255, 255])
		
		#crear mascara para detectar
		
		#mask1 usa los limites 1 del color rojo
	#	mask1 = cv2.inRange(image_out_hsv, lower_limit_1, upper_limit_1) 
	#	kernel = np.ones((5,5),np.uint8)
	#	mask1 = cv2.erode(mask1, kernel, iterations=1)
	#	mask1 = cv2.dilate(mask1, kernel, iterations=1)
		
		#mask2 usa los limites 2 del color rojo
		mask2 = cv2.inRange(image_out_hsv, lower_limit_2, upper_limit_2)
                #kernel = np.ones((5,5),np.uint8)
                #mask2 = cv2.erode(mask2, kernel, iterations=1)
                #mask2 = cv2.dilate(mask2, kernel, iterations=1)

		#mask usando mask1 y mask2 ponderadas. para usar este metodo, descomentar lineas 54-57, 61-63 y 66, y comentar linea 69
		#mask=cv2.addWeighted(mask1,1.0,mask2,1.0,0.0)
		
		#mask usando mask2 y un GaussianBlur
		mask=cv2.GaussianBlur(mask2,(9,9),2,2)
               
		#encuentra los bordes de los objetos detectados
		_,contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		#aplica la mascara a la imagen
		image_out = cv2.bitwise_and(image, image, mask = mask)
		
		#distancia entre el Duckiebot y el objeto detectado
		distancia = Point()

                for cnt in contours:
			x,y,w,h=cv2.boundingRect(cnt)
			if w*h>0:
                		cv2.rectangle(image_out, (x,y), (x+w,y+h), (0,0,255), 2) #dibuja un rectangulo alrededor de cada objeto detectado
				Dr =( 3 * 101.859163)/ h
				
                		distancia.x = x
                		distancia.y = y
                		distancia.z = Dr
	
		#publica la distancia
		self.pubPosicion.publish(distancia)
		msg_mask  = self.bridge.cv2_to_imgmsg(image_out, "bgr8")
		
		#publica la mascara  	
		self.pubMask.publish(msg_mask)
		
	def joystick(self,msg):	#controla el Duckiebot con el joystick
		B=(msg.buttons)[1]	#freno de emergencia presionando el boton B del joystick
		if B==1:
			x = 0
			y = 0
		else:
			eje0=(msg.axes)[0]
			eje1=(msg.axes)[1]

			x=10*eje0
			y=eje1

		#asigna velocidad lineal y angular
		self.msg_joy.v = y
                self.msg_joy.omega = -x
		#publica velocidad que indica el joystick (o freno de emergencia)
                self.pubJoy.publish(self.msg_joy)				

	def detencion(self,msg): #detencion automatica al detectar semaforo en rojo
		if (msg.z<45 and msg.z>0):		#if detecta semaforo en rojo a distancia menor a 45 se detiene
			self.msg_control.v = 0
                        self.msg_control.omega = 0
			self.count=0
			#print("dentencion:",msg.z)

		elif msg.z==0 or msg.z>45:		#if no detecta o esta lejos, aumenta el contador			
			self.count=self.count+1
			#print("counter:",self.count)				
			if self.count>2:		#if el contador es mayor a 2, el duckiebot se mueve (hace lo que dice el joystick)
				self.msg_control = self.msg_joy
				#print("counter:",self.count)
   
			else:				#if el contador es menor a 2, el duckiebot se mantiene detenido (para evitar falsas no-detecciones)
				self.msg_control.v = 0
	                        self.msg_control.omega = 0
				#print("safe")
		
		#publica la velocidad que corresponda
		self.pubWheels.publish(self.msg_control)

def main():
	rospy.init_node('Nodo_proyecto') #creacion y registro del nodo!

	obj = Proyecto('args') # Crea un objeto del tipo Proyecto, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
