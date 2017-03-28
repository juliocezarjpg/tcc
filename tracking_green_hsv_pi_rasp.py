# -*- coding: cp1252 -*-

# Importar as bibliotecas necessarias
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import socket
import serial

ser = serial.Serial("/dev/ttyAMA0", 9600)

ip = '10.0.55.11'
porta = 7013
addr = (ip, porta)

udp = socket.socket(2,2)

green = (0, 255, 0)
red = (0, 0, 255)

res = (320, 240)


a_read = open('values.txt', 'r')
texto = a_read.readline()
a_read.close()

#print (texto)

n1 = ''
values = []
for i in range(len(texto)):
	if texto[i] != ',':
		n1 += texto[i]
	else:
		n1 = int(n1)
		values.append(n1)
		n1 = ''



def nothing(x):
	pass

cv2.namedWindow('values', cv2.WINDOW_NORMAL)

cv2.createTrackbar('LowHue', 'values', values[0], 255, nothing)
cv2.createTrackbar('LowSat', 'values', values[1], 255, nothing)
cv2.createTrackbar('LowValue','values', values[2], 255, nothing)
cv2.createTrackbar('HighHue', 'values', values[3], 255, nothing)
cv2.createTrackbar('HighSat', 'values', values[4], 255, nothing)
cv2.createTrackbar('HighValue', 'values', values[5], 255, nothing)



# Inicializa a raspicam
camera = PiCamera()
camera.hflip = True
#camera.vflip = True
camera.resolution = res
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=res)

# Tempo para a camera ligar
time.sleep(0.1)

# Captura de frames
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	frame = frame.array
	

	LowHue = cv2.getTrackbarPos('LowHue', 'values')
	LowSat = cv2.getTrackbarPos('LowSat', 'values')
	LowValue = cv2.getTrackbarPos('LowValue', 'values')
	HighHue = cv2.getTrackbarPos('HighHue', 'values')
	HighSat = cv2.getTrackbarPos('HighSat', 'values')
	HighValue = cv2.getTrackbarPos('HighValue', 'values')
	

	cv2.namedWindow("Rgb",  cv2.WINDOW_NORMAL)
	cv2.imshow("Rgb", frame)	
	

	lower_collor = np.array([LowHue, LowSat, LowValue], dtype = 'uint8')
	upper_collor = np.array([HighHue, HighSat, HighValue], dtype = 'uint8')


 	#Transforma a imagem para HSV
    	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    	#Faz o thresold com a cor escolhida
    	threshold = cv2.inRange(hsv, lower_collor, upper_collor)

    	#Borra a imagem para tirar muitos detalhes desnecessarios
    	threshold = cv2.GaussianBlur(threshold, (3,3), 0)

    	#Cria kernel para usar na erosao e dilatacai
    	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    	#remove os ruídos da imagem usando a erosao
    	threshold = cv2.erode (threshold, kernel, iterations = 5)

    	#agrupa a imagem usando dilatacao
    	threshold = cv2.dilate(threshold, kernel, iterations = 2)
    
    	#Procurando contornos
    	(_, cnts, _) = cv2.findContours(threshold.copy(), mode = cv2.RETR_EXTERNAL,
                                        method = cv2.CHAIN_APPROX_SIMPLE)

    	#Contornos aproximados
    	(_,contours, _) = cv2.findContours(threshold.copy(),cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    	#Se tiver algum contorno
    	if len (cnts) > 0:

        	#Maior area
        	cnt = sorted (cnts, key = cv2.contourArea, reverse = True)[0]

        	contour = sorted(contours, key = cv2.contourArea, reverse = True)[0]

        	moments = cv2.moments(cnt)

        	#area do objeto
        	area = moments['m00']

        	#centroide do objeto
        	if moments['m00'] != 0:
            		cx = int(moments['m10']/moments['m00'])
            		cy = int(moments['m01']/moments['m00'])

        	if area > 200:       

            		#Lista de pontos para contorno
            		rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))

            		cv2.line(frame, (cx, cy), (cx, 240), green, 2)
            		cv2.line(frame, (0, cy), (cx, cy), green, 2)            		

            		font = cv2.FONT_HERSHEY_SIMPLEX
            		cv2.putText(frame, str(cx), (cx, 240), font, 1, 
				   (255, 255, 255), 2, cv2.LINE_AA)
            		cv2.putText(frame, str(240 - cy), (0, cy), font, 1, 
                                   (255, 255, 255), 2, cv2.LINE_AA)

			#Envia dados para o arduino
			ser.write(str(240-cy))
			ser.write('*')	

            		#Desenhando contornos aproximados            
            		cv2.drawContours(frame,contours,-1,(0, 0, 255),2)
            
            		hull = cv2.convexHull(contour)
            		hull_f = cv2.convexHull(contour, returnPoints = False)

            		defects = cv2.convexityDefects(contour, hull_f)

            		maior = 0
            		media = 0
            
		    	for i in range (defects.shape[0]):
		        	s, e, f, d = defects[i, 0]
		        	start = tuple(contour[s][0])
		        	end = tuple(contour[e][0])
		        	far = tuple(contour[f][0])

                		cv2.line(frame, start, end, [255, 0, 0], 2)
                		cv2.circle(frame, far, 5, [255, 0, 0], -1)

                		media = media + d
                		if d > maior:
                    			maior = d

            		#media = media - maior
            		media = media/(defects.shape[0])
			#print media

            		if (media < 5000):
				ser.write("(")
                	#	print 'fechado'

            		else:
				ser.write(")")
			#	print 'aberto'
			#if (cx > 400):
                	#	frame[80:130,10:50] = (0, 255, 0)
            		#if (cx < 200):
                	#	frame[150:200,10:50] = (255, 0, 0)

	img_str = cv2.imencode('.jpg', frame)[1]

	string = img_str.tostring()
	
	udp.sendto(string, addr)

	cv2.namedWindow("Frame",  cv2.WINDOW_NORMAL)
	cv2.namedWindow("Threshold",  cv2.WINDOW_NORMAL)
	cv2.namedWindow("Hsv",  cv2.WINDOW_NORMAL)
	cv2.imshow('Frame', frame)
    	cv2.imshow('Threshold', threshold)
	cv2.imshow("Hsv", hsv)	

	rawCapture.truncate(0)

	# se a letra 'q' for pressionada, encerra o programa
	key = cv2.waitKey(5) & 0xFF
	if key == 27:
		udp.sendto('q', addr)
		udp.close()
		
		values = ''
		values += (str(LowHue))
		values += (',')
		values += (str(LowSat))
		values += (',')
		values += (str(LowValue))
		values += (',')
		values += (str(HighHue))
		values += (',')
		values += (str(HighSat))
		values += (',')
		values += (str(HighValue))
		values += (',')

		a_write = open ('values.txt', 'w')
		a_write.write(values)
		a_write.close()
		
		break
		
