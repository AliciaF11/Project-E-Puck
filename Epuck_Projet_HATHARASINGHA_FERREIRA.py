"""

Webots R2021b
Python 3.8

Prénom, Nom: Nadeesha HATHARASINGHA, Alicia FERREIRA


Date: 25/10/2021

Contrôle d'un robot Epuck
    - Controle manuel par clavier
    - Evitement d'obstacle
    - Odométrie
    - Recherche de la balle verte
    - Se diriger et s'arrêter devant la balle



"""


# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import numpy as np
import struct
from tkinter import *

#from controller import 
from controller import Robot, DistanceSensor, Motor, Camera, Keyboard, Emitter


# On crée une instance du robot
robot = Robot()
keyboard=Keyboard()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



# Definition max speed 
MAX_SPEED = 6.28

PI= 3.14159265358979

WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
RANGE = (1024 / 2)


sensors_value=[1,1,1,1,1,1,1,1]  
speed=[0,0]

# initialize devices
print('---Starting initialisation device-------------')

print('---------Initialisation Leds------------------')
leds=[]
ledsNames = ['led0','led1','led2','led3','led4','led5','led6','led7','led8','led9']
for i in range(10):
    leds.append(robot.getDevice(ledsNames[i])) 


print('---Initialisation Proximity Sensors ps--------')
ps = []
psNames = ['ps0', 'ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)


print('----Initialisation Light Sensors ls-----------')
ls = []
lsNames = ['ls0', 'ls1', 'ls2','ls3','ls4','ls5','ls6','ls7']
for i in range(8):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(timestep)

print('----Initialisation Motors---------------------')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

print('----Initialisation Position Sensor---------------------') #decodeur
#'left wheel sensor' and 'right wheel sensor'
leftcodeur=robot.getDevice('left wheel sensor')
rightcodeur=robot.getDevice('right wheel sensor')
leftcodeur.enable(timestep)
rightcodeur.enable(timestep)
print(leftcodeur.getType())
print(rightcodeur.getType())
right_codeur=rightcodeur.getValue()
left_codeur=leftcodeur.getValue()
distance_parcourue=0

print('----Initialisation Camera---------------------')
camera=robot.getDevice('camera')
camera.enable(timestep)

print('----Initialisation GPS ---------------------')
gps=robot.getDevice('gps')
gps.enable(timestep)

print('----Initialisation compass ---------------------')
compass=robot.getDevice('compass')
compass.enable(timestep)


print('----Initialisation Keyboard---------------------')
keyboard.enable(timestep)

print('----Initialisation Etapes---------------------')
etape_strategie_controle=0
etape_recherche_balle=0
balle_trouvee=0
balle_detectee=0

   
print('--------- Initialisation finished-------------')

print('Vous êtes en mode manuel')

#-----------------------------fin partie init------------------------

useManual = False
useMode="Manual"


#Plusieurs modes d'intéraction
while robot.step(timestep) != -1:
    
    # Lecture clavier 
    currentKey = keyboard.getKey()   
    
    # Actualisation du mode de fonctionnement
    if currentKey == ord('m') or currentKey == ord('M'):
        useMode="Manual"
        print('Passage en mode manuel')
    if currentKey == ord('a') or currentKey == ord('A'):
        useMode="Auto"  
        print('Passage en mode automatique')  
  
    # Lecture et mise à jour des données des capteurs de proximité
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    # detection obstacles gauche / droite
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0    
       
    #récupération de l'image et des dimensions de l'image
    image = camera.getImageArray()   
    largeurImage = int(camera.getWidth())        
    hauteurImage = int(camera.getHeight())
    
    # création de tableaux contenant les valeurs des pixels (RGB)
    pixel_red = np.zeros((largeurImage,hauteurImage))
    pixel_green = np.zeros((largeurImage,hauteurImage))
    pixel_blue = np.zeros((largeurImage,hauteurImage))
    pixel_l = np.zeros((largeurImage, hauteurImage))
    pixel_teta = np.zeros((largeurImage, hauteurImage))
    pixel_phi = np.zeros((largeurImage, hauteurImage))
    pixels_verts = np.zeros(largeurImage)
    balle_reperee = False
    
   # On récupère RGB, on les mesure puis on calcule theta et phi et on les utilise pour détecter la balle
    for x in range(largeurImage):
        for y in range(hauteurImage):
            pixel_red[x][y] = image[x][y][0]
            pixel_green[x][y] = image[x][y][1]
            pixel_blue[x][y] = image[x][y][2]
            pixel_l[x][y] = math.sqrt(pixel_red[x][y]**2 + pixel_green[x][y]**2 + pixel_blue[x][y]**2)
            pixel_teta[x][y] = math.atan(pixel_red[x][y]/pixel_green[x][y])
            pixel_phi[x][y] = math.acos(pixel_blue[x][y]/pixel_l[x][y])
            
   # On compte les pixels et on détermine la colonne où il y a le plus de pixels verts
            if pixel_teta[x][y] < 0.7 and pixel_phi[x][y]>1.2:
                balle_reperee = True
                pixels_verts[x] += 1
                
    # Indice max (indice avec le + de pixels verts) permet de s'orienter si =2 on tourne à droite et =52 tourne à gauche
    index_max = pixels_verts.argmax()
    average = pixels_verts.mean()
    print("average=",average)
   
  
  
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.0 * MAX_SPEED
    rightSpeed = 0.0 * MAX_SPEED
    
    # Mode manuel
    if useMode=="Manual":
        if currentKey == keyboard.UP:
            leftSpeed  = 0.7 * MAX_SPEED
            rightSpeed = 0.7 * MAX_SPEED
        
        if currentKey == keyboard.DOWN:
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
        if currentKey == keyboard.LEFT:
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED 
        if currentKey == keyboard.RIGHT:
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED 
    
    
    # A compléter ================================================
 
    if currentKey == ord('l') or currentKey == ord('L'):
         for i in range(10):
             leds[i].set(1)
             print('Les LED seteignent')
    if currentKey == ord('k') or currentKey == ord('K'):
         for i in range(10):
             leds[i].set(0)
             print('Les LED sallument') 
     
     
    # ===========================================================  
   
    #Mode auto       
    if useMode=="Auto": 
        
    #Stratégie globale: 3 etapes
        #On clique sur s pour que le robot commence à avancer et effecue sa trajectoire
        if etape_strategie_controle==0 and (currentKey== ord("s") or currentKey== ord("S")):
            etape_strategie_controle=1
            
       #Le robot a détecté la balle et va avancer en direction de la balle
        if etape_strategie_controle==1 and balle_reperee:
            etape_strategie_controle=2
            
       #Le robot a trouvé la balle: il allume ses LED et donne sa position
        if etape_strategie_controle==2 and balle_trouvee:
            etape_strategie_controle=3
  
                    
        if etape_strategie_controle==0:
        #mettre vitesses à 0
            leftSpeed  = 0.0 * MAX_SPEED
            rightSpeed = 0.0 * MAX_SPEED
           
        
        #Recherche de la balle    
        if etape_strategie_controle==1:
            #gestion séquentielle des étapes

            if etape_recherche_balle==0: 
                leftSpeed  = 0.75 * MAX_SPEED
                rightSpeed = 0.75 * MAX_SPEED 
                print("Etape0= OK")
                if gps.getValues()[2]>=-0.18 and gps.getValues()[2]<=-0.15:                 
                    etape_recherche_balle+=1
                    print("Go Etape1")
                  
            if etape_recherche_balle==1:
                leftSpeed  = 0.75 * MAX_SPEED
                rightSpeed = -0.75 * MAX_SPEED
                print("Etape1= OK")
 
                if angle_rotation<=-4.82 and angle_rotation>=-5.02:
                    etape_recherche_balle+=1

                 
            if etape_recherche_balle==2:
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED 
                print("Etape2= OK")
                
                if gps.getValues()[0]<=-0.30 and gps.getValues()[0]>=-0.35 :
                    etape_recherche_balle+=1
                
            if etape_recherche_balle==3:
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED 
                print("Etape3= OK")
                
                if angle_rotation<=-6.82 and angle_rotation>=-7.02:
                    etape_recherche_balle+=1
                    
                    
            if etape_recherche_balle==4: 
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED 
                print("Etape4= OK")
                if gps.getValues()[2]>=0.90 and gps.getValues()[0]<=0.95 : 
                    etape_recherche_balle+=1                
                
  
            if etape_recherche_balle==5:
                leftSpeed  = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED 
                print("Etape5= OK")               
                if angle_rotation>=-5.22 and angle_rotation>=-5.42:
                    etape_recherche_balle+=1                
                 
                 
            if etape_recherche_balle==6:
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED 
                print("Etape6= OK")
                if gps.getValues()[0]>=0.85 and gps.getValues()[0]<=0.90 : 
                    etape_recherche_balle+=1                
                            
                
            if etape_recherche_balle==7:
                leftSpeed  = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED  
                print("Etape7= OK")

                if angle_rotation<=-3.14 and angle_rotation>=-3.18:
                    etape_recherche_balle+=1 
                         
            if etape_recherche_balle==8:
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED 
                print("Etape8= OK")
                if gps.getValues()[2]>=0.88 and gps.getValues()[2]<=0.90 : 
                    etape_recherche_balle+=1                
                            
                
            if etape_recherche_balle==9:
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED  
                print("Etape9= OK")

                if angle_rotation<=-1.03 and angle_rotation>=-1.22:
                    print("Environnement exploré")
                   
    
                            
                
        if etape_strategie_controle==2:
        #gérer posititionnement du robot une fois que la balle est repérée
        #Le robot s'arrete 

           #Se diriger et suivre la balle
            if index_max<20:#gauche 
                leftSpeed  = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED 
                    
            elif index_max>31: #droite
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED
                
            else:#tout droit
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                
            if average>20:
                balle_trouvee=1   
                  
        if etape_strategie_controle==3:

            #On allume les led
                for i in range(10):
                     leds[i].set(1)
                     print('Les LED sallument') 
   
             #On affiche la position
                print("Le robot se situe aux coordonnées: x={0:0.2f}, y={1:0.2f}, z={2:0.2f} ".format(gps_values[0],gps_values[1],gps_values[2] ))
    

            
 
        
                
        
        # Evitement obstacle
        # A compléter ================================================
        if right_obstacle:
            leftSpeed = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        if left_obstacle:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            
        # =============================================================        
                
   #Applique les consignes au moteur
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
      
    gps_values=gps.getValues()   
    compass_values=compass.getValues()
    dr=rightcodeur.getValue()*WHEEL_RADIUS #Nombre d'impulsion * 2pi
    dl=leftcodeur.getValue()*WHEEL_RADIUS
    
    # A compléter ================================================
    distance_parcourue= (dr + dl)/2
    angle_rotation= (dr - dl)/ AXLE_LENGTH
    # =============================================================
   
    print("=================================================================================")
    print( 'Time='+str(robot.getTime()) )
    print("x={0:0.2f}, y={1:0.2f}, z={2:0.2f} ".format(gps_values[0],gps_values[1],gps_values[2] ))
    print("angle_x={0:0.2f},angle_y={1:0.2f},angle_z={2:0.2f} ".format(compass_values[0],compass_values[1],compass_values[2] ))
    print("Distance parcourue:{0:0.2f} ".format(distance_parcourue))
    print("Angle de rotation:{0:0.2f} ".format(angle_rotation))
   
    if balle_reperee:
        print("balle repérée: i= ",index_max)
        print("pixel_r= ;",pixel_red[index_max][20],"pixel_g= ;",pixel_green[index_max][20],"pixel_b= ;",pixel_blue[index_max][20])
        print("pixel_l={0:0.2f}; pixel_teta={1:0.2f}; pixel_teta={2:0.2f}".format(pixel_l[index_max][20],pixel_teta[index_max][20],pixel_phi[index_max][20]))
   
     
    pass

# Enter here exit cleanup code.