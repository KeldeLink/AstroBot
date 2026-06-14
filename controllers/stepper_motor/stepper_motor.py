from controller import Robot
import math
from numpy import sign

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor = robot.getDevice("azimuth motor")
sensor = robot.getDevice("azimuth sensor")
if sensor:
    sensor.enable(timestep)

# Petite fonction pour attendre proprement dans Webots
def wait_webots(ms):
    start_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime() - start_time >= ms / 1000.0:
            break
            
            
DEG_PAR_PAS=1.8 #0.01
current_ang=0

def Step(x, steps_per_rev=200):
    global current_ang
    current = current_ang # On met à jour la variable locale de l'angle actuel avec sa valeur globale
    nouvelle_cible = current + x * math.pi/180*DEG_PAR_PAS # Calcule le nouvel angle en fonction du degré par pas (résolution) du moteur pas à pas
    
    motor.setPosition(nouvelle_cible)
    #print("Valeur théorique : ", round(current*180/math.pi,5),"Valeur réelle : ", sensor.getValue()*180/math.pi)
    current_ang=nouvelle_cible # On met à jour la variable globale de l'angle actuel avec sa nouvelle valeur

    #print(f"Nouvelle cible : {math.degrees(nouvelle_cible):.2f}°")

def StepPosition(ang):
    az_pos_re=math.degrees(sensor.getValue())
    erreur=abs((az_pos_re-ang))
    while (erreur>0.5):
        az_pos_re=math.degrees(sensor.getValue())
        print(f"erreur : {erreur} position réelle : {az_pos_re}")
        Step(1*sign(az_pos_re-ang))
        erreur=abs((az_pos_re-ang))
        wait_webots(50) # Remplace time.sleep()
    
    
# Initialisation
robot.step(timestep)
motor.setVelocity(0.5) # Vitesse du changement de pas

StepPosition(45)
wait_webots(1000)
StepPosition(271.63)
