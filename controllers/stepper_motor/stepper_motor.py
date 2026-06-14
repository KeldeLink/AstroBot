from controller import Robot
import math
from numpy import sign

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor_az = robot.getDevice("azimuth motor")
sensor_az = robot.getDevice("azimuth sensor")
motor_alt = robot.getDevice("altitude motor")
sensor_alt = robot.getDevice("altitude sensor")

for m in [motor_az, motor_alt]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

for s in [sensor_az, sensor_alt]:
    s.enable(timestep)

# Petite fonction pour attendre proprement dans Webots
def wait_webots(ms):
    start_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime() - start_time >= ms / 1000.0:
            break
            
            
DEG_PAR_PAS=0.9 #0.01
current_ang_az=0
current_ang_alt=0

def Step(x,motor,steps_per_rev=200):
    global current_ang_az
    current_ang=current_ang_az
    current = current_ang # On met à jour la variable locale de l'angle actuel avec sa valeur globale
    nouvelle_cible = current + x * math.pi/180*DEG_PAR_PAS # Calcule le nouvel angle en fonction du degré par pas (résolution) du moteur pas à pas
    
    motor.setPosition(nouvelle_cible)
    #print("Valeur théorique : ", round(current*180/math.pi,5),"Valeur réelle : ", sensor.getValue()*180/math.pi)
    current_ang=nouvelle_cible # On met à jour la variable globale de l'angle actuel avec sa nouvelle valeur

    #print(f"Nouvelle cible : {math.degrees(nouvelle_cible):.2f}°")

def StepPosition(capteur,motor, ang):
    pos_re=math.degrees(capteur.getValue())
    pos_re=pos_re*sign(pos_re-ang)
    erreur=abs((pos_re-ang))
    while (erreur>0.5):
        pos_re=math.degrees(capteur.getValue())
        pos_re=pos_re*sign(pos_re-ang)
        print(f"erreur : {abs((pos_re-ang))} position réelle : {pos_re}")
        erreur=abs((pos_re-ang))
        Step(1*sign(pos_re-ang), motor)
        wait_webots(5) # Remplace time.sleep()
    
    
# Initialisation
robot.step(timestep)
motor_az.setVelocity(0.5) # Vitesse du changement de pas

while robot.step(timestep) != -1:
    #StepPosition(sensor_az,motor_az,45)
    Step(50,motor_az)
    Step(50,motor_alt)
    wait_webots(2500)