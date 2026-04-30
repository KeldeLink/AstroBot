from controller import Robot
import math

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
            
            
DEG_PAR_PAS=0.01
current_ang=0

def Step(x, steps_per_rev=200):
    global current_ang
    current = current_ang # On met à jour la variable locale de l'angle actuel avec sa valeur globale
    nouvelle_cible = current + x * math.pi/180*DEG_PAR_PAS # Calcule le nouvel angle en fonction du degré par pas (résolution) du moteur pas à pas
    
    motor.setPosition(nouvelle_cible)
    print("Valeur théorique : ", round(current*180/math.pi,5),"Valeur réelle : ", sensor.getValue()*180/math.pi)
    current_ang=nouvelle_cible # On met à jour la variable globale de l'angle actuel avec sa nouvelle valeur

    print(f"Pas effectué. Nouvelle cible : {math.degrees(nouvelle_cible):.2f}°")

# Initialisation
robot.step(timestep)
motor.setVelocity(0.5)

while robot.step(timestep) != -1:
    # Faire 50 pas un par un vers l'avant
    print("Début séquence avant...")
    for i in range(9000):
        Step(1)
        wait_webots(500) # Remplace time.sleep(0.25)
        
    