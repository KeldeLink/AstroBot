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

def Step(x, steps_per_rev=200):
    # On lit la position réelle au moment T
    current = sensor.getValue()
    angle_a_ajouter = x * math.pi/180*DEG_PAR_PAS
    nouvelle_cible = current + angle_a_ajouter
    
    motor.setPosition(nouvelle_cible)
    print(f"Pas effectué. Nouvelle cible : {math.degrees(nouvelle_cible):.2f}°")

# Initialisation
robot.step(timestep)
motor.setVelocity(5)

while robot.step(timestep) != -1:
    # Faire 50 pas un par un vers l'avant
    print("Début séquence avant...")
    for i in range(9000):
        Step(1)
        wait_webots(0.5) # Remplace time.sleep(0.25)
        
    # Faire 50 pas un par un vers l'arrière
    print("Début séquence arrière...")
    for i in range(9000):
        Step(-1)
        wait_webots(0.5)