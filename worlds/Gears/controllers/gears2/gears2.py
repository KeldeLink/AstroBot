from controller import Robot
import math

# Initialisation du robot
robot = Robot()

# Définition du pas de temps (TIME_STEP)
time_step = 8

# Récupération des dispositifs (Devices)
motor = robot.getDevice("rotational motor")
sensor = robot.getDevice("position sensor")
sensor2 = robot.getDevice("position sensor2")

# Activation des capteurs
sensor.enable(time_step)
sensor2.enable(time_step)

# Configuration initiale du moteur
# En Python, float('inf') correspond à INFINITY en C
motor.setPosition(float('inf'))
motor.setVelocity(0.2)

print("--- Démarrage du contrôleur Python ---")

# Boucle principale (wb_robot_step)
while robot.step(time_step) != -1:
    
    # Lecture de la valeur du capteur 2
    # Note : pos2 sera en radians par défaut dans Webots
    pos=sensor.getValue()
    pos2 = sensor2.getValue()
    pos2=pos2*180/math.pi
    pos=pos*180/math.pi
    # Affichage formaté (identique au printf du C)
    # %8.4f : 8 caractères de large, 4 chiffres après la virgule
    print(f"Angle rouge : {pos:8.2f} deg | Angle bleu : {pos2:8.2f} deg")
    
    nbtour1=pos/360
    nbtour2=pos2/360
    print(f"Nombre tour rouge : {nbtour1:8.2f} tour | Nombre tour bleu : {nbtour2:8.2f} tour")
    
    
    # Mise à jour de la vitesse dans la boucle comme dans ton code C
    motor.setVelocity(0.8)

# Pas besoin de wb_robot_cleanup() explicite en Python, 
# la destruction de l'objet robot s'en charge.