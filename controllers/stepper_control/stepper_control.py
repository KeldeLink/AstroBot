
import math
from numpy import sign


from controller import Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor_az = robot.getDevice("azimuth motor")
motor_alt = robot.getDevice("altitude motor")

# --- SUPPRESSION DES CAPTEURS ---
# Plus besoin de getDevice pour les capteurs ni de s.enable()

# Initialisation des moteurs
for m in [motor_az, motor_alt]:
    m.setPosition(0.0)
    m.setVelocity(0.0) 

# Fonction pour attendre proprement dans Webots
def wait_webots(ms):
    start_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime() - start_time >= ms / 1000.0:
            break

DEG_PAR_PAS = 0.9

# Dictionnaire pour stocker l'angle cible actuel de chaque moteur indépendamment
current_ang = {
    motor_az: 0.0,
    motor_alt: 0.0
}

def Step(x, motor):
    """Fait avancer le moteur donné d'un certain nombre de pas (x)."""
    global current_ang
    current = current_ang[motor]
    motor.setVelocity(100)
    
    # Calcule le nouvel angle en radians
    nouvelle_cible = current + x * (math.pi / 180.0) * DEG_PAR_PAS
    
    motor.setPosition(nouvelle_cible)
    current_ang[motor] = nouvelle_cible # Mise à jour locale pour ce moteur spécifique
    motor.setVelocity(0)
    
def StepPosition(motor, ang_cible):
    """Atteint l'angle cible en comptant strictement le nombre de pas nécessaires (boucle ouverte)."""
    motor.setVelocity(100)
    
    # 1. On récupère la position actuelle estimée en degrés
    pos_actuelle_deg = -math.degrees(current_ang[motor])
    erreur_deg = ang_cible - pos_actuelle_deg
    
    # 2. On calcule le nombre exact de pas entiers à effectuer
    nb_pas = int(round(erreur_deg / DEG_PAR_PAS))
    
    # Si aucun pas n'est nécessaire, on s'arrête
    if nb_pas == 0:
        print(f"Cible déjà atteinte ou trop proche. (Position : {pos_actuelle_deg:.2f}°)\n")
        motor.setVelocity(0.0)
        return

    print(f"Déplacement de {abs(nb_pas)} pas requis pour passer de {pos_actuelle_deg:.2f}° à {ang_cible}°")
    
    direction = -sign(nb_pas)
    
    # 3. On effectue les pas un par un pour garder l'effet visuel fluide
    for _ in range(abs(nb_pas)):
        Step(direction, motor)
        wait_webots(5)
        
    print(f"Cible atteinte ! Nouvelle position théorique : {-math.degrees(current_ang[motor]):.2f}°\n")
    motor.setVelocity(0.0)


# Initialisation
robot.step(timestep)
i = 0

motor_az.setVelocity(1.0)

# 2. On exécute l'action de préparation
Step(50, motor_az) 

# 3. ON ATTEND que le robot ait le temps de faire le mouvement 
#    avant que la boucle while ne commence
wait_webots(2000)


while robot.step(timestep) != -1:
    #print(f"--- Mouvement vers l'Azimut {i}° ---")
    
    # Plus besoin de passer le capteur en argument
    #StepPosition(motor_az, i)
    StepPosition(motor_az, 90)
    #Step(100,motor_alt) 
    i = i + 45
    wait_webots(1000)

 