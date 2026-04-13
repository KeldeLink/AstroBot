from controller import Robot
import math

# ==========================================
# CONFIGURATION PID
# ==========================================
Kp = 3.5   # Force de rappel (Proportionnel)
Ki = 0.02  # Correction de l'erreur statique (Intégral)
Kd = 0.15  # Amortissement des secousses (Dérivé)

# Sens de rotation : Change 1 par -1 si le robot s'éloigne du Nord
SENS_AZ = 1 

# ==========================================
# INITIALISATION
# ==========================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

az_motor = robot.getDevice("azimuth motor")
imu = robot.getDevice("inertial unit")

az_motor.setPosition(float('inf'))
az_motor.setVelocity(0.0)
imu.enable(timestep)

# Variables PID
integral = 0
prev_error = 0
target_az = 0.0 # Le Nord

print("--- RECHERCHE DU NORD VIA PID ---")

while robot.step(timestep) != -1:
    rpy = imu.getRollPitchYaw()
    if math.isnan(rpy[0]): continue

    # 1. Mesure actuelle (Yaw)
    current_az = math.degrees(rpy[2])
    
    # 2. Calcul de l'erreur (-180 à +180 pour prendre le chemin le plus court)
    error = target_az - current_az
    if error > 180: error -= 360
    elif error < -180: error += 360

    # 3. Calcul du PID
    dt = timestep / 1000.0
    integral += error * dt
    derivative = (error - prev_error) / dt
    
    # Sortie du PID (vitesse)
    output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    
    # 4. Application de la vitesse avec bridage de sécurité
    # On limite la vitesse à 2.0 rad/s pour éviter les mouvements violents
    vitesse = max(min(math.radians(output), 2.0), -2.0)
    az_motor.setVelocity(vitesse * SENS_AZ)

    # Sauvegarde pour le prochain tour
    prev_error = error

    # 5. Affichage des performances
    if robot.getTime() % 0.5 < dt:
        print(f"Angle: {current_az:>7.2f}° | Erreur: {error:>7.2f}° | Vitesse: {vitesse:>5.2f}")

    # Optionnel : Arrêt complet si l'erreur est infime
    if abs(error) < 0.05 and abs(vitesse) < 0.01:
        az_motor.setVelocity(0)
        print(">> NORD ATTEINT AVEC PRÉCISION.")
        # break # Décommenter pour arrêter le script une fois stabilisé