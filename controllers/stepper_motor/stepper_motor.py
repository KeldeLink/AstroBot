import math
from numpy import sign

DEG_PAR_PAS = 0.01

# Dictionnaire global pour suivre la position théorique (en radians)
current_ang = {
    "az": 0.0,
    "alt": 0.0
}

def Step(x, motor, motor_key):
    """Fait avancer le moteur donné d'un certain nombre de pas (x)."""
    global current_ang
    current = current_ang[motor_key]
    
    # Calcule le nouvel angle en radians
    nouvelle_cible = current + x * (math.pi / 180.0) * DEG_PAR_PAS
    
    motor.setPosition(nouvelle_cible)
    current_ang[motor_key] = nouvelle_cible # Mise à jour de la mémoire du moteur

def StepPosition(motor, motor_key, ang_cible, robot, timestep):
    """Atteint l'angle cible par le chemin le plus court."""
    motor.setVelocity(1000)
    
    # 1. Récupère la position actuelle estimée en degrés
    pos_actuelle_deg = -math.degrees(current_ang[motor_key])
    
    # --- OPTIMISATION DU CHEMIN LE PLUS COURT ---
    # Différence brute entre la cible et la position
    diff_brute = ang_cible - pos_actuelle_deg
    
    # On force l'erreur à se situer entre -180° et +180°
    # Ex: Si diff_brute = 270°, (270+180)%360 - 180 = -90° (tourne à gauche)
    erreur_deg = (diff_brute + 180) % 360 - 180
    # --------------------------------------------
    
    # 2. Calcule le nombre exact de pas entiers à effectuer
    nb_pas = int(round(erreur_deg / DEG_PAR_PAS,5))
    
    if nb_pas == 0:
        print(f"[{motor_key.upper()}] Cible déjà atteinte ou trop proche. (Position : {pos_actuelle_deg:.2f}°)\n")
        motor.setVelocity(0.0)
        return

    print(f"[{motor_key.upper()}] Chemin optimisé : {abs(nb_pas)} pas requis pour faire {erreur_deg:.2f}° (Pos: {pos_actuelle_deg:.2f}° -> Cible: {ang_cible}°)")
    
    direction = -sign(nb_pas)
    
    # 3. Effectue les pas un par un
    for _ in range(abs(nb_pas)):
        Step(direction, motor, motor_key)
        print(f"[{motor_key.upper()}] | Nouvelle position : {-math.degrees(current_ang[motor_key]):.2f}°\n")
        print(current_ang["az"])
        # On n'attend qu'un seul "step" physique de Webots pour aller au plus vite
        start_time = robot.getTime()
        while robot.step(timestep) != -1:
            break
        
    
    motor.setVelocity(0.0)