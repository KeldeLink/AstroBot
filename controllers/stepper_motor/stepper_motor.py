import math
from numpy import sign

DEG_PAR_PAS = 0.2

# Dictionnaire global pour suivre la position PHYSIQUE théorique (en radians)
# Pour l'altitude : 0 = bas, 45 = horizon, 135 = zénith (si cible 90), 180 = haut
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
    
    # --- GESTION DE L'OFFSET D'ALTITUDE (Type Vespera) ---
    if motor_key == "alt":
        # Si la cible est l'horizon (0°), le moteur doit pointer à 90°
        cible_physique = ang_cible + 90.0
    else:
        cible_physique = ang_cible
        
    # 1. Récupère la position physique actuelle estimée en degrés
    pos_actuelle_deg = -math.degrees(current_ang[motor_key])
    
    # --- OPTIMISATION DU CHEMIN LE PLUS COURT ---
    # On calcule la différence avec la cible PHYSIQUE, pas l'angle astronomique
    diff_brute = cible_physique - pos_actuelle_deg
    
    # On force l'erreur à se situer entre -180° et +180°
    erreur_deg = (diff_brute + 180) % 360 - 180
    # --------------------------------------------
    
    # 2. Calcule le nombre exact de pas entiers à effectuer
    nb_pas = int(round(erreur_deg / DEG_PAR_PAS, 5))
    
    if nb_pas == 0:
        print(f"[{motor_key.upper()}] Cible déjà atteinte. (Moteur : {pos_actuelle_deg:.2f}°)\n")
        motor.setVelocity(0.0)
        return

    # Affichage clair pour comprendre la différence entre la demande et la réalité mécanique
    if motor_key == "alt":
        print(f"[ALT] Demande Astro: {ang_cible}° -> Cible Moteur: {cible_physique}°")
        
    print(f"[{motor_key.upper()}] Chemin optimisé : {abs(nb_pas)} pas requis pour faire {erreur_deg:.2f}°")
    
    direction = -sign(nb_pas)
    
    # 3. Effectue les pas un par un
    for _ in range(abs(nb_pas)):
        Step(direction, motor, motor_key)
        
        # On n'attend qu'un seul "step" physique de Webots pour aller au plus vite
        start_time = robot.getTime()
        while robot.step(timestep) != -1:
            break
        
    print(f"[{motor_key.upper()}] Mouvement terminé ! Nouvelle position moteur : {-math.degrees(current_ang[motor_key]):.2f}°\n")
    motor.setVelocity(0.0)