from controller import Supervisor
import math
from astropy.coordinates import get_body, AltAz, EarthLocation
from astropy.time import Time
import astropy.units as u

ASTRE_CIBLE = "sun" 
EMPLACEMENT = EarthLocation(lat=48.5720219, lon=7.766944, height=35*u.m)

# Sens des moteurs
SENS_AZIMUT = 1    
SENS_ALTITUDE = -1   

# Gains PID
Kp_init, Ki_init, Kd_init = 3.5, 0.02, 0.15
Kp_az, Ki_az, Kd_az = 1.5, 0.01, 0.1
Kp_alt, Ki_alt, Kd_alt = 5, 0.02, 0.2

# ==========================================
# INITIALISATION
# ==========================================
robot = Supervisor()
pas_de_temps = int(robot.getBasicTimeStep())

moteur_az = robot.getDevice("azimuth motor")
moteur_alt = robot.getDevice("altitude motor")
capteur_az = robot.getDevice("azimuth sensor")
capteur_alt = robot.getDevice("altitude sensor")
centrale_inertielle = robot.getDevice("inertial unit")

for m in [moteur_az, moteur_alt]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

for s in [capteur_az, capteur_alt, centrale_inertielle]:
    s.enable(pas_de_temps)

camera = robot.getDevice("camera")
camera.enable(pas_de_temps)

# Variables d'état
etat = "RECHERCHE_NORD" 
offset_enc_az = offset_enc_alt = 0
integrale_init = erreur_precedente_init = 0
cible_az_init = 0.0  # On cherche le Nord (0°)

integrales_suivi = {"az": 0, "alt": 0}
erreurs_prec_suivi = {"az": 0, "alt": 0}

noeud_cible = robot.getFromDef("TARGET_SPHERE")
noeud_robot = robot.getSelf() 
distance_visuelle = 30

print("--- RECHERCHE DU NORD (+X) ---")

while robot.step(pas_de_temps) != -1:
    maintenant = Time.now()
    dt = pas_de_temps / 1000.0
    temps_simu = robot.getTime()

    # Lecture des encodeurs en degrés
    enc_az = math.degrees(capteur_az.getValue())
    enc_alt = math.degrees(capteur_alt.getValue())

    # ==========================================
    # PHASE 1 : RECHERCHE DU NORD (PID + IMU)
    # ==========================================
    if etat == "RECHERCHE_NORD":
        rpy = centrale_inertielle.getRollPitchYaw()
        if math.isnan(rpy[0]): continue

        azimut_actuel_imu = math.degrees(rpy[2]) # Lacet (Yaw)
        erreur = cible_az_init - azimut_actuel_imu
        
        # Gestion du passage 180/-180
        if erreur > 180: erreur -= 360
        elif erreur < -180: erreur += 360

        integrale_init += erreur * dt
        derivee = (erreur - erreur_precedente_init) / dt
        sortie_pid = (Kp_init * erreur) + (Ki_init * integrale_init) + (Kd_init * derivee)
        
        vitesse = max(min(math.radians(sortie_pid), 2.0), -2.0)
        moteur_az.setVelocity(vitesse * SENS_AZIMUT)
        erreur_precedente_init = erreur

        if temps_simu % 0.5 < dt:
            print(f"INITIALISATION | Nord (+X) | IMU: {azimut_actuel_imu:>7.2f}° | Err: {erreur:>8.3f}°")

        # Si on est stable ou que le temps est écoulé
        if (abs(erreur) < 0.05 and abs(vitesse) < 0.01) or temps_simu > 15.0:
            moteur_az.setVelocity(0)
            offset_enc_az = enc_az - (azimut_actuel_imu / SENS_AZIMUT)
            offset_enc_alt = enc_alt 
            etat = "SUIVI_ASTRE"
            print("\n--- BASCULE EN MODE SUIVI ---\n")

    # ==========================================
    # PHASE 2 : SUIVI (PID + ENCODEURS)
    # ==========================================
    elif etat == "SUIVI_ASTRE":
        azimut_actuel = ((enc_az - offset_enc_az) * SENS_AZIMUT) % 360.0
        altitude_actuelle = (enc_alt - offset_enc_alt) * SENS_ALTITUDE

        # Calcul de la position de l'astre avec Astropy
        with u.set_enabled_equivalencies(u.dimensionless_angles()):
            corps = get_body(ASTRE_CIBLE, maintenant, EMPLACEMENT)
            coordonnees = corps.transform_to(AltAz(obstime=maintenant, location=EMPLACEMENT))
            cible_az = coordonnees.az.deg
            cible_alt = coordonnees.alt.deg

        # Calcul des erreurs de suivi
        err_az = cible_az - azimut_actuel
        if err_az > 180: err_az -= 360
        elif err_az < -180: err_az += 360
        err_alt = cible_alt - altitude_actuelle

        # PID Azimut
        integrales_suivi["az"] += err_az * dt
        v_az = (Kp_az * err_az) + (Ki_az * integrales_suivi["az"]) + (Kd_az * (err_az - erreurs_prec_suivi["az"])/dt)
        
        # PID Altitude
        integrales_suivi["alt"] += err_alt * dt
        v_alt = (Kp_alt * err_alt) + (Ki_alt * integrales_suivi["alt"]) + (Kd_alt * (err_alt - erreurs_prec_suivi["alt"])/dt)

        moteur_az.setVelocity(max(min(math.radians(v_az), 2.0), -2.0) * SENS_AZIMUT)
        moteur_alt.setVelocity(max(min(math.radians(v_alt), 1.5), -1.5) * SENS_ALTITUDE)
        
        erreurs_prec_suivi["az"], erreurs_prec_suivi["alt"] = err_az, err_alt

        # ==========================================
        # MISE À JOUR DE LA SPHÈRE VISUELLE
        # ==========================================
        if noeud_cible and noeud_robot:
            az_rad = math.radians(cible_az)
            alt_rad = math.radians(cible_alt)
            
            # Nord = +X | Est = +Y | Haut = +Z
            x_rel = distance_visuelle * math.cos(alt_rad) * math.cos(az_rad)
            y_rel = distance_visuelle * math.cos(alt_rad) * math.sin(az_rad)
            z_rel = distance_visuelle * math.sin(alt_rad)
            
            pos_robot = noeud_robot.getField("translation").getSFVec3f()
            
            noeud_cible.getField("translation").setSFVec3f([
                pos_robot[0] + x_rel,
                pos_robot[1] + y_rel,
                pos_robot[2] + z_rel
            ])

        # Affichage console
        if robot.getTime() % 1.0 < dt:
            print(f"[{ASTRE_CIBLE.upper()}] Cible AZ: {cible_az:.2f}° | ALT: {cible_alt:.2f}°")
            print(f"Erreur -> AZ: {err_az:.3f}° | ALT: {err_alt:.3f}°")
            print("-" * 20)