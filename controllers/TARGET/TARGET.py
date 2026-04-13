from controller import Supervisor
import math
from astropy.coordinates import get_sun, AltAz, EarthLocation
from astropy.time import Time
import astropy.units as u

# --- Configuration ---
LOCATION = EarthLocation(lat=48.5720*u.deg, lon=7.7669*u.deg, height=35*u.m)
RAYON = 1 

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
target_node = robot.getFromDef("TARGET_SPHERE")
robot_node = robot.getSelf()

print("--- ALIGNEMENT : NORD = +X (ROUGE) | HAUTEUR = +Z (BLEU) ---")

while robot.step(timestep) != -1:
    now = Time.now()
    #now = Time.now() + (robot.getTime() * 3600 * u.s)
    altaz_frame = AltAz(obstime=now, location=LOCATION)
    sun = get_sun(now).transform_to(altaz_frame)
    
    az_rad = sun.az.rad
    alt_rad = sun.alt.rad

    # --- NOUVELLE CONVERSION SELON TES AXES ---
    # Si le Nord du Robot est +X (Rouge) :
    # x (Nord/Sud) = Rayon * cos(Altitude) * cos(Azimut)
    # y (Est/Ouest) = Rayon * cos(Altitude) * sin(Azimut)
    # z (Hauteur) = Rayon * sin(Altitude)
    
    x = RAYON * math.cos(alt_rad) * math.cos(az_rad)
    y = RAYON * math.cos(alt_rad) * math.sin(az_rad)
    z = RAYON * math.sin(alt_rad)

    # Note : Si le soleil tourne dans le mauvais sens, 
    # remplace 'sin(az_rad)' par '-sin(az_rad)' ci-dessus.

    if target_node and robot_node:
        pos_r = robot_node.getField("translation").getSFVec3f()
        
        # On applique les coordonnées sur les bons axes Webots
        # Ici, j'utilise l'ordre (x, y, z) calculé ci-dessus.
        target_node.getField("translation").setSFVec3f([
            pos_r[0] + x, 
            pos_r[1] + y, 
            pos_r[2] + z
        ])

    if robot.getTime() % 2.0 < (timestep/1000):
        print(f"Soleil -> Az: {sun.az.deg:.1f}° | Alt: {sun.alt.deg:.1f}°")
        print(f"Position -> Rouge(X): {x:.1f} | Vert(Y): {y:.1f} | Bleu(Z): {z:.1f}")