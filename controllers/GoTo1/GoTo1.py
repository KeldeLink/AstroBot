from controller import Supervisor
import math
from astropy.coordinates import get_body, AltAz, EarthLocation
from astropy.time import Time
import astropy.units as u

# ==========================================
# CONFIGURATION
# ==========================================
TARGET_BODY = "sun" 
LOCATION = EarthLocation(lat=48.5720219, lon=7.766944, height=35*u.m)

# Sens moteurs (à ajuster selon la mécanique)
SENS_AZ = 1    
SENS_ALT = -1   

# Gains PID
Kp_h, Ki_h, Kd_h = 3.5, 0.02, 0.15
Kp_az, Ki_az, Kd_az = 1.5, 0.01, 0.1
Kp_alt, Ki_alt, Kd_alt = 5, 0.02, 0.2

# ==========================================
# INITIALISATION
# ==========================================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

az_motor = robot.getDevice("azimuth motor")
alt_motor = robot.getDevice("altitude motor")
az_sensor = robot.getDevice("azimuth sensor")
alt_sensor = robot.getDevice("altitude sensor")
imu = robot.getDevice("inertial unit")

for m in [az_motor, alt_motor]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

for s in [az_sensor, alt_sensor, imu]:
    s.enable(timestep)

cam = robot.getDevice("camera")
cam.enable(timestep)

# Variables d'état
state = "HOMING_AZ" 
offset_enc_az = offset_enc_alt = 0
integral_h = prev_error_h = 0
target_az_h = 0.0  # On cherche le Nord (0°)

integrals_t = {"az": 0, "alt": 0}
prev_errs_t = {"az": 0, "alt": 0}

target_node = robot.getFromDef("TARGET_SPHERE")
self_node = robot.getSelf() 
distance_affichage = 30

print("--- RECHERCHE DU NORD (+X) ---")

while robot.step(timestep) != -1:
    now = Time.now()
    dt = timestep / 1000.0
    temps_ecoule = robot.getTime()

    enc_az = math.degrees(az_sensor.getValue())
    enc_alt = math.degrees(alt_sensor.getValue())

    # ==========================================
    # PHASE 1 : HOMING (PID + IMU)
    # ==========================================
    if state == "HOMING_AZ":
        rpy = imu.getRollPitchYaw()
        if math.isnan(rpy[0]): continue

        current_az_imu = math.degrees(rpy[2]) # Yaw
        error = target_az_h - current_az_imu
        
        if error > 180: error -= 360
        elif error < -180: error += 360

        integral_h += error * dt
        derivative = (error - prev_error_h) / dt
        output = (Kp_h * error) + (Ki_h * integral_h) + (Kd_h * derivative)
        
        vitesse = max(min(math.radians(output), 2.0), -2.0)
        az_motor.setVelocity(vitesse * SENS_AZ)
        prev_error_h = error

        if temps_ecoule % 0.5 < dt:
            print(f"HOMING | Nord (+X) | IMU: {current_az_imu:>7.2f}° | Err: {error:>8.3f}°")

        if (abs(error) < 0.05 and abs(vitesse) < 0.01) or temps_ecoule > 15.0:
            az_motor.setVelocity(0)
            offset_enc_az = enc_az - (current_az_imu / SENS_AZ)
            offset_enc_alt = enc_alt 
            state = "TRACKING"
            print("\n--- BASCULE EN MODE TRACKING ---\n")

    # ==========================================
    # PHASE 2 : TRACKING (PID + ENCODEURS)
    # ==========================================
    elif state == "TRACKING":
        current_az = ((enc_az - offset_enc_az) * SENS_AZ) % 360.0
        current_alt = (enc_alt - offset_enc_alt) * SENS_ALT

        # Cible Astropy
        with u.set_enabled_equivalencies(u.dimensionless_angles()):
            body = get_body(TARGET_BODY, now, LOCATION)
            altaz = body.transform_to(AltAz(obstime=now, location=LOCATION))
            target_az = altaz.az.deg
            target_alt = altaz.alt.deg

        # Calcul des erreurs
        err_az = target_az - current_az
        if err_az > 180: err_az -= 360
        elif err_az < -180: err_az += 360
        err_alt = target_alt - current_alt

        # PID Azimut
        integrals_t["az"] += err_az * dt
        v_az = (Kp_az * err_az) + (Ki_az * integrals_t["az"]) + (Kd_az * (err_az - prev_errs_t["az"])/dt)
        
        # PID Altitude
        integrals_t["alt"] += err_alt * dt
        v_alt = (Kp_alt * err_alt) + (Ki_alt * integrals_t["alt"]) + (Kd_alt * (err_alt - prev_errs_t["alt"])/dt)

        az_motor.setVelocity(max(min(math.radians(v_az), 2.0), -2.0) * SENS_AZ)
        alt_motor.setVelocity(max(min(math.radians(v_alt), 1.5), -1.5) * SENS_ALT)
        
        prev_errs_t["az"], prev_errs_t["alt"] = err_az, err_alt

        # ==========================================
        # MISE À JOUR DE LA SPHÈRE (REPERE ROBOT)
        # ==========================================
        if target_node and self_node:
            az_rad = math.radians(target_az)
            alt_rad = math.radians(target_alt)
            
            # --- CONVERSION SELON TON GIZMO ---
            # Nord = +X (Rouge) | Est = +Y (Vert) | Haut = +Z (Bleu)
            x_rel = distance_affichage * math.cos(alt_rad) * math.cos(az_rad)
            y_rel = distance_affichage * math.cos(alt_rad) * math.sin(az_rad)
            z_rel = distance_affichage * math.sin(alt_rad)
            
            # Position du robot
            pos_robot = self_node.getField("translation").getSFVec3f()
            
            # Mise à jour
            target_node.getField("translation").setSFVec3f([
                pos_robot[0] + x_rel,
                pos_robot[1] + y_rel,
                pos_robot[2] + z_rel
            ])

        # Affichage
        if robot.getTime() % 1.0 < dt:
            print(f"[{TARGET_BODY.upper()}] Cible AZ: {target_az:.2f}° | ALT: {target_alt:.2f}°")
            print(f"Erreur -> AZ: {err_az:.3f}° | ALT: {err_alt:.3f}°")
            print("-" * 20)