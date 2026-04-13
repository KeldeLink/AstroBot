from controller import Supervisor
import math
from astropy.coordinates import get_body, AltAz, EarthLocation
from astropy.time import Time
import astropy.units as u

# ==========================================
# CONFIGURATION
# ==========================================
TARGET_BODY = "jupiter" 
LOCATION = EarthLocation(lat=48.5720219, lon=7.766944, height=35*u.m)

SENS_AZ = 1      
SENS_ALT = -1   

# --- GAINS STABILISÉS ---
Kp_h, Ki_h, Kd_h = 2.0, 0.01, 0.5
Kp_az, Ki_az, Kd_az = 1.0, 0.005, 0.2
Kp_alt, Ki_alt, Kd_alt = 2.0, 0.01, 0.3
Kp_vis = 0.005 

DEADBAND = 0.05 # Seuil en degrés (Homing et Astro)
DEADBAND_VISUEL = 130 # Seuil en pixels pour éviter le tremblement caméra

# ==========================================
# INITIALISATION
# ==========================================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

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
if cam.hasRecognition():
    cam.recognitionEnable(timestep)

# Variables d'état
state = "HOMING_AZ" 
offset_enc_az = offset_enc_alt = 0
integral_h = prev_error_h = 0
target_az_h = 0.0  # On cherche le Nord (0°)
debut_tracking_astro = 0.0

integrals_t = {"az": 0, "alt": 0}
prev_errs_t = {"az": 0, "alt": 0}

target_node = robot.getFromDef("TARGET_SPHERE")
self_node = robot.getSelf() 
distance_affichage = 40 

def calcul_pid(err, integral, prev_err, kp, ki, kd, dt):
    if abs(err) < DEADBAND:
        return 0.0, 0.0 
    
    new_integral = integral + (err * dt)
    new_integral = max(min(new_integral, 5.0), -5.0)
    
    derivative = (err - prev_err) / dt
    output = (kp * err) + (ki * new_integral) + (kd * derivative)
    return output, new_integral

while robot.step(timestep) != -1:
    now = Time.now()
    temps_simu = robot.getTime()

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

        if temps_simu % 0.5 < dt:
            print(f"HOMING | Nord (+X) | IMU: {current_az_imu:>7.2f}° | Err: {error:>8.3f}°")

        if (abs(error) < 0.05 and abs(vitesse) < 0.01) or temps_simu > 10:
            az_motor.setVelocity(0)
            offset_enc_az = enc_az - (current_az_imu / SENS_AZ)
            offset_enc_alt = enc_alt 
            state = "TRACKING_ASTRO"
            print("\n--- BASCULE EN MODE TRACKING ---\n")
            debut_tracking_astro = temps_simu


    # ==========================================
    # PHASE 2 : TRACKING ASTRO
    # ==========================================
    elif state == "TRACKING_ASTRO":
        current_az = ((enc_az - offset_enc_az) * SENS_AZ) % 360.0
        current_alt = (enc_alt - offset_enc_alt) * SENS_ALT

        with u.set_enabled_equivalencies(u.dimensionless_angles()):
            body = get_body(TARGET_BODY, now, LOCATION)
            altaz = body.transform_to(AltAz(obstime=now, location=LOCATION))
            target_az, target_alt = altaz.az.deg, altaz.alt.deg

        err_az = target_az - current_az
        if err_az > 180: err_az -= 360
        elif err_az < -180: err_az += 360
        err_alt = target_alt - current_alt

        out_az, integrals_t["az"] = calcul_pid(err_az, integrals_t["az"], prev_errs_t["az"], Kp_az, Ki_az, Kd_az, dt)
        out_alt, integrals_t["alt"] = calcul_pid(err_alt, integrals_t["alt"], prev_errs_t["alt"], Kp_alt, Ki_alt, Kd_alt, dt)

        az_motor.setVelocity(max(min(math.radians(out_az), 1.5), -1.5) * SENS_AZ)
        alt_motor.setVelocity(max(min(math.radians(out_alt), 1.0), -1.0) * SENS_ALT)
        
        prev_errs_t["az"], prev_errs_t["alt"] = err_az, err_alt

        if (temps_simu - debut_tracking_astro) > 30.0:
            state = "TRACKING_VISUEL"

    # ==========================================
    # PHASE 3 : TRACKING VISUEL (CAMERA)
    # ==========================================
    elif state == "TRACKING_VISUEL":
        objs = cam.getRecognitionObjects()
        if len(objs) > 0:
            pos_img = objs[0].getPositionOnImage() 
            
            err_x = (cam.getWidth() / 2) - pos_img[0]
            err_y = (cam.getHeight() / 2) - pos_img[1]

            # --- CORRECTION DU TREMBLEMENT (ZONE MORTE VISUELLE) ---
            if abs(err_x) < DEADBAND_VISUEL:
                err_x = 0.0
            if abs(err_y) < DEADBAND_VISUEL:
                err_y = 0.0

            v_az_vis = err_x * Kp_vis
            v_alt_vis = err_y * Kp_vis

            v_az_vis = max(min(v_az_vis, 0.5), -0.5)
            v_alt_vis = max(min(v_alt_vis, 0.5), -0.5)

            az_motor.setVelocity(v_az_vis * SENS_AZ)
            alt_motor.setVelocity(v_alt_vis * SENS_ALT)
            
            if temps_simu % 1.0 < dt:
                print(f"VISUEL | Erreur Image : X={err_x:.1f}px Y={err_y:.1f}px")
        else:
            az_motor.setVelocity(0.0)
            alt_motor.setVelocity(0.0)

    # --- MISE À JOUR SPHÈRE ---
    if target_node and self_node:
        with u.set_enabled_equivalencies(u.dimensionless_angles()):
            body_p = get_body(TARGET_BODY, now, LOCATION).transform_to(AltAz(obstime=now, location=LOCATION))
            t_az, t_alt = body_p.az.rad, body_p.alt.rad
        
        x_rel = distance_affichage * math.cos(t_alt) * math.cos(t_az)
        y_rel = distance_affichage * math.cos(t_alt) * math.sin(t_az)
        z_rel = distance_affichage * math.sin(t_alt)
        pos_robot = self_node.getField("translation").getSFVec3f()
        target_node.getField("translation").setSFVec3f([pos_robot[0]+x_rel, pos_robot[1]+y_rel, pos_robot[2]+z_rel])