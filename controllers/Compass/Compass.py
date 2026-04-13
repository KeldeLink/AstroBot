from controller import Robot
import math

# Initialisation
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Récupération du périphérique
compass = robot.getDevice("compass")
compass.enable(timestep)

print("--- Lecture du Compas lancée ---")

while robot.step(timestep) != -1:
    v = compass.getValues()
    
    # On teste les 3 combinaisons possibles
    angle_XZ = math.degrees(math.atan2(v[0], v[2]))
    angle_XY = math.degrees(math.atan2(v[0], v[1]))
    angle_YZ = math.degrees(math.atan2(v[1], v[2]))
    
    print(f"Brut: X={v[0]:.4f} | Y={v[1]:.4f} | Z={v[2]:.4f}")
    print(f"Angles possibles -> XZ: {angle_XZ:.1f}° | XY: {angle_XY:.1f}° | YZ: {angle_YZ:.1f}°")
    
    # Récupération des valeurs [X, Y, Z]
    north = compass.getValues()
    
    # On utilise Y (index 1) et X (index 0) comme dans ton exemple C
    # Attention : l'ordre dans atan2 est souvent (Y, X) pour obtenir l'angle depuis l'axe X
    angle_rad = math.atan2(north[1], north[0])
    angle_deg = math.degrees(angle_rad)
    
    print(f"Angle boussole : {angle_deg:.2f}°")
   