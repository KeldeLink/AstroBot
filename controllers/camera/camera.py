from controller import Robot

# Initialisation du robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 1. Récupération de l'appareil
# Assure-toi que le nom correspond exactement au champ "name" dans Webots
cam = robot.getDevice("camera")

# 2. Activation de la caméra
# Le timestep définit la fréquence de rafraîchissement (ms)
cam.enable(timestep)

print("--- Caméra activée ---")

# Boucle principale
while robot.step(timestep) != -1:
    # À chaque étape, la caméra capture une nouvelle image
    # On peut récupérer l'image brute avec cam.getImage() si besoin
    pass