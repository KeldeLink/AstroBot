#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h> // Indispensable pour la fonction fmod()

#define TIME_STEP 32

int main(void) {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.01);  // Vitesse lente pour tester la précision
  wb_motor_set_velocity(right_motor, -0.01);

  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder, TIME_STEP);
  wb_position_sensor_enable(right_encoder, TIME_STEP);

  bool first_run = true;
  double left_offset = 0.0;
  double right_offset = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_raw = wb_position_sensor_get_value(left_encoder);
    double right_raw = wb_position_sensor_get_value(right_encoder);

    if (first_run) {
        left_offset = left_raw;
        right_offset = right_raw;
        first_run = false;
        printf("--- Calibration OK ---\n");
    }

    // 1. Calcul de la position relative (en radians)
    double left_rel_rad = left_raw - left_offset;
    double right_rel_rad = right_raw - right_offset;

    // 2. Conversion en degrés
    double left_deg_raw = left_rel_rad * 57.2958;
    double right_deg_raw = right_rel_rad * 57.2958;

    // 3. Application du MODULO 360
    // fmod s'assure que la valeur reste entre 0 et 360
    double left_mod = fmod(left_deg_raw, 360.0);
    double right_mod = fmod(right_deg_raw, 360.0);

    // 4. Correction pour les valeurs négatives (si le moteur tourne en sens inverse)
    if (left_mod < 0) left_mod += 360.0;
    if (right_mod < 0) right_mod += 360.0;

    //printf("Angle Azimuth -> L: %8.4f deg | R: %8.4f deg\n", left_mod, right_mod);
    fflush(stdout);
  }

  wb_robot_cleanup();
  return 0;
}