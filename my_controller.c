#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 256
#define NUM_SENSORS 8
#define NUM_LEDS 8
#define VELOCITY_SCALE 6.28
#define PROXIMITY_THRESHOLD 200
#define RECOVERY_THRESHOLD 100
#define TOLERANCE 0.01
#define TURN_DURATION 3

int main(int argc, char **argv) {
  int i;
  double sensor_readings[NUM_SENSORS];
  double left_speed = 1.0, right_speed = 1.0;
  int turning = 0, sensor_triggered = -1, turn_counter = 1;
  const double *current_pos, *previous_pos;

  // Inicialização do Webots
  wb_robot_init();

  // Configuração dos motores
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  // Configuração dos sensores de proximidade
  WbDeviceTag proximity_sensors[NUM_SENSORS];
  for (i = 0; i < NUM_SENSORS; i++) {
    char sensor_name[5];
    sprintf(sensor_name, "ps%d", i);
    proximity_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(proximity_sensors[i], TIME_STEP);
  }

  // Configuração dos LEDs
  WbDeviceTag leds[NUM_LEDS];
  for (i = 0; i < NUM_LEDS; i++) {
    char led_name[5];
    sprintf(led_name, "led%d", i);
    leds[i] = wb_robot_get_device(led_name);
  }

  // Configuração do supervisor
  WbNodeRef box = wb_supervisor_node_get_from_def("box1");
  previous_pos = wb_supervisor_node_get_position(box);

  // Loop principal
  while (wb_robot_step(TIME_STEP) != -1) {
    // Atualização da posição da caixa
    current_pos = wb_supervisor_node_get_position(box);
    if (fabs(current_pos[0] - previous_pos[0]) > TOLERANCE || fabs(current_pos[1] - previous_pos[1]) > TOLERANCE) {
      // Parada do robô ao detectar colisão com a caixa
      for (i = 0; i < NUM_LEDS; i++) {
        wb_led_set(leds[i], 1); // Liga todos os LEDs
      }
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break; // Sai do loop principal
    }

    // Leitura dos sensores de proximidade
    for (i = 0; i < NUM_SENSORS; i++) {
      sensor_readings[i] = wb_distance_sensor_get_value(proximity_sensors[i]) - 60;
    }

    // Verificação dos sensores e ajuste de direção
    for (i = 0; i < NUM_SENSORS; i++) {
      if (sensor_readings[i] > PROXIMITY_THRESHOLD && turning == 0) {
        sensor_triggered = i;
      }
    }

    // Controle do movimento com base na leitura dos sensores
    if (sensor_triggered != -1) {
      if (turning == 0) {
        turn_counter = 1;  // Reinicia contador ao detectar um obstáculo
        turning = 1;
      }
      // Controla a direção do movimento por um período limitado
      if (turn_counter < TURN_DURATION) {
        left_speed = -1;
        right_speed = 1;
        turn_counter++;
      } else {
        left_speed = 1;
        right_speed = 1;
        sensor_triggered = -1;  // Reseta estado após o período de "virada"
        turning = 0;
      }
    } else {
      left_speed = 1;
      right_speed = 1;
    }

    // Ajuste da velocidade dos motores
    wb_motor_set_velocity(left_motor, VELOCITY_SCALE * left_speed);
    wb_motor_set_velocity(right_motor, VELOCITY_SCALE * right_speed);

    // Atualiza a posição anterior
    previous_pos = current_pos;
  }

  // Finalização
  wb_robot_cleanup();
  return 0;
}
