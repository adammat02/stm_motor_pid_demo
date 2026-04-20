#include "controller.h"
#include "ctrl_config.h"
#include <stdio.h>
#include "uart_comm.h"
#include "micros.h"
#include "cmd_parser.h"

static RotationDirection_t dirs[N_MOTORS] = {ROTATION_CCW, ROTATION_CCW, ROTATION_CCW, ROTATION_CCW};
static float poses[N_MOTORS];
static float32_t set_speed[N_MOTORS];

static char rx_buff[100], out[128];
static Command cmd;

void controller_init()
{
  for (uint8_t i = 0; i < N_MOTORS; i++)
  {
    motor_init(&motors[i]);
    encoder_init(&encoders[i]);
    motor_pid_init(&motor_pids[i]);
  }
}

static void controller_execute(const Command *cmd)
{
  CmdType type = cmd->cmd;
  switch (type)
  {
  case CMD_GET_POS:
  {
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      poses[i] = encoder_get_rotations(&encoders[i]) * (float)motor_sign[i];
    }
    sprintf(out, "E %.3f %.3f %.3f %.3f\r", poses[0], poses[1], poses[2], poses[3]);
    break;
  }
  case CMD_SET_SPEED:
  {
    // sprintf(out, "CMD_SET_SPEED;%hd;%hd;%hd;%hd\r", cmd->speeds[0], cmd->speeds[1], cmd->speeds[2], cmd->speeds[3]);
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      int16_t dir = cmd->speeds[i] * motor_sign[i];
      dirs[i] = (dir >= 0) ? ROTATION_CCW : ROTATION_CW;
      set_speed[i] = fabsf(cmd->speeds[i]);
    }
    strcpy(out, "OK\r");
    break;
  }
  case CMD_SET_PID:
  {
    // sprintf(out, "CMD_SET_PID;%.2f;%.2f;%.2f;\r", cmd->set_pid.kp, cmd->set_pid.ki, cmd->set_pid.kd);
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      motor_pid_set_pid(&motor_pids[i], cmd->set_pid.kp, cmd->set_pid.ki, cmd->set_pid.kd);
    }
    strcpy(out, "OK\r");
    break;
  }
  default:
    strcpy(out, "ERR\r");
    break;
  }
  uart_send_str(out);
}

void controller_poll(void)
{
  if (uart_is_line())
  {
    uart_get_line(rx_buff, sizeof(rx_buff));
    if (parse_command(rx_buff, &cmd))
    {
      controller_execute(&cmd);
    }
    else
    {
      uart_send_str("ERR\r");
    }
  }
}

void controller_update()
{
  for (uint8_t i = 0; i < N_MOTORS; i++)
  {
    motor_pid_update(&motor_pids[i], set_speed[i], dirs[i]);
  }
}