#include "cmd_parser.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

bool parse_command(const char *line, Command *cmd)
{
  char type;
  if (sscanf(line, " %c", &type) < 1)
    return false;

  switch (type)
  {
  case CMD_SET_SPEED:
  {
    cmd->cmd = CMD_SET_SPEED;
    return sscanf(line, " %*c %f %f %f %f",
                  &cmd->data.speeds[0], &cmd->data.speeds[1],
                  &cmd->data.speeds[2], &cmd->data.speeds[3]) == 4;
  }
  case CMD_SET_PID:
  {
    cmd->cmd = CMD_SET_PID;
    return sscanf(line, " %*c %f %f %f",
                  &cmd->set_pid.kp, &cmd->set_pid.ki, &cmd->set_pid.kd) == 3;
  }
  case CMD_FULL_FRAME_RX:
  {
    cmd->cmd = CMD_FULL_FRAME_RX;
    return sscanf(line, " %*c %f %f %f %f",
                  &cmd->data.speeds[0], &cmd->data.speeds[1],
                  &cmd->data.speeds[2], &cmd->data.speeds[3]) == 4;
  }
  case CMD_GET_POS:
  {
    cmd->cmd = CMD_GET_POS;
    return true;
  }
  case CMD_RESET:
  {
    cmd->cmd = CMD_RESET;
    return true;
  }
  default:
    return false;
  }
}
