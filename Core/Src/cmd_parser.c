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
    case 'S':
    {
      int16_t s0, s1, s2, s3;
      if (sscanf(line, " S %hd %hd %hd %hd", &s0, &s1, &s2, &s3) != 4)
        return false;
      
      cmd->cmd = CMD_SET_SPEED;
      cmd->speeds[0] = s0;
      cmd->speeds[1] = s1;
      cmd->speeds[2] = s2;
      cmd->speeds[3] = s3;
      return true;
    }
    case 'P':
    {
      float kp, ki, kd;
      if (sscanf(line, " P %f %f %f", &kp, &ki, &kd) != 3)
        return false;
      
      cmd->cmd = CMD_SET_PID;
      cmd->set_pid.kp = kp;
      cmd->set_pid.ki = ki;
      cmd->set_pid.kd = kd;
      return true;
    }
    case 'E':
    {      
      cmd->cmd = CMD_GET_POS;
      return true;
    }
    default:
      return false;
  }
}

bool dispatch_command(const char *out, Command *cmd)
{
  CmdType type = cmd->cmd;
  switch (type)
  {
    case CMD_GET_POS:
    {
      strcpy(out, "CMD_GET_POS\r");
      return true;
    }
    case CMD_SET_SPEED:
    {
      strcpy(out, "CMD_SET_SPEED\r");
      return true;
    }
    case CMD_SET_PID:
    {
      strcpy(out, "CMD_SET_PID\r");
      return true;
    }
    default:
      return false;
  }
}