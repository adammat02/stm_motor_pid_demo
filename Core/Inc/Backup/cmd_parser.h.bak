#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  CMD_SET_SPEED,
  CMD_SET_PID,
  CMD_GET_POS,
  CMD_UNKNOWN
} CmdType;

typedef struct
{
  float kp, ki, kd;
} set_pid_t;

typedef struct
{
  CmdType cmd;
  union
  {
    int16_t speeds[4];
    set_pid_t set_pid;
  };
  
} Command;

bool parse_command(const char *line, Command *cmd);

bool dispatch_command(const char *out, Command *cmd);

#endif