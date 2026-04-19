/**
 * @file cmd_parser.h
 * @brief Simple text command parser.
 *
 * Parses single-line ASCII commands into a Command struct. Supported forms:
 *   - "S <s0> <s1> <s2> <s3>"  set speed of 4 motors (int16)
 *   - "P <kp> <ki> <kd>"       set PID gains (float)
 *   - "E"                      request encoder position
 */

#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/** Recognised command types. */
typedef enum {
  CMD_SET_SPEED,
  CMD_SET_PID,
  CMD_GET_POS,
  CMD_UNKNOWN
} CmdType;

/** Payload for CMD_SET_PID. */
typedef struct
{
  float kp, ki, kd;
} set_pid_t;

/** Parsed command with tag-dependent payload. */
typedef struct
{
  CmdType cmd;
  union
  {
    int16_t speeds[4];   /**< Used when cmd == CMD_SET_SPEED. */
    set_pid_t set_pid;   /**< Used when cmd == CMD_SET_PID. */
  };

} Command;

/**
 * @brief Parse a single line into a Command.
 *
 * Leading whitespace is skipped. The line must contain all fields required
 * by the command type; otherwise parsing fails and @p cmd is left untouched.
 *
 * @param line  Null-terminated input line.
 * @param cmd   Destination command, written only on success.
 * @return true on successful parse, false on malformed or unknown input.
 */
bool parse_command(const char *line, Command *cmd);

#endif
