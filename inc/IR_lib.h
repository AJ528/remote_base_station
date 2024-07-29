#ifndef __IR_LIB_H
#define __IR_LIB_H

#include <stdint.h>
#include <stdbool.h>

#include "cmd_assoc_structs.h"

void send_command(void);
int16_t execute_command(const struct command *cmd, bool is_ditto);

#endif /* __IR_LIB_H */
