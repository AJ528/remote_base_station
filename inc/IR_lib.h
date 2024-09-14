#ifndef __IR_LIB_H
#define __IR_LIB_H

#include <stdint.h>
#include <stdbool.h>

#include "cmd_assoc_structs.h"


int32_t execute_command(const struct command *cmd, bool is_ditto);

int32_t format_NEC1_command(const struct command *cmd, bool is_ditto);

#endif /* __IR_LIB_H */
