#ifndef __IR_LIB_H
#define __IR_LIB_H

#include <stdint.h>
#include <stdbool.h>

#include "cmd_assoc_structs.h"


void receive_RF_command(uint8_t *RF_cmd, uint32_t RF_cmd_len);
void handle_RF_command_buffer(void);

int32_t execute_command(const struct command *cmd, bool is_ditto);
int32_t execute_command_RF(uint8_t protocol_id, uint8_t device_id, uint8_t subdevice_id, uint8_t function_code);

int32_t format_NEC1_command(const struct command *cmd, bool is_ditto);
int32_t format_NECx2_command(const struct command *cmd, bool is_ditto);
int32_t format_NECx2_command_RF(uint8_t device_id, uint8_t subdevice_id, uint8_t function_code, bool is_ditto);

#endif /* __IR_LIB_H */
