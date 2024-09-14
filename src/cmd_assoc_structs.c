
#include "cmd_assoc_structs.h"
#include "device_protocol_structs.h"



/*
 * SoundBar Commands
 */

//function = 0x40
const struct command SB_PWR_TOG =
{
 .function = 0x40,
 .function_len = 8,
 .device = &soundbar
};