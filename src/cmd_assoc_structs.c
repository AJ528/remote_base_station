
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

//function = 0x41
const struct command SB_VLUP =
{
 .function = 0x41,
 .function_len = 8,
 .device = &soundbar
};

//function = 0x45
const struct command SB_VLDN =
{
 .function = 0x45,
 .function_len = 8,
 .device = &soundbar
};

//function = 0x48
const struct command SB_MUTE =
{
 .function = 0x48,
 .function_len = 8,
 .device = &soundbar
};