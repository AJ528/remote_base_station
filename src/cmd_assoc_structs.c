
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

//function = 0x07
const struct command LHTV_VLUP =
{
 .function = 0x07,
 .function_len = 8,
 .device = &LH_samsung_tv
};

//function = 0xe6
const struct command LHTV_PWR =
{
 .function = 0xe6,
 .function_len = 8,
 .device = &LH_samsung_tv
};
