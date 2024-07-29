#include "device_protocol_structs.h"
#include "IR_lib.h"

//device = 0x40
//subdevice = 0xBF
const struct device toshiba_tv =
{
 .device_id = 0x40,
 .device_len = 8,
 .subdevice_id = 0xBF,
 .subdevice_len = 8,
 .prot_used = &NEC1
};

//IRP notation: {38.0k,564}<1,-1|1,-3>(16,-8,D:8,S:8,F:8,~F:8,1,^108m,(16,-4,1,^108m)*)
const struct protocol NEC1 =
{
 .carrier_freq = 38000,
 .enc_zero = {564, -564},
 .enc_one = {564, -1692},
 .LSB = true,
 .has_ditto = true,
 .primary_stream =
 {
  .extent_ms = 108,
  .lead_in = {9024, -4512},
  .lead_in_len = 2,
  .lead_out = {564},
  .lead_out_len = 1
 },
 .ditto_stream =
 {
  .extent_ms = 108,
  .lead_in = {9024, -2256},
  .lead_in_len = 2,
  .lead_out = {564},
  .lead_out_len = 1
 },
 // TODO: add format function
//  .fmt_func = format_NEC1_command
};

