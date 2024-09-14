#ifndef __DEVICE_PROTOCOL_STRUCTS_H
#define __DEVICE_PROTOCOL_STRUCTS_H

#include <stdint.h>
#include <stdbool.h>

struct command; //defined in cmd

//stream characteristics
struct stream_char{
    // extent_ms is the time a command must take. If done early, do nothing until extent time has passed.
    // if there is no extent, value will be 0
    uint16_t extent_ms;
    int16_t lead_in[2];
    uint8_t lead_in_len;
    int16_t lead_out[2];
    uint8_t lead_out_len;
};

typedef int32_t format_func(const struct command *, bool);

struct protocol{
    uint16_t carrier_freq;
    int16_t enc_zero[2];    
    int16_t enc_one[2];
    bool LSB;
    bool has_ditto;
    struct stream_char primary_stream;
    struct stream_char ditto_stream;
    format_func *fmt_func;
};

struct device{
    uint32_t device_id;
    uint8_t device_len;
    uint32_t subdevice_id;
    uint8_t subdevice_len;
    const struct protocol *prot_used;
};

extern const struct device toshiba_tv;
extern const struct protocol NEC1;


#endif /* __DEVICE_PROTOCOL_STRUCTS_H */
