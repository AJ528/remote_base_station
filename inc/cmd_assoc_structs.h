#ifndef __CMD_ASSOC_STRUCTS_H
#define __CMD_ASSOC_STRUCTS_H

#include "device_protocol_structs.h"

#include <stdbool.h>
#include <stdint.h>

enum action_type{
    command,
    cmd_seq
};

struct btn_assoc{
    void *action;
    enum action_type type;
};

struct cmd_seq{
    struct command const *const *sequence;
    uint8_t sequence_len;
};

struct command{
    uint32_t function;
    uint8_t function_len;
    const struct device *device;
};

#endif /* __CMD_ASSOC_STRUCTS_H */
