#ifndef OD_H
#define OD_H

#include "MainInclude.h"
#include "canfd.h"
#include "motor_ctrl.h"

#define ATTR_R      0x01
#define ATTR_W      0x02
#define ATTR_RW     0x03
#define ATTR_ROM    0x04
#define ATTR_RAM    0x08

typedef struct {
    uint16_t error_code;
    uint16_t control_word;
    uint16_t node_id;
    uint16_t in_encoder_offset;
    uint16_t ex_encoder_offset;
    uint16_t firmware_version;
}ODObjs_t;

extern ODObjs_t ODObjs;

void OD_init(void);
uint16_t OD_read(uint16_t idx, uint16_t *data);
uint16_t OD_write_1(uint16_t idx, uint16_t *data);
uint16_t OD_write_2(uint16_t idx, uint16_t *data);
uint16_t OD_write_4(uint16_t idx, uint16_t *data);

#endif
