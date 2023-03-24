#ifndef __USER_CRC32_H
#define __USER_CRC32_H

#include "user_c.h"

extern uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len);
extern bool_t verify_crc32_check_sum(uint32_t *data, uint32_t len);
extern void append_crc32_check_sum(uint32_t *data, uint32_t len);

#endif /*__USER_CRC32-H*/
