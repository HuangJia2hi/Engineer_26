#ifndef AUTO_KEY_BOARD_H
#define AUTO_KEY_BOARD_H

#include <stdint.h>
typedef enum auto_key_cmd_t{
    CMD_NONE,
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_A_SET,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_B_SET,
    CMD_AUTO_GET_C_POS,
    CMD_AUTO_GET_C_SET,
    CMD_EMERENCY_STASH,     /* 紧急存矿，配合 emerency_stash_get_idx 使用 */
}auto_key_cmd_t;

typedef enum auto_key_get_cmd_t {
    CMD_GET_NONE,
    CMD_AUTO_GET_RIGHT_BACK,
    CMD_AUTO_GET_RIGHT_MID,
    CMD_AUTO_GET_RIGHT_FRONT,
    CMD_AUTO_GET_LEFT_FORNT,
} auto_key_get_cmd_t;

#ifdef __cplusplus
extern "C" {
#endif

extern auto_key_cmd_t auto_key_cmd;
extern auto_key_get_cmd_t auto_key_get_cmd;
extern uint8_t emerency_stash_get_idx;  /* 紧急存矿的目标位置索引 (0~3) */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C"
#endif
void auto_key_cmd_exec(auto_key_cmd_t cmd);

#ifdef __cplusplus
extern "C"
#endif
void auto_key_get_cmd_exec(auto_key_get_cmd_t cmd);

#endif
