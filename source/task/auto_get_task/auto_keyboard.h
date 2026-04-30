#ifndef AUTO_KEY_BOARD_H
#define AUTO_KEY_BOARD_H

typedef enum auto_key_cmd_t{
    CMD_NONE,
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_A_SET,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_B_SET,
    CMD_AUTO_GET_C_POS,
    CMD_AUTO_GET_C_SET,
}auto_key_cmd_t;

extern auto_key_cmd_t auto_key_cmd;

#ifdef __cplusplus
extern "C"
#endif
void auto_key_cmd_exec(auto_key_cmd_t cmd);

#endif
