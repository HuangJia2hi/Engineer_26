#include "referee_ui_config.h"

const referee_ui_auto_cmd_map_t g_referee_ui_auto_cmd_map[] = {
    {CMD_AUTO_GET_A_POS, REFEREE_UI_GET_SLOT_2, REFEREE_UI_GET_SLOT_2},
    {CMD_AUTO_GET_A_SET, REFEREE_UI_GET_SLOT_2, REFEREE_UI_GET_SLOT_2},
    {CMD_AUTO_GET_B_POS, REFEREE_UI_GET_SLOT_3, REFEREE_UI_GET_SLOT_3},
    {CMD_AUTO_GET_B_SET, REFEREE_UI_GET_SLOT_3, REFEREE_UI_GET_SLOT_3},
    {CMD_AUTO_GET_C_POS, REFEREE_UI_GET_SLOT_4, REFEREE_UI_GET_SLOT_4},
    {CMD_AUTO_GET_C_SET, REFEREE_UI_GET_SLOT_4, REFEREE_UI_GET_SLOT_4},
};

const uint32_t g_referee_ui_auto_cmd_map_count =
    sizeof(g_referee_ui_auto_cmd_map) / sizeof(g_referee_ui_auto_cmd_map[0]);

const referee_ui_auto_get_map_t g_referee_ui_auto_get_map[] = {
    {CMD_AUTO_GET_RIGHT_BACK, REFEREE_UI_GET_SLOT_4, REFEREE_UI_GET_SLOT_4},
    {CMD_AUTO_GET_RIGHT_MID, REFEREE_UI_GET_SLOT_3, REFEREE_UI_GET_SLOT_3},
    {CMD_AUTO_GET_RIGHT_FRONT, REFEREE_UI_GET_SLOT_2, REFEREE_UI_GET_SLOT_2},
    {CMD_AUTO_GET_LEFT_FORNT, REFEREE_UI_GET_SLOT_1, REFEREE_UI_GET_SLOT_1},
};

const uint32_t g_referee_ui_auto_get_map_count =
    sizeof(g_referee_ui_auto_get_map) / sizeof(g_referee_ui_auto_get_map[0]);

const referee_ui_auto_color_config_t g_referee_ui_auto_color_config = {
    .get_flag = {
        .inactive_color = UI_Color_Main,
        .active_color = UI_Color_Green,
    },
    .get_ring = {
        .inactive_color = UI_Color_Main,
        .active_color = UI_Color_Green,
    },
    .set_flag = {
        .inactive_color = UI_Color_Main,
        .active_color = UI_Color_Green,
    },
    .reget_flag = {
        .inactive_color = UI_Color_Main,
        .active_color = UI_Color_Green,
    },
    .reget_ring = {
        .inactive_color = UI_Color_Main,
        .active_color = UI_Color_Green,
    },
};

const referee_ui_status_color_config_t g_referee_ui_status_color_config = {
    .inactive_color = UI_Color_Main,
    .active_color = UI_Color_Green,
    .alt_active_color_1 = UI_Color_Green,
    .alt_active_color_2 = UI_Color_Green,
};
