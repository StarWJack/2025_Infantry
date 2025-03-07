//
// Created by RM UI Designer
//

#include "ui_shooter1_group1_0.h"
#include "string.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 0

ui_string_frame_t ui_shooter1_group1_0;

ui_interface_string_t* ui_shooter1_group1_auto = &ui_shooter1_group1_0.option;

void _ui_init_shooter1_group1_0() {
    ui_shooter1_group1_0.option.figure_name[0] = FRAME_ID;
    ui_shooter1_group1_0.option.figure_name[1] = GROUP_ID;
    ui_shooter1_group1_0.option.figure_name[2] = START_ID;
    ui_shooter1_group1_0.option.operate_tpyel = 1;
    ui_shooter1_group1_0.option.figure_tpye = 7;
    ui_shooter1_group1_0.option.layer = 0;
    ui_shooter1_group1_0.option.font_size = 20;
    ui_shooter1_group1_0.option.start_x = 1352;
    ui_shooter1_group1_0.option.start_y = 694;
    ui_shooter1_group1_0.option.color = 8;
    ui_shooter1_group1_0.option.str_length = 4;
    ui_shooter1_group1_0.option.width = 2;
    strcpy(ui_shooter1_group1_auto->string, "AUTO");

    ui_proc_string_frame(&ui_shooter1_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group1_0, sizeof(ui_shooter1_group1_0));
}

void _ui_update_shooter1_group1_0() {
    ui_shooter1_group1_0.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_shooter1_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group1_0, sizeof(ui_shooter1_group1_0));
}

void _ui_remove_shooter1_group1_0() {
    ui_shooter1_group1_0.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_shooter1_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group1_0, sizeof(ui_shooter1_group1_0));
}