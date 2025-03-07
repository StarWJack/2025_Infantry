//
// Created by RM UI Designer
//

#include "ui_shooter1_group3_0.h"

#define FRAME_ID 1
#define GROUP_ID 1
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_shooter1_group3_0;
ui_interface_line_t *ui_shooter1_group3_Front = (ui_interface_line_t *)&(ui_shooter1_group3_0.data[0]);

void _ui_init_shooter1_group3_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group3_0.data[i].figure_name[0] = FRAME_ID;
        ui_shooter1_group3_0.data[i].figure_name[1] = GROUP_ID;
        ui_shooter1_group3_0.data[i].figure_name[2] = i + START_ID;
        ui_shooter1_group3_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_shooter1_group3_0.data[i].operate_tpyel = 0;
    }

    ui_shooter1_group3_Front->figure_tpye = 0;
    ui_shooter1_group3_Front->layer = 0;
    ui_shooter1_group3_Front->start_x = 273;
    ui_shooter1_group3_Front->start_y = 723;
    ui_shooter1_group3_Front->end_x = 273;
    ui_shooter1_group3_Front->end_y = 807;
    ui_shooter1_group3_Front->color = 1;
    ui_shooter1_group3_Front->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group3_0, sizeof(ui_shooter1_group3_0));
}

void _ui_update_shooter1_group3_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group3_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group3_0, sizeof(ui_shooter1_group3_0));
}

void _ui_remove_shooter1_group3_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group3_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group3_0, sizeof(ui_shooter1_group3_0));
}
