//
// Created by RM UI Designer
//

#include "ui_shooter1_group5_0.h"

#define FRAME_ID 1
#define GROUP_ID 2
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_shooter1_group5_0;
ui_interface_line_t *ui_shooter1_group5_electricity = (ui_interface_line_t *)&(ui_shooter1_group5_0.data[0]);

void _ui_init_shooter1_group5_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group5_0.data[i].figure_name[0] = FRAME_ID;
        ui_shooter1_group5_0.data[i].figure_name[1] = GROUP_ID;
        ui_shooter1_group5_0.data[i].figure_name[2] = i + START_ID;
        ui_shooter1_group5_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_shooter1_group5_0.data[i].operate_tpyel = 0;
    }

    ui_shooter1_group5_electricity->figure_tpye = 0;
    ui_shooter1_group5_electricity->layer = 0;
    ui_shooter1_group5_electricity->start_x = 1559;
    ui_shooter1_group5_electricity->start_y = 340;
    ui_shooter1_group5_electricity->end_x = 1559;
    ui_shooter1_group5_electricity->end_y = 742;
    ui_shooter1_group5_electricity->color = 2;
    ui_shooter1_group5_electricity->width = 7;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group5_0, sizeof(ui_shooter1_group5_0));
}

void _ui_update_shooter1_group5_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group5_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group5_0, sizeof(ui_shooter1_group5_0));
}

void _ui_remove_shooter1_group5_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group5_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group5_0, sizeof(ui_shooter1_group5_0));
}
