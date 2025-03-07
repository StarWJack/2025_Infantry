//
// Created by RM UI Designer
//

#include "ui_shooter1_group2_0.h"

#define FRAME_ID 1
#define GROUP_ID 3
#define START_ID 0
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_shooter1_group2_0;
ui_interface_rect_t *ui_shooter1_group2_fric_point = (ui_interface_rect_t *)&(ui_shooter1_group2_0.data[0]);
ui_interface_rect_t *ui_shooter1_group2_auto_point = (ui_interface_rect_t *)&(ui_shooter1_group2_0.data[1]);

void _ui_init_shooter1_group2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group2_0.data[i].figure_name[0] = FRAME_ID;
        ui_shooter1_group2_0.data[i].figure_name[1] = GROUP_ID;
        ui_shooter1_group2_0.data[i].figure_name[2] = i + START_ID;
        ui_shooter1_group2_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_shooter1_group2_0.data[i].operate_tpyel = 0;
    }

    ui_shooter1_group2_fric_point->figure_tpye = 1;
    ui_shooter1_group2_fric_point->layer = 0;
    ui_shooter1_group2_fric_point->start_x = 1443;
    ui_shooter1_group2_fric_point->start_y = 735;
    ui_shooter1_group2_fric_point->color = 8;
    ui_shooter1_group2_fric_point->width = 12;
    ui_shooter1_group2_fric_point->end_x = 1454;
    ui_shooter1_group2_fric_point->end_y = 746;

    ui_shooter1_group2_auto_point->figure_tpye = 1;
    ui_shooter1_group2_auto_point->layer = 0;
    ui_shooter1_group2_auto_point->start_x = 1443;
    ui_shooter1_group2_auto_point->start_y = 674;
    ui_shooter1_group2_auto_point->color = 8;
    ui_shooter1_group2_auto_point->width = 12;
    ui_shooter1_group2_auto_point->end_x = 1454;
    ui_shooter1_group2_auto_point->end_y = 685;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group2_0, sizeof(ui_shooter1_group2_0));
}

void _ui_update_shooter1_group2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group2_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group2_0, sizeof(ui_shooter1_group2_0));
}

void _ui_remove_shooter1_group2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group2_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group2_0, sizeof(ui_shooter1_group2_0));
}
