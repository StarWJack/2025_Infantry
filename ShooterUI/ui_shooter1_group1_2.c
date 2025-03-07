//
// Created by RM UI Designer
//

#include "ui_shooter1_group1_2.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 2
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_shooter1_group1_2;
ui_interface_line_t *ui_shooter1_group1_midline = (ui_interface_line_t *)&(ui_shooter1_group1_2.data[0]);
ui_interface_line_t *ui_shooter1_group1_left = (ui_interface_line_t *)&(ui_shooter1_group1_2.data[1]);
ui_interface_line_t *ui_shooter1_group1_right = (ui_interface_line_t *)&(ui_shooter1_group1_2.data[2]);
ui_interface_line_t *ui_shooter1_group1_long_horizen = (ui_interface_line_t *)&(ui_shooter1_group1_2.data[3]);
ui_interface_rect_t *ui_shooter1_group1_body_rect = (ui_interface_rect_t *)&(ui_shooter1_group1_2.data[4]);
ui_interface_rect_t *ui_shooter1_group1_capasitorRect = (ui_interface_rect_t *)&(ui_shooter1_group1_2.data[5]);
ui_interface_line_t *ui_shooter1_group1_frontLine = (ui_interface_line_t *)&(ui_shooter1_group1_2.data[6]);

void _ui_init_shooter1_group1_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group1_2.data[i].figure_name[0] = FRAME_ID;
        ui_shooter1_group1_2.data[i].figure_name[1] = GROUP_ID;
        ui_shooter1_group1_2.data[i].figure_name[2] = i + START_ID;
        ui_shooter1_group1_2.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_shooter1_group1_2.data[i].operate_tpyel = 0;
    }

    ui_shooter1_group1_midline->figure_tpye = 0;
    ui_shooter1_group1_midline->layer = 0;
    ui_shooter1_group1_midline->start_x = 960;
    ui_shooter1_group1_midline->start_y = 200;
    ui_shooter1_group1_midline->end_x = 960;
    ui_shooter1_group1_midline->end_y = 778;
    ui_shooter1_group1_midline->color = 1;
    ui_shooter1_group1_midline->width = 2;

    ui_shooter1_group1_left->figure_tpye = 0;
    ui_shooter1_group1_left->layer = 0;
    ui_shooter1_group1_left->start_x = 400;
    ui_shooter1_group1_left->start_y = 52;
    ui_shooter1_group1_left->end_x = 715;
    ui_shooter1_group1_left->end_y = 538;
    ui_shooter1_group1_left->color = 1;
    ui_shooter1_group1_left->width = 3;

    ui_shooter1_group1_right->figure_tpye = 0;
    ui_shooter1_group1_right->layer = 0;
    ui_shooter1_group1_right->start_x = 1522;
    ui_shooter1_group1_right->start_y = 52;
    ui_shooter1_group1_right->end_x = 1207;
    ui_shooter1_group1_right->end_y = 538;
    ui_shooter1_group1_right->color = 1;
    ui_shooter1_group1_right->width = 3;

    ui_shooter1_group1_long_horizen->figure_tpye = 0;
    ui_shooter1_group1_long_horizen->layer = 0;
    ui_shooter1_group1_long_horizen->start_x = 568;
    ui_shooter1_group1_long_horizen->start_y = 540;
    ui_shooter1_group1_long_horizen->end_x = 1351;
    ui_shooter1_group1_long_horizen->end_y = 540;
    ui_shooter1_group1_long_horizen->color = 1;
    ui_shooter1_group1_long_horizen->width = 2;

    ui_shooter1_group1_body_rect->figure_tpye = 1;
    ui_shooter1_group1_body_rect->layer = 0;
    ui_shooter1_group1_body_rect->start_x = 222;
    ui_shooter1_group1_body_rect->start_y = 651;
    ui_shooter1_group1_body_rect->color = 0;
    ui_shooter1_group1_body_rect->width = 3;
    ui_shooter1_group1_body_rect->end_x = 323;
    ui_shooter1_group1_body_rect->end_y = 781;

    ui_shooter1_group1_capasitorRect->figure_tpye = 1;
    ui_shooter1_group1_capasitorRect->layer = 0;
    ui_shooter1_group1_capasitorRect->start_x = 1553;
    ui_shooter1_group1_capasitorRect->start_y = 340;
    ui_shooter1_group1_capasitorRect->color = 8;
    ui_shooter1_group1_capasitorRect->width = 2;
    ui_shooter1_group1_capasitorRect->end_x = 1564;
    ui_shooter1_group1_capasitorRect->end_y = 740;

    ui_shooter1_group1_frontLine->figure_tpye = 0;
    ui_shooter1_group1_frontLine->layer = 0;
    ui_shooter1_group1_frontLine->start_x = 243;
    ui_shooter1_group1_frontLine->start_y = 791;
    ui_shooter1_group1_frontLine->end_x = 307;
    ui_shooter1_group1_frontLine->end_y = 791;
    ui_shooter1_group1_frontLine->color = 0;
    ui_shooter1_group1_frontLine->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group1_2);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group1_2, sizeof(ui_shooter1_group1_2));
}

void _ui_update_shooter1_group1_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group1_2.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group1_2);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group1_2, sizeof(ui_shooter1_group1_2));
}

void _ui_remove_shooter1_group1_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_shooter1_group1_2.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_shooter1_group1_2);
    SEND_MESSAGE((uint8_t *) &ui_shooter1_group1_2, sizeof(ui_shooter1_group1_2));
}
