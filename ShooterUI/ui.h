
#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "ui_interface.h"

#include "ui_shooter1_group1_0.h"
#include "ui_shooter1_group1_1.h"
#include "ui_shooter1_group1_2.h"

#define ui_init_shooter1_group1() \
_ui_init_shooter1_group1_0(); \
osDelay(100);\
_ui_init_shooter1_group1_1(); \
osDelay(100);\
_ui_init_shooter1_group1_2();\
osDelay(100)

#define ui_update_shooter1_group1() \
_ui_update_shooter1_group1_0(); \
_ui_update_shooter1_group1_1(); \
_ui_update_shooter1_group1_2()

#define ui_remove_shooter1_group1() \
_ui_remove_shooter1_group1_0(); \
_ui_remove_shooter1_group1_1(); \
_ui_remove_shooter1_group1_2()
    

#include "ui_shooter1_group3_0.h"

#define ui_init_shooter1_group3() \
_ui_init_shooter1_group3_0()

#define ui_update_shooter1_group3() \
_ui_update_shooter1_group3_0()

#define ui_remove_shooter1_group3() \
_ui_remove_shooter1_group3_0()
    

#include "ui_shooter1_group5_0.h"

#define ui_init_shooter1_group5() \
_ui_init_shooter1_group5_0()

#define ui_update_shooter1_group5() \
_ui_update_shooter1_group5_0()

#define ui_remove_shooter1_group5() \
_ui_remove_shooter1_group5_0()
    

#include "ui_shooter1_group2_0.h"

#define ui_init_shooter1_group2() \
_ui_init_shooter1_group2_0()

#define ui_update_shooter1_group2() \
_ui_update_shooter1_group2_0()

#define ui_remove_shooter1_group2() \
_ui_remove_shooter1_group2_0()
    

void UI_Init(void);
void UITask (void *argument);

void ui_reinit(void);
void gimbal_direction(float delta);
void capasitor_level(uint8_t level);
void auto_shift(uint8_t status);
void fric_shift(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif //UI_H
