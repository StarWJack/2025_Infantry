//未做适配，后期估计将该部分移植至上位机


//#include "choose_armor.h"
//#include "math.h"
//#include "Serial.h"
//
//uint8_t armor_choose = 1;
//uint8_t suggest_fire = 1;
//float predict_time = 500;
//
//float get_delta_ang_pi(float a, float b)
//{
//    float delta = a - b;
//    if (delta > PI)
//        delta -= 2 * PI;
//    else if (delta < -PI)
//        delta += 2 * PI;
//    return delta;
//}
//
//
//int sign(float x)
//{
//
//    return (x > 0) - (x < 0);
//
//}
//
//
//void expected_preview_calc(void)
//{
//    float armor_x = 0.f, armor_y = 0.f, armor_z = 0.f;
//
//    uint8_t target = aim_packet_from_nuc . target;
//    uint8_t id_num = aim_packet_from_nuc . id_num;
//    float yaw = aim_packet_from_nuc . yaw;
//    float r1 = aim_packet_from_nuc . r1;
//    float r2 = aim_packet_from_nuc . r2;
//    float xc = aim_packet_from_nuc . x;
//    float yc = aim_packet_from_nuc . y;
//    float zc = aim_packet_from_nuc . z;
//    float vx = aim_packet_from_nuc . vx;
//    float vy = aim_packet_from_nuc . vy;
//    float vz = aim_packet_from_nuc . vz;
//    float vyaw = aim_packet_from_nuc . v_yaw;
//    float dz = aim_packet_from_nuc . dz;
//    uint8_t armor_num = aim_packet_to_nuc . armor_num;
//    uint32_t r_timestamp = aim_packet_from_nuc . r_timestamp;
//    uint16_t first_phase_1 = aim_packet_from_nuc . first_phase;
//    float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;
//
//    // aim_status judge
//    if (target == 1) {
//        // Prediction
//        // predict_time --> shoot delay
//        // (now_timestamp - r_timestamp) --> vision calculate using time
//        float predict_time_final =
//                (float) (now_timestamp - r_timestamp) / 1000 + predict_time;
//
//        xc = xc + predict_time_final * vx;
//        yc = yc + predict_time_final * vy;
//        zc = zc + predict_time_final * vz;
//        float predict_yaw = yaw + predict_time_final * vyaw;
//        float center_theta = atan2(yc, xc);
//
//        uint8_t use_1 = 1;
//        float diff_angle = 2 * PI / armor_num;
//
//        for (size_t i = 0; i < armor_num; i++) {
//            float armor_yaw = predict_yaw + i * diff_angle;
//            float yaw_diff = get_delta_ang_pi(armor_yaw, center_theta);
//            if (fabsf(yaw_diff) < diff_angle / 2) {
//                armor_choose = 1;
//
//                // Only 4 armors has 2 radius and height
//                float r = r1;
//                armor_z = zc;
//                if (armor_num == 4) {
//                    r = use_1 ? r1 : r2;
//                    armor_z = use_1 ? zc : (zc + dz);
//                }
//
//                // Robot state to armor
//                armor_x = xc - r * cos(armor_yaw);
//                armor_y = yc - r * sin(armor_yaw);
//
//                // Calculate angle of advance
//
//                float armor_z_next = zc;
//                if (armor_num == 4) {
//                    r = !use_1 ? r1 : r2;
//                    armor_z_next = !use_1 ? zc : (zc + dz);
//                }
//                float next_armor_yaw =
//                        armor_yaw -
//                        sign(vyaw) * diff_angle;  // TODO: use future span to calculate
//                // target delta angle
//                float armor_x_next = xc - r * cos(next_armor_yaw);
//                float armor_y_next = yc - r * sin(next_armor_yaw);
//
//                float yaw_motor_delta =
//                        get_delta_ang_pi(atan2(armor_y, armor_x),
//                                         atan2(armor_y_next, armor_x_next));
//                float angle_of_advance =
//                        fabsf(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * fabsf(vyaw) / 2;
//
//                float est_yaw;
//                float est_x;
//                float est_y;
//                float est_z;
//                if (sign(vyaw) * yaw_diff < diff_angle / 2 - angle_of_advance ||
//                    angle_of_advance > diff_angle / 4) {
//                    est_x = armor_x;
//                    est_y = armor_y;
//                    est_z = armor_z;
//                    est_yaw = armor_yaw;
//                } else {
//                    est_x = armor_x_next;
//                    est_y = armor_y_next;
//                    est_z = armor_z_next;
//                    est_yaw = next_armor_yaw;
//                }
//
//                // Calculate fire control params
//                float armor_w;
//                if (armor_num == 2 || id_num == 1)
//                    armor_w = LARGE_ARMOR_WIDTH;
//                else
//                    armor_w = SMALL_ARMOR_WIDTH;
//                float ax = est_x - 0.5f * armor_w * sin(est_yaw);
//                float ay = est_y + 0.5f * armor_w * cos(est_yaw);
//                float bx = est_x + 0.5f * armor_w * sin(est_yaw);
//                float by = est_y - 0.5f * armor_w * cos(est_yaw);
//                float angle_a = atan2(ay, ax);
//                float angle_b = atan2(by, bx);
//                float angle_c = atan2(est_y, est_x);
//                allow_fire_ang_max = angle_c - angle_b;
//                allow_fire_ang_min = angle_c - angle_a;
//
//                break;
//            } else {
//                armor_choose = 0;
//            }
//            use_1 = !use_1;
//        }
//    }
//    fire_ctrl(allow_fire_ang_max, allow_fire_ang_min, target);
//}
//
//void fire_ctrl(float allow_fire_ang_max, float allow_fire_ang_min,
//               uint8_t target)
//{
//    if (target == 0 || armor_choose == 0) {
//        suggest_fire = 0;
//    } else if (target == 1) {
//        // yaw_ang_ref
//        float control_delta_angle = 0;
////                get_delta_ang_pi(motor / 4096, _imu_eular . yaw / 4096);
//        suggest_fire = (control_delta_angle < allow_fire_ang_max &&
//                        control_delta_angle > allow_fire_ang_min);
//    } else {
//        // ERROR
//        suggest_fire = 0;
//    }
//}