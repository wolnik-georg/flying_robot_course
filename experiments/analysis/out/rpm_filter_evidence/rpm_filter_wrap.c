#include "/home/georg/Desktop/flying_robot_course/flying_drone_stack/firmware_app/rpm_filter.h"
uint16_t step(uint16_t r, uint8_t s, uint16_t *p){return rpm_filter_step(r,s,p);}
