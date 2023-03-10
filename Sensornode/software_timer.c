#include "software_timer.h"

void set_timer(int duration) {
    timer_count = duration;
    timer_flag = 0;
}

void run_timer() {
    if (timer_count > 0) {
        timer_count--;
        if (timer_count <= 0) {
            timer_flag = 1;
        }
    }
}