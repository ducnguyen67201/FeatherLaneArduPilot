/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "FL_Status.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
bool FL_Status::update(uint8_t mode, bool armed)
{
    _mode = mode;
    _armed = armed;
    ++_update_count;

    if (_report_countdown > 0) {
        --_report_countdown;
        return false;
    }

    _report_countdown = 4;
    return true;
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
