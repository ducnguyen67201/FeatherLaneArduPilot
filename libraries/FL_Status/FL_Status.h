/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdint.h>

// L08 observer only: no command, controller or motor interfaces.
class FL_Status {
public:
    // Called at a nominal 1 Hz; true requests the first/every fifth report.
    bool update(uint8_t mode, bool armed);

    uint32_t update_count() const { return _update_count; }
    uint8_t mode() const { return _mode; }
    bool armed() const { return _armed; }

private:
    uint32_t _update_count = 0;
    uint8_t _mode = 0;
    bool _armed = false;
    uint8_t _report_countdown = 0;
};
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
