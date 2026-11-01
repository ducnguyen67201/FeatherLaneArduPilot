/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// Admission only. There is deliberately no execution/controller interface.
class FL_TaskAdmission {
public:
    struct Request {
        // COMMAND_INT carries these fields as floats; validate before conversion.
        float version;
        float task_id;
        float speed_mps;
        float radius_m;
        int32_t latitude_e7;
        int32_t longitude_e7;
        float height_m;
        uint8_t frame;
        uint8_t current;
        uint8_t autocontinue;
    };

    struct Context {
        bool source_allowed;
        bool vehicle_allowed;
        bool home_valid;
        bool navigation_valid;
        Location home;
    };

    struct Task {
        uint16_t id;
        Location goal;
        float speed_mps;
        float radius_m;
        Location admitted_home;
    };

    enum class Result : uint8_t {
        ACCEPTED, DUPLICATE, SOURCE, VEHICLE, VERSION, ID, FRAME, FLAGS,
        COORDINATE, HEIGHT, SPEED, RADIUS, HOME, NAVIGATION, RANGE,
        ID_CONFLICT, BUSY
    };

    Result admit(const Request &request, const Context &context);
    static const char *reason(Result result);
    uint16_t retained_id() const { return _has_task ? _task.id : 0; }
    const Task *task() const { return _has_task ? &_task : nullptr; }

private:
    Task _task {};
    bool _has_task = false;
};
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
