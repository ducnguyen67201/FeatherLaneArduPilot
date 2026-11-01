/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "FL_TaskAdmission.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

// version and bounds for the admission-only coordinate task contract
static constexpr float SUPPORTED_TASK_VERSION = 1.0f;
static constexpr uint16_t MIN_TASK_ID = 1;
static constexpr uint16_t MAX_TASK_ID = 65535;
static constexpr float MIN_HEIGHT_M = 0.01f;
static constexpr float MAX_HEIGHT_M = 10.0f;
static constexpr float MAX_SPEED_MPS = 2.0f;
static constexpr float MIN_ARRIVAL_RADIUS_M = 0.5f;
static constexpr float MAX_ARRIVAL_RADIUS_M = 2.0f;
static constexpr float MAX_HOME_DISTANCE_M = 20.0f;

// validate and retain a task without changing flight state
FL_TaskAdmission::Result FL_TaskAdmission::admit(const Request &request, const Context &context)
{
    // check source and vehicle prerequisites supplied by the Copter receiver
    if (!context.source_allowed) {
        return Result::SOURCE;
    }

    if (!context.vehicle_allowed) {
        return Result::VEHICLE;
    }

    // protocol versions require an exact match, not a measurement tolerance
    if (!isfinite(request.version) ||
        request.version < SUPPORTED_TASK_VERSION || request.version > SUPPORTED_TASK_VERSION) {
        return Result::VERSION;
    }

    // the wire ID is a float; reject fractions before converting to uint16_t
    if (!isfinite(request.task_id) ||
        request.task_id < MIN_TASK_ID || request.task_id > MAX_TASK_ID ||
        !is_equal(request.task_id, floorf(request.task_id))) {
        return Result::ID;
    }

    // require WGS84 latitude/longitude with height measured above home
    if (request.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
        return Result::FRAME;
    }

    // this admission-only contract requires both command flags to be zero
    if (request.current != 0 || request.autocontinue != 0) {
        return Result::FLAGS;
    }

    // latitude and longitude are signed degrees scaled by 1e7
    if (!check_latlng(request.latitude_e7, request.longitude_e7)) {
        return Result::COORDINATE;
    }

    // reject heights that exceed the bound or could round to zero in Location
    if (!isfinite(request.height_m) ||
        request.height_m < MIN_HEIGHT_M || request.height_m > MAX_HEIGHT_M) {
        return Result::HEIGHT;
    }

    // validate the requested horizontal speed limit without applying it
    if (!isfinite(request.speed_mps) || !is_positive(request.speed_mps) || request.speed_mps > MAX_SPEED_MPS) {
        return Result::SPEED;
    }

    // keep the requested arrival radius inside the supported bounds
    if (!isfinite(request.radius_m) ||
        request.radius_m < MIN_ARRIVAL_RADIUS_M || request.radius_m > MAX_ARRIVAL_RADIUS_M) {
        return Result::RADIUS;
    }

    // an above-home goal needs a known home and usable absolute navigation
    if (!context.home_valid) {
        return Result::HOME;
    }

    if (!context.navigation_valid) {
        return Result::NAVIGATION;
    }

    // convert the validated height from metres to Location centimetres
    const Location goal(request.latitude_e7, request.longitude_e7,
                        int32_t(lroundf(request.height_m * 100)),
                        Location::AltFrame::ABOVE_HOME);

    // bound horizontal distance from the current home for this learning slice
    if (goal.get_distance(context.home) > MAX_HOME_DISTANCE_M) {
        return Result::RANGE;
    }

    // retain one task; a different ID cannot replace the occupied slot
    const uint16_t id = uint16_t(request.task_id);
    if (_has_task) {
        if (id != _task.id) {
            return Result::BUSY;
        }

        // acknowledge retries when the centimetre-resolution goal and limits match
        if (goal.lat == _task.goal.lat && goal.lng == _task.goal.lng && goal.alt == _task.goal.alt &&
            is_equal(request.speed_mps, _task.speed_mps) &&
            is_equal(request.radius_m, _task.radius_m)) {
            return Result::DUPLICATE;
        }

        // an existing ID cannot be reused to change the admitted task
        return Result::ID_CONFLICT;
    }

    // retain the home reference too; execution must revalidate it later
    _task = {id, goal, request.speed_mps, request.radius_m, context.home};
    _has_task = true;
    return Result::ACCEPTED;
}

// short reason strings for MAVLink status reporting
const char *FL_TaskAdmission::reason(Result result)
{
    switch (result) {
    case Result::ACCEPTED: return "accepted";
    case Result::DUPLICATE: return "duplicate";
    case Result::SOURCE: return "source";
    case Result::VEHICLE: return "vehicle";
    case Result::VERSION: return "version";
    case Result::ID: return "id";
    case Result::FRAME: return "frame";
    case Result::FLAGS: return "flags";
    case Result::COORDINATE: return "coordinate";
    case Result::HEIGHT: return "height";
    case Result::SPEED: return "speed";
    case Result::RADIUS: return "radius";
    case Result::HOME: return "home";
    case Result::NAVIGATION: return "navigation";
    case Result::RANGE: return "range";
    case Result::ID_CONFLICT: return "id_conflict";
    case Result::BUSY: return "busy";
    }
    return "unknown";
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
