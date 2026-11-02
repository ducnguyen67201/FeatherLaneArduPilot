/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "Copter.h"

#if MODE_FEATHERLANE_ENABLED

#if AP_RANGEFINDER_ENABLED
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#endif

static constexpr float MAX_REFERENCE_SHIFT_M = 0.5f;
static constexpr float MAX_ENTRY_SPEED_MPS = 0.3f;
static constexpr float MAX_ENTRY_HEIGHT_ERROR_M = 0.5f;
static constexpr float MAX_EXECUTION_DISTANCE_M = 20.0f;
static constexpr float CM_TO_METRES = 0.01f;
static constexpr float MAX_ARRIVAL_SPEED_MPS = 0.3f;
static constexpr float MAX_ARRIVAL_HEIGHT_ERROR_M = 0.2f;
static constexpr float MAX_ARRIVAL_VERTICAL_SPEED_MPS = 0.2f;
static constexpr uint32_t ARRIVAL_DWELL_MS = 2000;
static constexpr uint32_t MAX_ARRIVAL_UPDATE_GAP_MS = 250;
static constexpr float MAX_AVOIDANCE_SPEED_MPS = 0.5f;
static constexpr float MAX_AVOIDANCE_RADIUS_M = 0.5f;
static constexpr float MAX_AVOIDANCE_DISTANCE_M = 6.0f;
static constexpr uint32_t MAX_TASK_EXECUTION_MS = 30000;

#if AP_AVOIDANCE_ENABLED && HAL_PROXIMITY_ENABLED && AP_RANGEFINDER_ENABLED && \
    AP_PROXIMITY_RANGEFINDER_ENABLED && AP_RANGEFINDER_MAVLINK_ENABLED
static constexpr float FRONT_RANGE_MIN_M = 0.2f;
static constexpr float FRONT_RANGE_MAX_M = 8.0f;
static constexpr float MAX_FRONT_PATH_ANGLE_DEG = 3.0f;
static constexpr float MAX_SENSOR_TILT_DEG = 5.0f;
static constexpr float MIN_GOAL_DIRECTION_DISTANCE_M = 0.1f;
static constexpr float MIN_AVOIDANCE_CHANGE_MPS = 0.01f;
static constexpr uint32_t MAX_OBSERVATION_AGE_MS = 350;
#endif

const char *ModeFeatherLane::task_state_name() const
{
    switch (_task_state) {
    case TaskState::IDLE:
        return "idle";
    case TaskState::RUNNING:
        return "running";
    case TaskState::DWELLING:
        return "dwelling";
    case TaskState::COMPLETE:
        return "complete";
    case TaskState::BLOCKED:
        return "blocked";
    case TaskState::ABORTED:
        return "aborted";
    case TaskState::INTERRUPTED:
        return "interrupted";
    case TaskState::FAILED:
        return "failed";
    }
    return "unknown";
}

// report transitions once, using the same boot clock as the native dwell
void ModeFeatherLane::set_task_state(TaskState state, const char *reason)
{
    if (_task_state == state) {
        return;
    }
    _task_state = state;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FL L11 t=%u s=%s r=%s ms=%lu",
                  unsigned(_task_id), task_state_name(), reason, (unsigned long)AP_HAL::millis());
}

bool ModeFeatherLane::reject_start(const char *reason)
{
    _start_failure = reason;
    return false;
}

// only the explicit task-start receiver grants the mode-entry latch
bool ModeFeatherLane::start_task(uint16_t task_id)
{
    _start_failure = nullptr;
    if (_executing && copter.flightmode == this && task_id == _task_id) {
        return true;
    }

    if (_consumed) {
        return reject_start("used");
    }

    _task_id = task_id;
    _entry_requested = true;
    const bool started = copter.set_mode(Number::FEATHERLANE, ModeReason::GCS_COMMAND);
    _entry_requested = false;
    if (!started && _start_failure == nullptr) {
        return reject_start("mode_change");
    }
    return started;
}

// successful stock BRAKE entry calls exit(); only that path records operator abort
bool ModeFeatherLane::abort_task(uint16_t task_id)
{
    _abort_failure = nullptr;
    if (task_id != _task_id) {
        _abort_failure = "task";
        return false;
    }
    if (_task_state == TaskState::ABORTED) {
        return true;
    }
    if (!_executing || copter.flightmode != this ||
        (_task_state != TaskState::RUNNING && _task_state != TaskState::DWELLING)) {
        _abort_failure = "not_running";
        return false;
    }
    if (!navigation_valid() || !reference_valid()) {
        _abort_failure = "navigation";
        return false;
    }

#if MODE_BRAKE_ENABLED
    _abort_requested = true;
    const bool changed_mode = copter.set_mode(Number::BRAKE, ModeReason::GCS_COMMAND);
    _abort_requested = false;
    if (changed_mode) {
        return true;
    }
    _abort_failure = "brake_entry";
#else
    _abort_failure = "brake_disabled";
#endif
    return false;
}

bool ModeFeatherLane::navigation_valid() const
{
    return copter.ahrs.home_is_set() && copter.ekf_has_absolute_position() &&
           copter.ekf_alt_ok() && !copter.any_failsafe_triggered();
}

// revalidate current authority and references before touching shared controllers
bool ModeFeatherLane::init(bool ignore_checks)
{
    if (!_entry_requested) {
        return reject_start("start_required");
    }

    const auto *task = copter.featherlane_task.task();
    if (task == nullptr || task->id != _task_id) {
        return reject_start("task");
    }

    if (!motors->armed() || copter.ap.land_complete || copter.flightmode != &copter.mode_guided) {
        return reject_start("airborne_guided");
    }

    if (!navigation_valid()) {
        return reject_start("navigation");
    }

    // qualify one position-controller path; other Guided options need their own slice
    if (g2.guided_options.get() != 0) {
        return reject_start("guided_options");
    }

    Vector3f velocity_ned_ms;
    if (!ahrs.get_velocity_NED(velocity_ned_ms) || velocity_ned_ms.length() > MAX_ENTRY_SPEED_MPS) {
        return reject_start("entry_speed");
    }

    const Location current_home = ahrs.get_home();
    if (current_home.get_distance(task->admitted_home) > MAX_REFERENCE_SHIFT_M ||
        fabsf(float(current_home.alt) - float(task->admitted_home.alt)) * CM_TO_METRES > MAX_REFERENCE_SHIFT_M) {
        return reject_start("home_changed");
    }

    // preserve the admitted above-home altitude even if arming slightly updates home
    const Location destination(task->goal.lat, task->goal.lng,
                               task->admitted_home.alt + task->goal.alt,
                               Location::AltFrame::ABSOLUTE);
    Location current_location;
    if (!ahrs.get_location(current_location) ||
        !current_location.change_alt_frame(Location::AltFrame::ABSOLUTE) ||
        !ahrs.get_origin(_entry_origin)) {
        return reject_start("reference");
    }

    if (destination.get_distance(current_location) > MAX_EXECUTION_DISTANCE_M ||
        fabsf(float(destination.alt) - float(current_location.alt)) * CM_TO_METRES > MAX_ENTRY_HEIGHT_ERROR_M) {
        return reject_start("entry_distance");
    }

    // this slice qualifies one slow, forward corridor and a small arrival region
    if (task->speed_mps > MAX_AVOIDANCE_SPEED_MPS || task->radius_m > MAX_AVOIDANCE_RADIUS_M ||
        destination.get_distance(current_location) > MAX_AVOIDANCE_DISTANCE_M) {
        return reject_start("avoidance_limits");
    }

    _destination = destination;
    _speed_limit_mps = task->speed_mps;
    const AvoidanceResult avoidance = check_avoidance();
    if (avoidance != AvoidanceResult::CLEAR) {
        return reject_start(avoidance == AvoidanceResult::BLOCKED ? "obstacle" : _avoidance_failure);
    }

#if !MODE_BRAKE_ENABLED
    return reject_start("brake_disabled");
#endif

    Vector3p destination_ned_m;
    bool terrain_alt;
    if (!wp_nav->get_vector_NED_m(destination, destination_ned_m, terrain_alt) || terrain_alt) {
        return reject_start("conversion");
    }

#if AP_FENCE_ENABLED
    if (!copter.fence.check_destination_within_fence(destination)) {
        return reject_start("fence");
    }
#endif

    // initialise inherited controllers, then install the native owner's stored goal
    if (!ModeGuided::init(ignore_checks) || !set_destination(destination)) {
        copter.mode_guided.hold_position();
        return reject_start("destination");
    }

    // set_destination initialises speed defaults, so apply the retained limit afterwards
    ModeGuided::set_speed_NE_ms(task->speed_mps);
    _entry_home = current_home;
    Vector2f position_reset_m;
    float height_reset_m;
    _position_reset_ms = ahrs.getLastPosNorthEastReset(position_reset_m);
    _height_reset_ms = ahrs.getLastPosDownReset(height_reset_m);
    _executing = true;
    _consumed = true;
    _destination = destination;
    _arrival_radius_m = task->radius_m;
    _last_arrival_check_ms = AP_HAL::millis();
    _execution_start_ms = _last_arrival_check_ms;
    set_task_state(TaskState::RUNNING, "start");

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FL L10 goal=%ld,%ld alt_cm=%ld",
                  (long)destination.lat, (long)destination.lng, (long)destination.alt);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FL L10 speed=%.2fm/s task=%u", double(task->speed_mps), unsigned(_task_id));
    return true;
}

// the global goal is converted once; stop using it if the estimator reference resets
bool ModeFeatherLane::reference_valid()
{
    Location origin;
    if (!ahrs.get_origin(origin) || origin.lat != _entry_origin.lat || origin.lng != _entry_origin.lng ||
        fabsf(float(origin.alt) - float(_entry_origin.alt)) * CM_TO_METRES > MAX_REFERENCE_SHIFT_M) {
        return false;
    }

    const Location home = ahrs.get_home();
    if (home.get_distance(_entry_home) > MAX_REFERENCE_SHIFT_M ||
        fabsf(float(home.alt) - float(_entry_home.alt)) * CM_TO_METRES > MAX_REFERENCE_SHIFT_M) {
        return false;
    }

    Vector2f position_reset_m;
    float height_reset_m;
    return ahrs.getLastPosNorthEastReset(position_reset_m) == _position_reset_ms &&
           ahrs.getLastPosDownReset(height_reset_m) == _height_reset_ms;
}

// all bounds must remain satisfied across a continuous series of native updates
bool ModeFeatherLane::update_arrival()
{
    if (_task_state == TaskState::COMPLETE) {
        return true;
    }

    Location current_location;
    Vector3f velocity_ned_ms;
    if (!ahrs.get_location(current_location) ||
        !current_location.change_alt_frame(Location::AltFrame::ABSOLUTE) ||
        !ahrs.get_velocity_NED(velocity_ned_ms) ||
        !isfinite(velocity_ned_ms.x) || !isfinite(velocity_ned_ms.y) || !isfinite(velocity_ned_ms.z)) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const bool update_gap = now_ms - _last_arrival_check_ms > MAX_ARRIVAL_UPDATE_GAP_MS;
    _last_arrival_check_ms = now_ms;
    const bool within_bounds = current_location.get_distance(_destination) <= _arrival_radius_m &&
        velocity_ned_ms.xy().length() <= MAX_ARRIVAL_SPEED_MPS &&
        fabsf(float(current_location.alt) - float(_destination.alt)) * CM_TO_METRES <= MAX_ARRIVAL_HEIGHT_ERROR_M &&
        fabsf(velocity_ned_ms.z) <= MAX_ARRIVAL_VERTICAL_SPEED_MPS;

    if (update_gap || !within_bounds) {
        set_task_state(TaskState::RUNNING, update_gap ? "gap" : "bounds");
        return true;
    }
    if (_task_state != TaskState::DWELLING) {
        _dwell_start_ms = now_ms;
        set_task_state(TaskState::DWELLING, "bounds");
    }
    if (now_ms - _dwell_start_ms >= ARRIVAL_DWELL_MS) {
        set_task_state(TaskState::COMPLETE, "dwell");
    }
    return true;
}

// use the qualified stock limiter as a veto; it does not replace goal tracking
ModeFeatherLane::AvoidanceResult ModeFeatherLane::check_avoidance()
{
    _avoidance_failure = "avoidance_disabled";
#if AP_AVOIDANCE_ENABLED && HAL_PROXIMITY_ENABLED && AP_RANGEFINDER_ENABLED && \
    AP_PROXIMITY_RANGEFINDER_ENABLED && AP_RANGEFINDER_MAVLINK_ENABLED
    const AP_Proximity *proximity = AP::proximity();
    const auto *sensor = copter.rangefinder.get_backend(0);
    _avoidance_failure = "sensor_profile";
    if (sensor == nullptr || copter.rangefinder.num_sensors() != 1 ||
        sensor->allocated_type() != RangeFinder::Type::MAVLink ||
        sensor->type() != RangeFinder::Type::MAVLink || sensor->orientation() != ROTATION_NONE ||
        !is_equal(sensor->min_distance(), FRONT_RANGE_MIN_M) ||
        !is_equal(sensor->max_distance(), FRONT_RANGE_MAX_M) ||
        proximity == nullptr || proximity->num_sensors() != 1 ||
        proximity->get_type(0) != AP_Proximity::Type::RangeFinder ||
        !copter.avoid.proximity_avoidance_enabled() || !is_zero(copter.avoid.get_min_alt())) {
        return AvoidanceResult::INVALID;
    }

    // age the actual backend receive time, not the simulator's wire timestamp
    _avoidance_failure = "observation";
    const uint32_t last_reading_ms = sensor->last_reading_ms();
    const uint32_t age_ms = AP_HAL::millis() - last_reading_ms;
    if (last_reading_ms == 0 || age_ms > MAX_OBSERVATION_AGE_MS ||
        proximity->get_instance_status(0) != AP_Proximity::Status::Good ||
        (sensor->status() != RangeFinder::Status::Good &&
         sensor->status() != RangeFinder::Status::OutOfRangeHigh) || !isfinite(sensor->distance())) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FL L15 observation age_ms=%lu", (unsigned long)age_ms);
        return AvoidanceResult::INVALID;
    }

    // no-return remains unknown; only this prequalified narrow corridor is supported
    _avoidance_failure = "coverage";
    if (!isfinite(ahrs.get_roll_rad()) || !isfinite(ahrs.get_pitch_rad()) || !isfinite(ahrs.get_yaw_rad()) ||
        fabsf(ahrs.get_roll_rad()) > radians(MAX_SENSOR_TILT_DEG) ||
        fabsf(ahrs.get_pitch_rad()) > radians(MAX_SENSOR_TILT_DEG)) {
        return AvoidanceResult::INVALID;
    }

    Location current_location;
    _avoidance_failure = "measurement";
    if (!ahrs.get_location(current_location)) {
        return AvoidanceResult::INVALID;
    }
    const Vector2f goal_offset_ne_m = current_location.get_distance_NE(_destination);
    const float distance_m = goal_offset_ne_m.length();
    if (!isfinite(distance_m)) {
        return AvoidanceResult::INVALID;
    }
    if (distance_m <= MIN_GOAL_DIRECTION_DISTANCE_M) {
        return AvoidanceResult::CLEAR;
    }

    _avoidance_failure = "coverage";
    const Vector2f goal_offset_body_m = ahrs.earth_to_body2D(goal_offset_ne_m);
    if (fabsf(atan2f(goal_offset_body_m.y, goal_offset_body_m.x)) > radians(MAX_FRONT_PATH_ANGLE_DEG)) {
        return AvoidanceResult::INVALID;
    }

    const Vector3f requested_velocity_ned_ms{goal_offset_ne_m.x * _speed_limit_mps / distance_m,
                                           goal_offset_ne_m.y * _speed_limit_mps / distance_m, 0.0f};
    Vector3f limited_velocity_ned_ms = requested_velocity_ned_ms;
    copter.avoid.adjust_velocity_NED_m(limited_velocity_ned_ms,
                                     pos_control->NE_get_pos_p().kP(), pos_control->NE_get_max_accel_mss(),
                                     pos_control->D_get_pos_p().kP(), pos_control->D_get_max_accel_mss(), G_Dt);
    _avoidance_failure = nullptr;
    return (limited_velocity_ned_ms - requested_velocity_ned_ms).length() > MIN_AVOIDANCE_CHANGE_MPS ?
        AvoidanceResult::BLOCKED : AvoidanceResult::CLEAR;
#else
    return AvoidanceResult::INVALID;
#endif
}

// only successful stock mode entry records the requested blocked/failed result
void ModeFeatherLane::stop_task(TaskState state, const char *reason)
{
#if MODE_BRAKE_ENABLED
    _stop_state = state;
    _stop_reason = reason;
    _stop_requested = true;
    const bool changed_mode = copter.set_mode(Number::BRAKE, ModeReason::FAILSAFE);
    _stop_requested = false;
    if (changed_mode) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FL L15 handoff r=%s ms=%lu",
                      reason, (unsigned long)AP_HAL::millis());
        return;
    }
#endif
    fail_task("brake_entry");
}

void ModeFeatherLane::fail_task(const char *reason)
{
    _fallback_requested = true;
    if (_task_state == TaskState::COMPLETE) {
        // completed task state and subsequent flight safety remain separate
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FL L11 guard r=%s", reason);
    } else {
        set_task_state(TaskState::FAILED, reason);
    }
    copter.set_mode_land_with_pause(ModeReason::FAILSAFE);
}

void ModeFeatherLane::run()
{
    // latch a failed execution's fallback even if an estimator flag later recovers
    if (_fallback_requested) {
        copter.set_mode_land_with_pause(ModeReason::FAILSAFE);
        return;
    }

    if (!navigation_valid()) {
        fail_task("navigation");
        return;
    }
    if (!reference_valid()) {
        fail_task("reference");
        return;
    }

    const AvoidanceResult avoidance = check_avoidance();
    if (avoidance == AvoidanceResult::INVALID) {
        stop_task(TaskState::FAILED, _avoidance_failure);
        return;
    }
    if (avoidance == AvoidanceResult::BLOCKED) {
        stop_task(TaskState::BLOCKED, "obstacle");
        return;
    }
    if (_task_state != TaskState::COMPLETE &&
        AP_HAL::millis() - _execution_start_ms >= MAX_TASK_EXECUTION_MS) {
        stop_task(TaskState::FAILED, "timeout");
        return;
    }
    if (!update_arrival()) {
        fail_task("measurement");
        return;
    }

    ModeGuided::run();
}

// do not alter shared controller state here: the next mode has already initialised
void ModeFeatherLane::exit()
{
    _executing = false;
    if (_task_state == TaskState::RUNNING || _task_state == TaskState::DWELLING) {
        if (_stop_requested) {
            set_task_state(_stop_state, _stop_reason);
        } else {
            set_task_state(_abort_requested ? TaskState::ABORTED : TaskState::INTERRUPTED,
                           _abort_requested ? "operator" : "mode_exit");
        }
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FL L10 exit task=%u", unsigned(_task_id));
}

#endif // MODE_FEATHERLANE_ENABLED
