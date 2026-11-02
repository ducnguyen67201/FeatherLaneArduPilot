/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "Copter.h"

#if MODE_FEATHERLANE_ENABLED

static constexpr float MAX_REFERENCE_SHIFT_M = 0.5f;
static constexpr float MAX_ENTRY_SPEED_MPS = 0.3f;
static constexpr float MAX_ENTRY_HEIGHT_ERROR_M = 0.5f;
static constexpr float MAX_EXECUTION_DISTANCE_M = 20.0f;
static constexpr float CM_TO_METRES = 0.01f;

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

void ModeFeatherLane::run()
{
    if (!navigation_valid() || !reference_valid()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FL L10 guard reference/navigation");
        copter.set_mode_land_with_pause(ModeReason::FAILSAFE);
        return;
    }

    ModeGuided::run();
}

// do not alter shared controller state here: the next mode has already initialised
void ModeFeatherLane::exit()
{
    _executing = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FL L10 exit task=%u", unsigned(_task_id));
}

#endif // MODE_FEATHERLANE_ENABLED
