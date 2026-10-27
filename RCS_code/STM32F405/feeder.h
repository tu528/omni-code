#pragma once
#include <stdint.h>

// Fixed seven-point geometry for single shots. No homing or entry movement.
class FeederController {
public:
    enum State { IDLE, RUN, SETTLE };
    enum { COUNTS_PER_DISK = 8192 * 36, POINTS = 7, STOP_TOLERANCE = 200 };
    struct Input {
        uint32_t now = 0;
        uint16_t angle = 0;
        int rpm = 0;
        uint8_t session = 0;
        bool feedback_available = false, enabled = false, request = false;
    };
    volatile State state = IDLE;
    volatile uint32_t shoot_now_count = 0; // 上电以来累计开始拨弹次数，不代表确认出弹数
    int speed = 0;
    int64_t position = 0, base = 0, target = 0;
    int64_t points[POINTS]{};
    int64_t slot_index = 0, target_index = 0;
    bool points_ready = false;
    static int64_t Point(int64_t index) {
        int64_t cycle = index / POINTS;
        int remainder = static_cast<int>(index % POINTS);
        if (remainder < 0) { --cycle; remainder += POINTS; }
        return cycle * COUNTS_PER_DISK + (remainder * COUNTS_PER_DISK + 3) / POINTS;
    }
    bool Busy() const { return state == RUN || state == SETTLE; }
    void Update(const Input& in) {
        speed = 0;
        if (in.feedback_available) {
            if (!feedback_seen) { previous_angle = in.angle; feedback_seen = true; }
            int delta = static_cast<int>(in.angle) - previous_angle;
            if (delta > 4096) delta -= 8192;
            if (delta < -4096) delta += 8192;
            position += delta; previous_angle = in.angle;
        }
        if (!in.enabled || in.session != last_session) {
            state = IDLE; last_session = in.session;
            if (!in.enabled) return; // Keep the same points across mode switches.
        }
        if (!feedback_seen) return;
        if (state == IDLE && in.request) {
            if (!points_ready) {
                base = position;
                for (int i = 0; i < POINTS; ++i) points[i] = base + Point(i);
                points_ready = true;
            }
            target_index = slot_index + 1;
            const int64_t relative = position - base;
            if (relative > 0) {
                const int64_t nearby = (relative / COUNTS_PER_DISK) * POINTS +
                    (relative % COUNTS_PER_DISK) * POINTS / COUNTS_PER_DISK;
                if (nearby > target_index) target_index = nearby;
            }
            while (base + Point(target_index) <= position + STOP_TOLERANCE) ++target_index;
            target = base + Point(target_index);
            move_start = position;
            state = RUN; ++shoot_now_count;
        }
        if (state == RUN) {
            const int64_t remaining = target - position;
            if (remaining <= STOP_TOLERANCE) {
                state = SETTLE; stopped_timing = false;
            } else {
                int64_t traveled = position - move_start;
                if (traveled < 0) traveled = 0;
                const int up = 150 + static_cast<int>(850 * (traveled < 3600 ? traveled : 3600) / 3600);
                const int down = 150 + static_cast<int>(850 * (remaining < 3000 ? remaining : 3000) / 3000);
                speed = up < down ? up : down;
                return;
            }
        }
        if (state == SETTLE) {
            if (in.rpm < -30 || in.rpm > 30) { stopped_timing = false; return; }
            if (!stopped_timing) { stopped_since = in.now; stopped_timing = true; }
            if (in.now - stopped_since >= 100) { slot_index = target_index; state = IDLE; }
        }
    }
private:
    bool feedback_seen = false, stopped_timing = false;
    uint16_t previous_angle = 0;
    uint8_t last_session = 0;
    uint32_t stopped_since = 0;
    int64_t move_start = 0;
};
