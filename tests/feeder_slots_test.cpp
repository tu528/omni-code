#include "../RCS_code/STM32F405/feeder.h"
#include <cstdio>
#include <cstdlib>
#include <algorithm>

#define CHECK(x) do { if (!(x)) { std::printf("FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)
typedef FeederController F;

struct Rig {
    F f;
    F::Input in;
    int64_t actual = 0;
    int raw_start;
    explicit Rig(int raw = 7900) : raw_start(raw) {
        in.enabled = true; in.feedback_available = true; in.session = 1;
        tick();
    }
    void tick(int delta = 0, unsigned dt = 2) {
        actual += delta;
        int64_t raw = (raw_start + actual) % 8192;
        if (raw < 0) raw += 8192;
        in.angle = static_cast<uint16_t>(raw);
        in.now += dt;
        f.Update(in);
        CHECK(f.speed >= 0 && f.speed <= 1000);
    }
    void request() {
        in.request = true; tick(); in.request = false;
    }
    void arrive(int error = 0) {
        const int64_t destination = f.target + error;
        in.rpm = 300;
        while (actual < destination) {
            tick(static_cast<int>(std::min<int64_t>(100, destination - actual)));
        }
        in.rpm = 0;
        for (int i = 0; i < 60 && f.Busy(); ++i) tick();
        CHECK(f.state == F::IDLE);
    }
};

int main() {
    int scenarios = 0;
    { // Entering or changing modes never feeds automatically.
        Rig r;
        for (int mode = 1; mode <= 4; ++mode) {
            r.in.session = mode;
            for (int i = 0; i < 300; ++i) r.tick();
            CHECK(r.f.state == F::IDLE && r.f.speed == 0);
            CHECK(!r.f.points_ready && r.f.started_count == 0);
        }
        ++scenarios;
    }
    { // Encoder wrap and the raw power-on angle do not change the first step.
        const int raw_starts[] = {0, 1, 4095, 4096, 7900, 8191};
        for (unsigned i = 0; i < sizeof(raw_starts) / sizeof(raw_starts[0]); ++i) {
            Rig r(raw_starts[i]); r.tick(500); r.request();
            CHECK(r.f.base == 500 && r.f.target == 42630);
            CHECK(r.f.speed == 150 && r.f.started_count == 1);
            r.arrive();
            CHECK(r.f.slot_index == 1);
        }
        ++scenarios;
    }
    { // Actual stopping errors must never move the planned seven points.
        Rig r; r.request();
        const int64_t expected[] = {0, 42130, 84261, 126391, 168521, 210651, 252782};
        for (int i = 0; i < 7; ++i) CHECK(r.f.points[i] == expected[i]);
        for (int shot = 1; shot <= 70; ++shot) {
            if (shot > 1) r.request();
            const int64_t planned = (static_cast<int64_t>(shot) * 294912 + 3) / 7;
            CHECK(r.f.target == planned);
            r.arrive(shot % 2 ? -100 : 100);
            CHECK(r.f.slot_index == shot && r.f.started_count == static_cast<unsigned>(shot));
            CHECK(r.f.base == 0);
            for (int i = 0; i < 7; ++i) CHECK(r.f.points[i] == expected[i]);
        }
        CHECK(r.f.target == 10 * 294912);
        ++scenarios;
    }
    { // Switching modes preserves the completed lattice and does not seek.
        Rig r; r.request(); r.arrive(-100);
        for (int mode = 2; mode <= 4; ++mode) {
            r.in.session = mode; r.tick();
            CHECK(r.f.state == F::IDLE && r.f.started_count == 1);
            CHECK(r.f.base == 0 && r.f.slot_index == 1);
        }
        r.request(); CHECK(r.f.target == 84261); r.arrive();
        ++scenarios;
    }
    { // A cancelled half-shot resumes toward the same still-forward point.
        Rig r; r.request();
        for (int i = 0; i < 100; ++i) r.tick(100);
        r.in.enabled = false; r.tick();
        CHECK(r.f.state == F::IDLE && r.f.speed == 0 && r.f.slot_index == 0);
        r.in.enabled = true; r.in.session = 2; r.tick();
        CHECK(r.f.started_count == 1);
        r.request(); CHECK(r.f.target == 42130 && r.f.base == 0); r.arrive();
        ++scenarios;
    }
    { // If a cancelled shot is already at its target, only move forward.
        Rig r; r.request();
        r.in.rpm = 300;
        while (r.actual < r.f.target) r.tick(static_cast<int>(std::min<int64_t>(100, r.f.target-r.actual)));
        CHECK(r.f.state == F::SETTLE);
        r.in.enabled = false; r.tick();
        r.in.enabled = true; r.request();
        CHECK(r.f.target == 84261 && r.f.speed > 0);
        r.arrive();
        ++scenarios;
    }
    { // Requests during movement are not queued or allowed to shift a target.
        Rig r; r.request(); const int64_t target = r.f.target;
        r.request(); CHECK(r.f.started_count == 1 && r.f.target == target);
        r.arrive(); r.tick(); CHECK(r.f.started_count == 1 && r.f.state == F::IDLE);
        ++scenarios;
    }
    { // External forward movement skips stale slots without changing geometry.
        Rig r; r.request(); r.arrive(); r.in.enabled = false;
        const int64_t destination = 3 * 294912 + 1000;
        while (r.actual < destination) r.tick(static_cast<int>(std::min<int64_t>(1000,destination-r.actual)));
        r.in.enabled = true; r.request();
        CHECK(r.f.base == 0 && r.f.target == 3 * 294912 + 42130);
        r.arrive(); ++scenarios;
    }
    { // Settle waits for 100 ms of continuously low speed.
        Rig r; r.request(); r.in.rpm = 40;
        while (r.actual < r.f.target) r.tick(static_cast<int>(std::min<int64_t>(100,r.f.target-r.actual)));
        for (int i = 0; i < 100; ++i) r.tick();
        CHECK(r.f.state == F::SETTLE && r.f.slot_index == 0);
        r.in.rpm = 0; for (int i = 0; i < 40; ++i) r.tick();
        CHECK(r.f.state == F::SETTLE);
        r.in.rpm = 40; r.tick(); r.in.rpm = 0;
        for (int i = 0; i < 40; ++i) r.tick();
        CHECK(r.f.state == F::SETTLE);
        for (int i = 0; i < 20; ++i) r.tick();
        CHECK(r.f.state == F::IDLE && r.f.slot_index == 1);
        ++scenarios;
    }
    { // Forward ramp never commands reversal, including negative encoder drift.
        Rig r; r.request(); r.tick(-100);
        CHECK(r.f.speed == 150);
        for (int i = 0; i < 40; ++i) r.tick(100);
        CHECK(r.f.speed == 1000); r.arrive(); ++scenarios;
    }
    { // Missing feedback never creates a permanent lock or resets the points.
        F f; F::Input in; in.enabled = true; in.request = true; in.session = 1;
        f.Update(in); CHECK(f.started_count == 0 && f.speed == 0);
        Rig r; r.request(); r.in.feedback_available = false; r.tick();
        CHECK(r.f.state == F::RUN && r.f.speed > 0);
        r.in.feedback_available = true; r.arrive(); r.request();
        CHECK(r.f.state == F::RUN && r.f.target == 84261);
        ++scenarios;
    }
    { // No-progress time no longer trips or requires a controller reset.
        Rig r; r.request();
        for (int i = 0; i < 300; ++i) r.tick();
        CHECK(r.f.state == F::RUN && r.f.speed > 0);
        r.in.enabled = false; r.tick();
        CHECK(r.f.state == F::IDLE && r.f.speed == 0);
        r.in.enabled = true; r.request(); r.arrive();
        CHECK(r.f.started_count == 2 && r.f.slot_index == 1); ++scenarios;
    }
    { // A slow shot can exceed the former two-second timeout and finish.
        Rig r; r.request();
        for (int i = 0; i < 1200; ++i) r.tick(1);
        CHECK(r.f.state == F::RUN && r.f.speed > 0);
        r.arrive(); ++scenarios;
    }
    { // Overshoot stops forward motion without latching or reversing.
        Rig r; r.request();
        while (r.actual < r.f.target - 1000) r.tick(static_cast<int>(std::min<int64_t>(100,r.f.target-1000-r.actual)));
        r.tick(1600); CHECK(r.f.state == F::SETTLE && r.f.speed == 0);
        for (int i = 0; i < 60; ++i) r.tick();
        CHECK(r.f.state == F::IDLE);
        r.request(); CHECK(r.f.state == F::RUN && r.f.target == 84261); ++scenarios;
    }
    { // A long settling interval remains cancellable and finishes normally.
        Rig r; r.request(); r.in.rpm = 100;
        while (r.actual < r.f.target) r.tick(static_cast<int>(std::min<int64_t>(100,r.f.target-r.actual)));
        for (int i = 0; i < 600; ++i) r.tick();
        CHECK(r.f.state == F::SETTLE && r.f.speed == 0);
        r.in.rpm = 0; for (int i = 0; i < 60; ++i) r.tick();
        CHECK(r.f.state == F::IDLE); ++scenarios;
    }
    { // The normal 100 ms stop confirmation also works across timer wrap.
        Rig r; r.request(); r.in.rpm = 300;
        while (r.actual < r.f.target - 300) r.tick(static_cast<int>(std::min<int64_t>(100,r.f.target-300-r.actual)));
        r.in.now = 0xFFFFFFF0U; r.in.rpm = 0; r.tick(100);
        CHECK(r.f.state == F::SETTLE);
        for (int i = 0; i < 49; ++i) r.tick();
        CHECK(r.f.state == F::SETTLE);
        r.tick();
        CHECK(r.f.state == F::IDLE && r.f.slot_index == 1); ++scenarios;
    }
    std::printf("PASS: %d fixed-point feeder scenarios (including 70 shots with alternating stop errors)\n", scenarios);
}
