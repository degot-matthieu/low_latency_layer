#include "delay_controller.hh"

#include <algorithm>
#include <fstream>
#include <print>

namespace low_latency {

DelayController::DelayController() {}

DelayController::~DelayController() {}

void DelayController::delay(const DeviceClock::duration& min_delay) {
    using namespace std::chrono;

    if (!this->previous_frame.has_value()) {
        this->previous_frame.emplace(frame_info{
            .release = DeviceClock::now(),
            .frametime = 0ns,
            .jitter = 0ns,
            .drain = 0ns,
        });
        return;
    }

    // Apply frame limits.
    while (DeviceClock::now() < this->previous_frame->release + min_delay) {
        std::this_thread::yield();
    }

    const auto jitter_magnitude =
        std::max(this->previous_frame->frametime / 100, nanoseconds{100'000});
    const auto jitter =
        this->previous_frame->jitter == 0ns ? jitter_magnitude : 0ns;

    // Apply jitter.
    for (const auto start = DeviceClock::now();
         DeviceClock::now() < start + jitter;) {
        std::this_thread::yield();
    }

    // Apply extra drain time.
    for (const auto start = DeviceClock::now();
         DeviceClock::now() < start + this->drain;) {
        std::this_thread::yield();
    }

    const auto now = DeviceClock::now();
    const auto frametime = now - this->previous_frame->release;
    const auto applied_drain = this->drain;

    // Calculate our gradient.
    // -1 => Applying a random jitter sleep actually improved frametimes.
    // 0  => Applying a sleep did nothing. We are behind in simulation depth.
    // 1  => Applying a sleep directly impacted frametimes. Don't sleep!
    const auto gradient = [&]() -> auto {
        const auto dt_jitter = this->previous_frame->jitter != 0ns
                                   ? -this->previous_frame->jitter
                                   : jitter;
        // Strip drain from both sides so changing drain doesn't corrupt the signal.
        const auto natural_frametime = frametime - applied_drain;
        const auto prev_natural_frametime = this->previous_frame->frametime
                                            - this->previous_frame->drain;
        const auto dt_frametime = natural_frametime - prev_natural_frametime;
        const auto dt_frametime_ns = static_cast<double>(dt_frametime.count());
        const auto dt_jitter_ns = static_cast<double>(dt_jitter.count());
        return std::clamp(dt_frametime_ns / dt_jitter_ns, -1.0, 1.0);
    }();

    // Feed our gradient into ewma -> our gradient is noisy and an ewma smooths
    // it out into a readable signal.
    constexpr auto ALPHA = 0.01; // Not tuned - appears to work OK.
    this->gradient_ewma =
        (ALPHA * gradient) + ((1.0 - ALPHA) * this->gradient_ewma);

    const auto drain_attack = [&]() -> nanoseconds {
        if (this->gradient_ewma <= EWMA_DRAIN_ON) {
            const auto factor = std::clamp(
                (EWMA_DRAIN_ON - this->gradient_ewma) / EWMA_DRAIN_ON, 0.0, 1.0);
            const auto frametime_ns = static_cast<double>(frametime.count());
            return nanoseconds{static_cast<long>(frametime_ns * 0.01 * factor)};
        }
        if (this->gradient_ewma > EWMA_DRAIN_OFF) {
            return -(this->drain / 100);
        }
        return 0ns;
    }();

    // Don't drain more than our current frametime as a safety measure.
    if (drain_attack != 0ns) {
        const auto prev_drain = this->drain;
        this->drain += drain_attack;
        const auto prev_time = frametime - prev_drain;
        this->drain = std::clamp(this->drain, nanoseconds{0}, prev_time);
    }

    static auto out = std::ofstream{"/tmp/low_latency.log", std::ios::trunc};

    std::println(out,
                 "delay_controller::delay()\n"
                 "    gradient: {}\n"
                 "    gradient_ewma: {}\n"
                 "    drain: {}\n"

                 ,
                 gradient, gradient_ewma, drain);

    this->previous_frame.emplace(frame_info{
        .release = now,
        .frametime = frametime,
        .jitter = jitter,
        .drain = applied_drain,
    });
}

} // namespace low_latency
