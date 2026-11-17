// Latency chain and frame-rate independence (2026-11-17):
//   - AimerConfig::totalLatency sums the chain; a measured exposure->now
//     interval replaces transfer + process; bad measurements fall back;
//   - the aimers predict at now + totalLatency (+ gimbal_lag) and report it;
//   - tracker thresholds given in seconds mean the same at 25 and 165 Hz;
//   - eval.max_shot_rate caps the simulated shots per second.
#include <cmath>
#include <cstdio>
#include <limits>

#include "autoaim/aimer.hpp"
#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/evaluation.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/tracker.hpp"

#include "check.hpp"
#include "fixtures.hpp"

using namespace autoaim;

namespace {

AimerConfig chainConfig() {
    AimerConfig a;
    a.latency = 0.001;
    a.transfer_delay = 0.010;
    a.process_delay = 0.005;
    a.comm_delay = 0.020;
    a.fire_delay = 0.030;
    return a;
}

TargetState spinnerState(double stamp) {
    TargetState s;
    s.valid = true;
    s.id = "R3:small";
    s.label = "3";
    s.state = TrackState::TRACKING;
    s.armors_num = 4;
    s.stamp = stamp;
    s.center = {5.0, 0.2, -0.1};
    s.omega = 3.0;
    s.yaw = 0.2;
    for (int k = 0; k < 4; ++k) {
        s.armors[k].radius = 0.24;
        s.armors[k].yaw = s.yaw + kPi / 2 * k;
        s.armors[k].position = s.center - 0.24 * cv::Vec3d(std::cos(s.armors[k].yaw), std::sin(s.armors[k].yaw), 0);
    }
    return s;
}

}  // namespace

void testLatencyChain() {
    const AimerConfig a = chainConfig();
    CHECK(std::abs(a.totalLatency() - 0.066) < 1e-12, "chain sum %.6f", a.totalLatency());
    // measured exposure -> now replaces transfer + process (0.015)
    CHECK(std::abs(a.totalLatency(0.0072) - (0.001 + 0.0072 + 0.020 + 0.030)) < 1e-12, "measured %.6f",
          a.totalLatency(0.0072));
    CHECK(std::abs(a.totalLatency(-1.0) - a.totalLatency()) < 1e-12, "negative = not measured");
    CHECK(std::abs(a.totalLatency(std::numeric_limits<double>::quiet_NaN()) - a.totalLatency()) < 1e-12,
          "NaN = not measured");
    CHECK(std::abs(a.totalLatency(std::numeric_limits<double>::infinity()) - a.totalLatency()) < 1e-12,
          "inf = not measured");
    // the historical lumped config is unchanged
    AimerConfig legacy;
    CHECK(std::abs(legacy.totalLatency() - legacy.latency) < 1e-15, "chain defaults to 0");

    // Config validation rejects negative components and an absurd total.
    Config c;
    c.aimer.comm_delay = -0.001;
    CHECK(!c.validate().empty(), "negative comm_delay rejected");
    c.aimer.comm_delay = 0.95;
    CHECK(!c.validate().empty(), "total >= 1 s rejected");

    // The aimer predicts at now + totalLatency and reports the latency used.
    Ballistics ball{BallisticConfig{}};
    const TargetState s = spinnerState(10.0);
    {
        RobotAimer aimer(a, ball);
        const AimCommand cmd = aimer.aim(s, 10.0);
        CHECK(cmd.valid, "chain aim valid");
        CHECK(std::abs(cmd.latency - 0.066) < 1e-9, "reported latency %.6f", cmd.latency);
        CHECK(std::abs(cmd.t_impact - (10.0 + 0.066 + cmd.t_fly)) < 1e-9, "t_impact uses chain");
    }
    {
        RobotAimer aimer(a, ball);
        const AimCommand cmd = aimer.aim(s, 10.0, 0.0072);
        CHECK(std::abs(cmd.latency - 0.0582) < 1e-9, "measured latency used: %.6f", cmd.latency);
        CHECK(std::abs(cmd.t_impact - (10.0 + 0.0582 + cmd.t_fly)) < 1e-9, "t_impact uses measured");
    }
    // Equivalence: the chain and a lumped latency of the same total give the
    // same command (the split changes bookkeeping, not the maths).
    {
        AimerConfig lumped;
        lumped.latency = a.totalLatency();
        RobotAimer x(a, ball), y(lumped, ball);
        const AimCommand cx = x.aim(s, 10.0), cy = y.aim(s, 10.0);
        CHECK(std::abs(cx.yaw - cy.yaw) < 1e-12 && std::abs(cx.pitch - cy.pitch) < 1e-12 && cx.fire == cy.fire,
              "chain == lumped of same total");
    }
    // gimbal_lag predicts further ahead but is not departure latency.
    {
        AimerConfig lag = a;
        lag.gimbal_lag = 0.02;
        AimerConfig lumped;
        lumped.latency = a.totalLatency() + 0.02;
        RobotAimer x(lag, ball), y(lumped, ball);
        const AimCommand cx = x.aim(s, 10.0), cy = y.aim(s, 10.0);
        CHECK(std::abs(cx.yaw - cy.yaw) < 1e-12, "gimbal_lag aims like a longer horizon");
        CHECK(std::abs(cx.latency - 0.066) < 1e-9, "gimbal_lag not reported as latency: %.6f", cx.latency);
    }
}

void testTimeBasedThresholds() {
    const Spinner sp;
    CameraModel cam{CameraConfig{}};
    // tracking_thres_time: at 165 Hz, 3 frames (12 ms) must not promote when
    // 80 ms of evidence is required; at 25 Hz the time rule and the frame rule
    // coincide (frames 0, 1, 2 span 80 ms).
    {
        TrackerConfig tc;
        tc.tracking_thres_time = 0.08;
        RobotTracker tr(tc, cam);
        const double fps = 165.0;
        int n = 0;
        double first_tracking = -1;
        for (; n < 60; ++n) {
            tr.update(sp.frame(n / fps, n));
            if (tr.state() == TrackState::TRACKING && first_tracking < 0) first_tracking = n / fps;
        }
        CHECK(first_tracking >= 0.08 - 1e-9 && first_tracking < 0.08 + 2.0 / fps,
              "165 Hz: TRACKING after %.4f s (want ~0.08)", first_tracking);
    }
    {
        TrackerConfig frames, timed;
        timed.tracking_thres_time = 0.08;
        RobotTracker a(frames, cam), b(timed, cam);
        for (int n = 0; n < 6; ++n) {
            a.update(sp.frame(n / 25.0, n));
            b.update(sp.frame(n / 25.0, n));
            CHECK(a.state() == b.state(), "25 Hz: frame %d states agree (%s vs %s)", n, toString(a.state()),
                  toString(b.state()));
        }
    }
    // lost_grace_time: 20 ms of dropout at 165 Hz (3 frames) holds TRACKING,
    // 5 frames (30 ms) demote.
    {
        TrackerConfig tc;
        tc.lost_grace_time = 0.02;
        RobotTracker tr(tc, cam);
        const double fps = 165.0;
        int n = 0;
        for (; n < 200; ++n) tr.update(sp.frame(n / fps, n));
        CHECK(tr.state() == TrackState::TRACKING, "165 Hz grace: tracking before drop");
        const double t_last = (n - 1) / fps;
        for (int k = 0; k < 3; ++k, ++n) {
            FrameObservations e;
            e.frame = n, e.stamp = n / fps;
            tr.update(e);
        }
        CHECK(tr.state() == TrackState::TRACKING, "3 dropped frames (%.1f ms) hold TRACKING",
              ((n - 1) / fps - t_last) * 1000);
        for (int k = 0; k < 2; ++k, ++n) {
            FrameObservations e;
            e.frame = n, e.stamp = n / fps;
            tr.update(e);
        }
        CHECK(tr.state() != TrackState::TRACKING, "5 dropped frames (%.1f ms) demote",
              ((n - 1) / fps - t_last) * 1000);
    }
    // trackingTime() follows the stamps.
    {
        RobotTracker tr(TrackerConfig{}, cam);
        for (int n = 0; n < 50; ++n) tr.update(sp.frame(n / 165.0, n));
        CHECK(tr.state() == TrackState::TRACKING && tr.trackingTime() > 0.2 && tr.trackingTime() < 0.31,
              "trackingTime %.3f", tr.trackingTime());
    }
}

void testShotRateCap() {
    // A spinner fed at 165 Hz with a judge that only counts: without a cap
    // the evaluator fires on (almost) every frame; with max_shot_rate 20 it
    // fires at most 20 times per second.
    const Spinner sp;
    CameraModel cam{CameraConfig{}};
    std::vector<FrameObservations> obs;
    const double fps = 165.0;
    for (int n = 0; n < 330; ++n) obs.push_back(sp.frame(n / fps, n));
    const JudgeFn judge = [](double, double, double, double, const std::string&, ShotRecord& rec) {
        rec.result = ShotResult::HIT;
    };
    Config c;
    c.eval.fps = fps;
    c.aimer.fire_sigma_scale = 0;
    const EvalResult free_run = evaluate(c, obs, cam, false, &judge);
    c.eval.max_shot_rate = 20;
    const EvalResult capped = evaluate(c, obs, cam, false, &judge);
    const int robot_free = free_run.strategies[2].shots, robot_capped = capped.strategies[2].shots;
    CHECK(robot_capped > 0 && robot_capped <= 20 * 2 + 1, "capped shots %d over 2 s", robot_capped);
    CHECK(robot_free > 3 * robot_capped, "uncapped fires far more: %d vs %d", robot_free, robot_capped);
}
