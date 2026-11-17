#include "autoaim/draw.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace autoaim {
namespace {

void text(cv::Mat& img, const std::string& s, cv::Point p, cv::Scalar c, double scale = 0.6) {
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, c, 1, cv::LINE_AA);
}

cv::Point toPx(const cv::Point2d& p) { return {cvRound(p.x), cvRound(p.y)}; }

}  // namespace

void drawTrajectory(cv::Mat& img, const AimCommand& cmd, const Ballistics& ballistics, const CameraModel& camera,
                    const cv::Scalar& color) {
    if (!cmd.valid) return;
    const double s_end = std::hypot(cmd.aim_point[0], cmd.aim_point[1]);
    const auto path = ballistics.trajectory(cmd.pitch, s_end);
    if (path.size() < 2) return;
    const double cy = std::cos(cmd.yaw), sy = std::sin(cmd.yaw);
    std::vector<cv::Point> poly;
    for (const auto& p : path) {
        const cv::Vec3d w(p.s * cy, p.s * sy, p.h);
        // The first metre or so is beside / behind the camera (lever arm);
        // projecting it would blow up, so the curve starts where it is visible.
        if (!camera.inFront(w)) continue;
        poly.push_back(toPx(camera.project(w)));
    }
    if (poly.size() < 2) return;
    cv::polylines(img, poly, false, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::polylines(img, poly, false, color, 2, cv::LINE_AA);
    cv::circle(img, poly.back(), 4, color, -1, cv::LINE_AA);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2fm  %.0fms", cmd.distance, path.back().t * 1000);
    text(img, buf, poly.back() + cv::Point(8, 16), color, 0.5);
}

void drawTrajectorySideView(cv::Mat& img, const AimCommand& cmd, const Ballistics& ballistics, cv::Rect box) {
    box &= cv::Rect(0, 0, img.cols, img.rows);
    if (!cmd.valid || box.width < 120 || box.height < 80) return;
    const double s_end = std::hypot(cmd.aim_point[0], cmd.aim_point[1]);
    const auto path = ballistics.trajectory(cmd.pitch, s_end, 40);
    if (path.size() < 2) return;
    cv::Mat sub = img(box);
    sub *= 0.3;
    cv::rectangle(img, box, {255, 255, 255}, 1);
    // metric -> pixel, equal scale: fit the range horizontally, keep the
    // height band of the trajectory and the target visible.
    double h_min = std::min(0.0, cmd.aim_point[2]), h_max = std::max(0.0, cmd.aim_point[2]);
    for (const auto& p : path) h_min = std::min(h_min, p.h), h_max = std::max(h_max, p.h);
    const int ml = 14, mr = 14, mt = 34, mb = 30;
    const double sx = (box.width - ml - mr) / std::max(s_end, 0.5);
    const double sy = (box.height - mt - mb) / std::max(h_max - h_min, 0.05);
    const double sc = std::min(sx, sy * 8.0);   // never exaggerate height more than 8x
    const double vy = std::min(sy, sc * 8.0);   // vertical scale (<= 8x horizontal)
    // local (sub-image) coordinates: drawing on `sub` clips at the box edge
    auto map = [&](double s, double h) {
        return cv::Point(ml + cvRound(s * sc), box.height - mb - cvRound((h - h_min) * vy));
    };
    // ground reference (muzzle height) and line of sight
    cv::line(sub, map(0, 0), map(s_end, 0), {70, 70, 70}, 1, cv::LINE_AA);
    cv::line(sub, map(0, 0), map(s_end, cmd.aim_point[2]), {160, 160, 160}, 1, cv::LINE_AA);
    // barrel direction (straight line at the commanded pitch)
    cv::line(sub, map(0, 0), map(s_end, s_end * std::tan(cmd.pitch)), {255, 255, 0}, 1, cv::LINE_AA);
    std::vector<cv::Point> poly;
    for (const auto& p : path) poly.push_back(map(p.s, p.h));
    cv::polylines(sub, poly, false, {0, 165, 255}, 2, cv::LINE_AA);
    // target plate (panel height, upright)
    const double ph = kSmallArmor.panel_height / 2;
    cv::line(sub, map(s_end, cmd.aim_point[2] - ph), map(s_end, cmd.aim_point[2] + ph), {0, 0, 255}, 3);
    cv::circle(sub, map(0, 0), 3, {255, 255, 255}, -1, cv::LINE_AA);
    const double drop = s_end * std::tan(cmd.pitch) - cmd.aim_point[2];  // barrel line minus target height
    char buf[96];
    std::snprintf(buf, sizeof(buf), "side view  d %.2fm  pitch %+.2fdeg", cmd.distance, cmd.pitch * 180 / kPi);
    text(img, buf, {box.x + 6, box.y + 16}, {230, 230, 230}, 0.45);
    std::snprintf(buf, sizeof(buf), "drop comp %.1fcm  t_fly %.0fms  lead %.0f+%.0fms", drop * 100,
                  path.back().t * 1000, cmd.latency * 1000, cmd.t_fly * 1000);
    text(img, buf, {box.x + 6, box.y + box.height - 10}, {0, 165, 255}, 0.42);
}

void drawOverlay(cv::Mat& img, const FrameObservations& obs, const FrameLog& log, const RobotTracker& geom,
                 const CameraModel& camera, const Ballistics* ballistics) {
    // detections (green) with panel outline
    for (const auto& a : obs.armors) {
        std::vector<cv::Point> poly;
        for (const auto& p : a.det.corners) poly.push_back(toPx(p));
        cv::polylines(img, poly, true, {0, 255, 0}, 1, cv::LINE_AA);
        std::vector<cv::Point> panel;
        for (const auto& p : panelPolygon(a.det)) panel.push_back(toPx(p));
        cv::polylines(img, panel, true, {0, 160, 0}, 1, cv::LINE_AA);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s %.1fm %+.0fdeg", a.det.number.c_str(), a.distance,
                      a.yaw * 180 / kPi);
        text(img, buf, toPx(a.det.corners[1]) + cv::Point(0, -6), {0, 255, 0}, 0.45);
    }

    const bool active = log.state == TrackState::TRACKING || log.state == TrackState::TEMP_LOST;
    const auto& x = log.x;
    if (active) {
        // robot model at the current time: all armors (cyan = facing camera)
        const double center_dir = std::atan2(x[RobotTracker::YC], x[RobotTracker::XC]);
        for (int k = 0; k < geom.armorsNum(); ++k) {
            const double yaw = geom.armorYaw(x, k);
            const bool facing = std::abs(wrapAngle(yaw - center_dir)) < kPi / 2;
            const auto corners = armorCorners(geom.armorPosition(x, k), yaw, 15 * kPi / 180, geom.armor_type);
            std::vector<cv::Point> poly;
            bool ok = true;
            for (const auto& c : corners) {
                if (!camera.inFront(c)) ok = false;
                poly.push_back(toPx(camera.project(c)));
            }
            if (!ok) continue;
            cv::polylines(img, poly, true, facing ? cv::Scalar(255, 255, 0) : cv::Scalar(120, 120, 120), 1,
                          cv::LINE_AA);
        }
        const cv::Vec3d c(x[RobotTracker::XC], x[RobotTracker::YC], x[RobotTracker::ZC]);
        if (camera.inFront(c)) cv::drawMarker(img, toPx(camera.project(c)), {0, 255, 255}, cv::MARKER_CROSS, 14, 2);
    }
    if (ballistics) {
        drawTrajectory(img, log.cmd, *ballistics, camera);
        const int w = std::min(420, img.cols / 3), h = std::min(170, img.rows / 5);
        drawTrajectorySideView(img, log.cmd, *ballistics, cv::Rect(img.cols - w - 10, img.rows - h - 10, w, h));
    }
    if (log.cmd.valid && camera.inFront(log.cmd.aim_point)) {
        cv::circle(img, toPx(camera.project(log.cmd.aim_point)), 7,
                   log.cmd.center_mode ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    if (log.shot && log.shot_rec.result != ShotResult::UNKNOWN) {
        const bool hit = log.shot_rec.result == ShotResult::HIT;
        cv::drawMarker(img, toPx(log.shot_rec.impact_px), hit ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                       cv::MARKER_TILTED_CROSS, 10, 2);
    }

    // status text
    char buf[160];
    std::snprintf(buf, sizeof(buf), "frame %d  %s  matched %d", log.frame, toString(log.state), log.matched);
    text(img, buf, {10, 25}, {255, 255, 255});
    if (active) {
        std::snprintf(buf, sizeof(buf), "w %+.2f rad/s  r1 %.2f  r2 %.2f  dz %+.3f  d %.2fm", x[RobotTracker::W],
                      x[RobotTracker::R1], x[RobotTracker::R2], x[RobotTracker::DZ],
                      std::hypot(x[RobotTracker::XC], x[RobotTracker::YC]));
        text(img, buf, {10, 50}, {255, 255, 0});
    }
    if (log.cmd.valid) {
        std::snprintf(buf, sizeof(buf), "%s  yaw %+.2f pitch %+.2f deg  lead %.0f+%.0f ms  facing %+.0f  %s",
                      log.cmd.center_mode ? "CENTER" : "ARMOR", log.cmd.yaw * 180 / kPi, log.cmd.pitch * 180 / kPi,
                      log.cmd.latency * 1000, log.cmd.t_fly * 1000, log.cmd.facing_deg,
                      log.shot ? (log.shot_rec.result == ShotResult::HIT
                                      ? "FIRE: HIT"
                                      : (log.shot_rec.result == ShotResult::MISS ? "FIRE: MISS" : "FIRE: ?"))
                               : "hold");
        text(img, buf, {10, 75}, log.shot ? cv::Scalar(0, 200, 255) : cv::Scalar(200, 200, 200));
    }

    // top-down inset (x forward -> up in the inset, y left -> left)
    if (active) {
        const int S = 220;
        const cv::Rect roi(img.cols - S - 10, 10, S, S);
        cv::Mat sub = img(roi);
        sub *= 0.35;
        cv::rectangle(img, roi, {255, 255, 255}, 1);
        const cv::Point2d c(x[RobotTracker::XC], x[RobotTracker::YC]);
        const double scale = S / 1.2;  // 1.2 m across
        auto map = [&](double wx, double wy) {
            return cv::Point(roi.x + S / 2 + cvRound(-(wy - c.y) * scale), roi.y + S / 2 + cvRound(-(wx - c.x) * scale));
        };
        // line of sight from the camera
        const double los = std::atan2(c.y, c.x);
        cv::line(img, map(c.x - 0.6 * std::cos(los), c.y - 0.6 * std::sin(los)), map(c.x, c.y), {90, 90, 90}, 1);
        cv::drawMarker(img, map(c.x, c.y), {0, 255, 255}, cv::MARKER_CROSS, 10, 1);
        for (int k = 0; k < geom.armorsNum(); ++k) {
            const cv::Vec3d p = geom.armorPosition(x, k);
            const double yaw = geom.armorYaw(x, k);
            const double hw = 0.07;
            const cv::Point a = map(p[0] - hw * std::sin(yaw), p[1] + hw * std::cos(yaw));
            const cv::Point b = map(p[0] + hw * std::sin(yaw), p[1] - hw * std::cos(yaw));
            const bool chosen = log.cmd.valid && k == log.cmd.armor_k;
            cv::line(img, a, b, chosen ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0), chosen ? 3 : 2);
        }
        for (const auto& a : obs.armors) {
            if (deviceId(a.det) != geom.id() && !geom.id().empty()) continue;
            cv::circle(img, map(a.position[0], a.position[1]), 3, {0, 255, 0}, -1);
        }
        text(img, "top view (1.2 m)", {roi.x + 5, roi.y + S - 8}, {220, 220, 220}, 0.4);
    }
}

}  // namespace autoaim
