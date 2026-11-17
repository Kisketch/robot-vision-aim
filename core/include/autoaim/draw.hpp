// Debug overlay: detections, the robot model re-projected, aim point, shots,
// and a top-down inset of the estimated robot.
#pragma once

#include <opencv2/core.hpp>

#include "autoaim/evaluation.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/tracker.hpp"

namespace autoaim {

// `ballistics` is optional: when given, the predicted bullet path of the
// command is drawn as well (drawTrajectory).
void drawOverlay(cv::Mat& img, const FrameObservations& obs, const FrameLog& log, const RobotTracker& geom,
                 const CameraModel& camera, const Ballistics* ballistics = nullptr);

// Predicted bullet path of `cmd`: fired from the gimbal origin at
// (cmd.yaw, cmd.pitch), integrated with the drag model out to the horizontal
// range of cmd.aim_point, projected into the image.  The impact end is
// labelled with the target distance and the time of flight.  No-op for an
// invalid command.
void drawTrajectory(cv::Mat& img, const AimCommand& cmd, const Ballistics& ballistics, const CameraModel& camera,
                    const cv::Scalar& color = cv::Scalar(0, 165, 255));

}  // namespace autoaim
