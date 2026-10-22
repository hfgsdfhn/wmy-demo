#include "pid/PIDController.hpp"

#include <cmath>
#include <stdexcept>

namespace pid {

PIDController::PIDController(float kp, float ki, float kd,
                             float sample_time_ms, float output_limit,
                             float integral_limit)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      sample_time_ms_(sample_time_ms),
      integral_(0.0F),
      previous_feedback_(0.0F),
      output_limit_(std::fabs(output_limit)),
      integral_limit_(std::fabs(integral_limit)),
      initialized_(false) {
    if (sample_time_ms_ <= 0.0F) {
        throw std::invalid_argument("sample_time_ms must be positive");
    }
}

void PIDController::reset() {
    integral_ = 0.0F;
    previous_feedback_ = 0.0F;
    initialized_ = false;
}

float PIDController::calculate(float target, float feedback) {
    const float error = target - feedback;
    integral_ = clamp(integral_ + error, integral_limit_);

    float derivative = 0.0F;
    if (initialized_) {
        derivative = -(feedback - previous_feedback_) / sample_time_ms_;
    }

    const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    previous_feedback_ = feedback;
    initialized_ = true;
    return clamp(output, output_limit_);
}

float PIDController::clamp(float value, float limit) {
    if (limit == 0.0F) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

}
