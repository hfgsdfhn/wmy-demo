#pragma once

namespace pid {

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float sample_time_ms,
                  float output_limit, float integral_limit);

    void reset();
    float calculate(float target, float feedback);

private:
    static float clamp(float value, float limit);

    float kp_;
    float ki_;
    float kd_;
    float sample_time_ms_;
    float integral_;
    float previous_feedback_;
    float output_limit_;
    float integral_limit_;
    bool initialized_;
};

}  // namespace pid
