#include "pid/PIDController.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

int main() {
    pid::PIDController controller(1.0F, 0.5F, 2.0F, 10.0F, 20.0F, 3.0F);

    // First calculation omits the derivative term, matching the original C API.
    assert(std::fabs(controller.calculate(10.0F, 0.0F) - 11.5F) < 0.0001F);
    assert(std::fabs(controller.calculate(10.0F, 2.0F) - 9.1F) < 0.0001F);

    controller.reset();
    assert(std::fabs(controller.calculate(100.0F, 0.0F) - 20.0F) < 0.0001F);

    bool invalid_sample_time_rejected = false;
    try {
        pid::PIDController invalid(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    } catch (const std::invalid_argument&) {
        invalid_sample_time_rejected = true;
    }
    assert(invalid_sample_time_rejected);
    return 0;
}
