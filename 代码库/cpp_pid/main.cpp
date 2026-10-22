#include "pid/PIDController.hpp"

#include <iomanip>
#include <iostream>

int main() {
    pid::PIDController controller(1.0F, 0.1F, 0.05F, 10.0F, 100.0F, 50.0F);

    const float target = 30.0F;
    const float feedback_values[] = {0.0F, 8.0F, 16.0F, 23.0F, 28.0F};

    std::cout << std::fixed << std::setprecision(2);
    for (float feedback : feedback_values) {
        std::cout << "feedback=" << feedback
                  << ", output=" << controller.calculate(target, feedback)
                  << '\n';
    }
    return 0;
}
