/**
 * @file
 * Desktop implementations of the hardware interfaces.
 * Explicitly reject live acquisition and TPU requests in a simulation build.
 * The application selects DemoSource only when --demo is supplied.
 */

#include "app.h"
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
namespace tabletop {
struct Sensors::Impl {}; // Empty complete type allows the public unique_ptr owner to be destroyed in a desktop build.
Sensors::Sensors(const std::string &, int) { // Fail explicitly on live acquisition; desktop callers must choose --demo.
    throw std::runtime_error(
        "Built with hardware disabled: use --demo or build on Pi with TABLETOP_HARDWARE=ON");
}
Sensors::~Sensors() = default; // Default destruction satisfies the public interface without acquiring hardware resources.
Inputs Sensors::snapshot() { // Unreachable after rejecting construction; retained to complete the desktop linkage contract.
    return {};
}
struct Coral::Impl {}; // Empty implementation type completes the desktop inference stub's ownership contract.
Coral::Coral(const std::string &, const std::string &) { // Reject a real Coral request when the executable was compiled without Linux hardware support.
    throw std::runtime_error(
        "Native Coral backend not built: configure TABLETOP_HARDWARE=ON on the Pi");
}
Coral::~Coral() = default; // Default destruction releases no accelerator resources in this backend.
void Coral::submit(const Frame &) {} // Desktop stub has no interpreter; construction already rejects use as a live service.
CoralState Coral::status(double) { // Return no readiness or health evidence from an unavailable inference backend.
    return {};
}
} // namespace tabletop
