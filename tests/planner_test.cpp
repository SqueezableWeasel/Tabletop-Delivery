/**
 * @file
 * Geometric planner tests and an obstacle-position sweep.
 * Validate every returned segment against the modeled obstacle and lane,
 * including continuous diagonals and exact endpoints rather than grid nodes only.
 */

#include "../native/core.h"
#include <cmath> // Finite-value checks, trigonometry, interpolation, and metric geometry calculations.
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
#include <algorithm> // Sorting, bounded copies, searches, and min/max/clamp operations on observations and geometry.

void require(bool ok, const char *message) { // Fail a geometric expectation with a scenario-specific diagnostic.
    if (!ok) // Throw only when the independent geometry check does not hold.
        throw std::runtime_error(message);
}
Config config() { // Build the minimum valid engine configuration needed for isolated planner tests.
    Config c{}; // Zero-initialize all fields before filling the subset required by core configuration validation.
    c.forward = 269.065; // LiDAR sensor azimuth corresponding to tabletop forward, in degrees.
    c.xmin = -.25; // Left edge of the metric tracking region, in meters.
    c.xmax = .25; // Right edge of the metric tracking region, in meters.
    c.ymin = .4; // Near edge of the metric tracking region, in meters forward of the LiDAR.
    c.ymax = .8; // Far edge of the metric tracking region, in meters forward of the LiDAR.
    c.fresh = .5; // Maximum camera/scan receive age allowed for tracking, in seconds.
    c.pair_gap = .25; // Maximum absolute receive-time difference between paired camera and LiDAR, in seconds.
    c.reacquire = .7; // Gap after which a position must be confirmed again instead of continuing the old filter, in seconds.
    c.smoothing = .12; // Exponential position filter time constant, in seconds; smaller values respond faster.
    c.confirmation = 2; // Consecutive consistent scans required before a new track authorizes motion.
    c.min_points = 3; // Minimum number of valid LiDAR returns required for a cluster.
    c.cluster_gap = .045; // Maximum spatial gap between neighboring returns in one cluster, in meters.
    c.anchors[0] = 155; // Set the left camera-pixel anchor so the minimal engine contract has ordered calibration coordinates.
    c.anchors[2] = 333; // Set the center anchor strictly to the right of the left one.
    c.anchors[4] = 517; // Set the right anchor strictly to the right of the center one.
    c.roi[2] = 640; // Make the exclusive right image boundary 640 pixels.
    c.roi[3] = 480; // Make the exclusive bottom image boundary 480 pixels.
    c.robot_radius = .022; // Circular virtual robot footprint radius used for collision checking, in meters.
    c.figure_radius = .045; // Assumed figure footprint radius used by this tabletop model, in meters.
    c.margin = .005; // Additional nonnegative clearance beyond both body radii, in meters.
    c.grid_m = .01; // A* search-grid spacing, in meters; continuous edge checks still guard movement.
    c.speed = .028; // Virtual robot travel speed, in meters per second.
    c.lane_width = .48; // Full width of the wide virtual lane, in meters before footprint insetting.
    c.narrow_width = .16; // Full width of the narrow virtual lane, in meters before footprint insetting.
    c.lane_ymin = .4; // Near boundary of the virtual lane, in meters forward of the LiDAR.
    c.lane_ymax = .8; // Far boundary of the virtual lane, in meters forward of the LiDAR.
    c.start = {0, .435}; // Exact depot coordinate [right,forward], in meters.
    c.goal = {0, .765}; // Exact delivery coordinate [right,forward], in meters.
    return c; // Return the deterministic geometry fixture for every planner case.
}
double distance_to_segment(XY a, XY b, XY p) { // Independently check continuous segment clearance rather than trusting the planner's own checker.
    double dx = b.x - a.x, dy = b.y - a.y, l = dx * dx + dy * dy; // Compute segment direction and squared length for a clamped point projection.
    double t = l > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l, 0., 1.) : 0; // Project onto the finite segment; degenerate segments use their start point.
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy); // Return metric distance from obstacle center to the nearest point along the swept segment.
}
int main() {
    try {
        Config c = config(); // Use the same validated geometry throughout the independent route cases.
        XY path[128]; // Allocate the caller-owned bounded output array for the public planner API.
        int n = core_plan(&c, c.start, c.goal, nullptr, 0, 0, path, 128); // Ask for a wide-lane route with no obstacles; exact start and goal should suffice.
        require(n == 2, "Empty lane must have a direct route"); // An empty lane must return only its two exact endpoints.
        XY centre{0, .63}; // Place a single obstruction on the direct centerline route.
        n = core_plan(&c, c.start, c.goal, &centre, 1, 0, path, 128); // Request a wide-lane route around the centerline figure.
        require(n > 2, "Central obstacle needs a detour in the wide lane"); // The obstructed direct segment must be replaced by at least one intermediate waypoint.
        double radius = c.robot_radius + c.figure_radius + c.margin; // Independently reconstruct the mandatory robot-center exclusion radius.
        for (int i = 1; i < n; ++i) // Check every consecutive returned segment, including diagonal connectors.
            require(distance_to_segment(path[i - 1], path[i], centre) > radius,
                    "Route intersects inflated obstacle"); // Require the complete segment to remain outside the inflated obstacle, not just its endpoints.
        require(core_plan(&c, c.start, c.goal, &centre, 1, 1, path, 128) == 0,
                "Narrow lane must be blocked"); // The same obstruction must leave no route in the narrower lane.
        require(core_plan(&c, c.start, c.goal, &c.start, 1, 0, path, 128) == 0,
                "Cannot plan out of an occupied start"); // A start already inside an obstacle must not produce an escape path.
        require(core_plan(&c, c.start, c.goal, &c.goal, 1, 0, path, 128) == 0,
                "Cannot enter an occupied goal"); // An occupied goal cannot be accepted as a reachable endpoint.
        int checked = 0; // Count completed obstacle placements in the deterministic geometry sweep.
        for (int ix = -20; ix <= 20; ix += 2) // Sweep obstacle x coordinates from -20 to +20 cm in 2 cm steps.
            for (int iy = 50; iy <= 72; iy += 2) { // Sweep obstacle y coordinates from 50 to 72 cm in 2 cm steps.
                XY o{ix / 100., iy / 100.}; // Convert sweep coordinates from centimeters into the core's meter units.
                n = core_plan(&c, c.start, c.goal, &o, 1, 0, path, 128); // Request a route for each independent obstacle placement.
                require(n >= 0, "Planner returned an error on valid geometry"); // Valid geometry may be blocked but must not produce an API argument/internal error.
                for (int i = 0; i < n; ++i) { // Validate every returned waypoint and its incoming segment.
                    require(std::abs(path[i].x) <= c.lane_width / 2 - c.robot_radius + 1e-8,
                            "Route exceeds lane bounds"); // Require each robot-center waypoint inside the footprint-inset lateral lane bounds.
                    if (i) // The first waypoint has no preceding segment; check edges beginning with the second waypoint.
                        require(distance_to_segment(path[i - 1], path[i], o) > radius,
                                "Swept route collision"); // Require the complete segment to remain outside the inflated obstacle, not just its endpoints.
                }
                ++checked; // Count this placement only after all returned geometry passes independent checks.
            }
        for (double selected : {.03, .10}) { // Exercise both supported radii through the real planner, independently of slider labels.
            c.boundary_radius = selected; // Override the nominal footprint sum with the selected total exclusion radius.
            XY side{selected + .022, .60}; // Leave room for the planner's 2 cm preferred padding as well as mandatory clearance.
            require(core_plan(&c, c.start, c.goal, &side, 1, 0, path, 128) == 2,
                    "Selected radius was inflated or ignored"); // A clear direct path must stay direct without silently restoring the nominal radius.
            n = core_plan(&c, c.start, c.goal, &centre, 1, 0, path, 128); // Place the prop on the centerline to require an actual detour at this radius.
            require(n > 2, "Endpoint radius failed to route around central obstacle"); // Both endpoint settings must remain functional collision constraints.
            for (int i = 1; i < n; ++i) // Independently check every segment of the resulting route.
                require(distance_to_segment(path[i - 1], path[i], centre) > selected,
                        "Route violates selected exclusion radius"); // Validate geometry rather than trusting the reported radius alone.
        }
        std::cout << "Planner tests passed: direct, detour, blocked, occupied endpoints, 3/10 cm boundaries, "
                  << checked << " obstacle positions\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
