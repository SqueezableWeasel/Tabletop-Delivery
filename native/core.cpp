/**
 * @file
 * Portable image processing, association, tracking, planning, and virtual motion.
 * This module has no OpenCV, libcamera, SDK, or TPU dependency. It accepts
 * owned observations through core_step and returns a bounded Result snapshot.
 */

#include "core.h"
#include <algorithm> // Sorting, bounded copies, searches, and min/max/clamp operations on observations and geometry.
#include <array> // Fixed-size standard arrays used where element count is part of the data contract.
#include <cmath> // Finite-value checks, trigonometry, interpolation, and metric geometry calculations.
#include <cstring> // Byte copying or C string comparison at native library and image-buffer boundaries.
#include <deque> // History storage supporting efficient removal of expired oldest observations.
#include <functional> // Comparison/function utilities used by the native implementation.
#include <limits> // Numeric limits, including the unreachable-cost infinity used by grid search.
#include <queue> // Priority-queue storage for lowest-estimated-cost-first route search.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
#include <utility> // Move semantics and pair values used to transfer ownership or store search state.
#include <vector> // Owned variable-length storage for pixels, measurements, routes, or detections.

namespace {
constexpr double PI = 3.14159265358979323846; // Convert degrees from the LiDAR/calibration into radians for trigonometry.
double norm(XY a) { // Euclidean length of a two-dimensional vector, in the vector's units.
    return std::hypot(a.x, a.y); // hypot computes the length without directly squaring large coordinates.
}
XY sub(XY a, XY b) { // Subtract positions to obtain the displacement from b to a.
    return {a.x - b.x, a.y - b.y}; // Compute independent rightward and forward displacement components.
}
XY add(XY a, XY b) { // Translate a point by a displacement, component by component.
    return {a.x + b.x, a.y + b.y}; // Add both metric coordinates without changing the coordinate convention.
}
XY mul(XY a, double f) { // Scale a displacement by a dimensionless interpolation factor.
    return {a.x * f, a.y * f}; // Apply the same factor to x and y to preserve direction.
}
double distance(XY a, XY b) { // Obtain point-to-point distance in meters using the vector helpers.
    return norm(sub(a, b)); // Measure the length of the displacement rather than either absolute position.
}
double wrap(double a) { // Normalize a bearing to the half-open interval [-180, 180) degrees.
    a = std::fmod(a + 180., 360.); // Shift the interval before taking the remainder of one revolution.
    if (a < 0) // C++ remainder can be negative, so correct it before shifting back.
        a += 360.; // Move a negative remainder into the positive revolution.
    return a - 180.; // Restore the signed bearing centered on forward.
}
double segment_distance(XY a, XY b, XY p) { // Distance from a point to the closest point on a finite segment. Clamping the projection is what protects endpoint and diagonal motion checks.
    XY d = sub(b, a); // Direction and length of the segment whose closest point is needed.
    double length = d.x * d.x + d.y * d.y; // Squared segment length; the projection denominator uses meters squared.
    if (length < 1e-16) // Treat coincident endpoints as one point and avoid dividing by zero.
        return distance(a, p); // The only available closest point is the degenerate segment's endpoint.
    double t = std::clamp(((p.x - a.x) * d.x + (p.y - a.y) * d.y) / length, 0., 1.); // Project p onto the line and clamp to the finite segment, including its endpoints.
    return distance(add(a, mul(d, t)), p); // Measure from p to the clamped projection in meters.
}
bool valid_config(const Config &c) { // Minimal engine invariants, also used by core-only tests. The application parser applies additional type/range checks before creating the engine.
    return std::isfinite(c.forward) && c.xmin < c.xmax && c.ymin < c.ymax && c.fresh > 0 &&
           c.pair_gap > 0 && c.reacquire > 0 && c.smoothing > 0 && c.confirmation >= 1 &&
           c.min_points >= 2 && c.cluster_gap > 0 && c.anchors[0] < c.anchors[2] &&
           c.anchors[2] < c.anchors[4] && c.roi[0] >= 0 && c.roi[1] >= 0 && c.roi[2] <= 640 &&
           c.roi[3] <= 480 && c.roi[0] < c.roi[2] && c.roi[1] < c.roi[3] && c.robot_radius > 0 &&
           c.figure_radius > 0 && c.margin >= 0 && std::isfinite(c.figure_radius + c.robot_radius + c.margin) &&
           valid_boundary_radius(exclusion_radius(c)) && c.grid_m >= .005 && c.grid_m <= .04 &&
           c.speed > 0 && c.speed <= .2 && c.speed_scale > 0 && c.speed_scale <= 1 &&
           c.lane_width > 2 * c.robot_radius &&
           c.narrow_width > 2 * c.robot_radius && c.lane_ymax - c.lane_ymin > 2 * c.robot_radius; // Reject invalid geometry, timing, calibration order, and unsupported planner resolution.
}

struct Space { // Configuration space for the robot center: inset lane walls by robot radius and exclude the selected radius around each observed figure position.
    double lo, hi, bottom, top, radius; // Robot-center limits and inflated obstacle radius, all in meters.
    std::vector<XY> obstacles; // Obstacle centers; raw and smoothed figure positions can both be present.
    Space(const Config &c, bool narrow, std::vector<XY> obs) : obstacles(std::move(obs)) { // Move the obstacle list into this collision-checking configuration space.
        double width = narrow ? c.narrow_width : c.lane_width; // Choose the active lane's full width before accounting for robot size.
        lo = -width / 2 + c.robot_radius; // Left wall inset by the robot footprint; x increases to the right.
        hi = width / 2 - c.robot_radius; // Right wall inset by the same footprint radius.
        bottom = c.lane_ymin + c.robot_radius; // Near lane boundary inset so the robot body stays inside the lane.
        top = c.lane_ymax - c.robot_radius; // Far lane boundary inset by the robot radius.
        radius = exclusion_radius(c); // Use the same selected total radius as the display and saved preference; do not add another robot radius.
    }
    bool inside(XY p) const { // Test robot-center bounds with a small tolerance for floating-point rounding.
        return p.x >= lo - 1e-9 && p.x <= hi + 1e-9 && p.y >= bottom - 1e-9 && p.y <= top + 1e-9; // Require all four bounds; the 1e-9 meter allowance is numerical, not physical clearance.
    }
    bool clear(XY a, XY b) const { // A rectangular lane is convex, so in-lane endpoints keep the segment inside. The segment-to-center distance tests the entire sweep against every obstacle.
        if (!inside(a) || !inside(b)) // Reject a motion whose start or end lies outside the inset rectangular lane.
            return false;
        for (XY p : obstacles) // Test the whole segment against every supplied obstacle center.
            if (segment_distance(a, b, p) <= radius + 1e-7) // Contact with or penetration of an inflated obstacle blocks movement.
                return false;
        return true;
    }
};

Space planning_space(const Space &hard, XY start, XY goal) { // Plan away from the hard collision boundary. Using exactly the same radius for planning and validation makes a tangent route fail on the next noisy scan. Keep every motion check against the original raw + filtered obstacles.
    double padding = .020; // Prefer 2 cm of extra clearance when geometry permits.
    for (XY obstacle : hard.obstacles) {
        double available =
            std::min(distance(start, obstacle), distance(goal, obstacle)) - hard.radius; // Find the smaller endpoint clearance beyond the mandatory collision radius.
        padding = std::min(padding, std::max(0., available * .5)); // Do not trap an otherwise valid start or goal inside the extra band. Limit preferred padding to half the free endpoint clearance, never below zero.
    }
    Space preferred = hard; // Copy hard constraints so preference changes cannot alter mandatory collision checks.
    preferred.radius += padding; // Add only discretionary clearance; lane bounds remain unchanged.
    return preferred; // Return the padded space for route selection, not for final motion authorization.
}

std::vector<XY> plan(const Config &c, XY start, XY goal, const Space &space) { // A* on an eight-neighbor grid, with exact continuous endpoints and Euclidean heuristic. Return an empty path if endpoints or all candidate passages fail.
    if (!space.clear(start, start) || !space.clear(goal, goal)) // Require both exact endpoints to be valid before searching for a route.
        return {};
    if (space.clear(start, goal)) // Skip grid search when a continuously checked direct segment is available.
        return {start, goal}; // A direct path contains both exact endpoints and no intermediate grid nodes.
    int nx = int(std::floor((space.hi - space.lo) / c.grid_m)) + 1; // Count grid columns that fit within the inset lane at grid_m meter spacing.
    int ny = int(std::floor((space.top - space.bottom) / c.grid_m)) + 1; // Count grid rows between the near and far robot-center limits.
    if (nx < 1 || ny < 1 || nx * ny > 50000) // Bound grid allocation and reject dimensions that cannot contain any node.
        return {};
    auto point = [&](int i) { // Convert a flat row-major grid index into a metric coordinate.
        return XY{space.lo + (i % nx) * c.grid_m, space.bottom + (i / nx) * c.grid_m}; // Remainder selects the column; integer division selects the row.
    };
    const int n = nx * ny; // Total number of nodes, already bounded by the grid-size check.
    std::vector<double> cost(n, std::numeric_limits<double>::infinity()); // Initialize best-known path costs to unreachable until a node is discovered.
    std::vector<int> parent(n, -2); // -2 means undiscovered; -1 will mark a node connected directly to the start.
    using Item = std::pair<double, int>; // Each priority-queue item holds estimated total cost followed by node index.
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open; // A min-priority queue expands the smallest path cost plus Euclidean heuristic.
    for (int i = 0; i < n; ++i) { // Connect exact continuous start to nearby grid nodes; no unchecked snap. Consider every grid node as a possible exact-start connector.
        XY p = point(i); // Recover this grid node's metric position for distance and collision checks.
        double d = distance(start, p); // Measure the exact start-to-node connector length in meters.
        if (d <= c.grid_m * 1.6 && space.clear(start, p)) { // Connect only nearby nodes, and check the entire connector before admitting it.
            cost[i] = d; // Seed this node's path cost with the checked connector length.
            parent[i] = -1; // Mark a root connector so backtracking terminates at the continuous start.
            open.push({d + distance(p, goal), i}); // Prioritize using path length so far plus straight-line distance to the goal.
        }
    }
    int end = -1; // Expand the lowest estimated total cost. Ignore stale queue entries and connect the goal only through a continuously collision-checked segment. No goal-connected node has been found yet.
    while (!open.empty()) { // Continue A* until a valid goal connector is found or reachable nodes are exhausted.
        auto [estimate, i] = open.top(); // Read the queued estimated cost and its flat grid index.
        open.pop(); // Remove this entry; improved costs may leave older entries elsewhere in the queue.
        XY p = point(i); // Recover this grid node's metric position for distance and collision checks.
        if (estimate > cost[i] + distance(p, goal) + 1e-9) // Discard a queued estimate superseded by a shorter path to the same node.
            continue;
        if (distance(p, goal) <= c.grid_m * 1.6 && space.clear(p, goal)) { // Connect to the exact goal only from a nearby node with a clear segment.
            end = i; // Remember the grid endpoint for parent-chain reconstruction.
            break;
        }
        for (int dy = -1; dy <= 1; ++dy) // Visit neighboring rows, including the current row for horizontal connections.
            for (int dx = -1; dx <= 1; ++dx) { // Visit neighboring columns; together with dy this covers the eight neighbors.
                if (dx == 0 && dy == 0) // Exclude the current node, which is not a movement edge.
                    continue;
                int x = i % nx + dx, y = i / nx + dy; // Convert the current grid index to coordinates and apply the neighbor offset.
                if (x < 0 || y < 0 || x >= nx || y >= ny) // Ignore neighbors beyond the finite search grid.
                    continue;
                int j = y * nx + x; // Convert the neighbor coordinates back to a row-major index.
                XY q = point(j); // Obtain the neighbor's metric endpoint for continuous collision checking.
                if (!space.clear(p, q)) // Reject even a diagonal edge if any part crosses an inflated obstacle or wall.
                    continue; // Includes swept diagonal collision.
                double next = cost[i] + distance(p, q); // Candidate path length is the best current cost plus this edge's metric length.
                if (next + 1e-10 < cost[j]) { // Update only for a meaningful improvement, avoiding floating-point tie churn.
                    cost[j] = next; // Store the shorter known distance to this neighbor.
                    parent[j] = i; // Record the predecessor needed to reconstruct the accepted route.
                    open.push({next + distance(q, goal), j}); // Queue the improved route with an admissible straight-line goal heuristic.
                }
            }
    }
    if (end < 0) // Exhausting the frontier without a connector means no route was found.
        return {};
    std::vector<XY> reverse{goal}; // Begin reconstruction at the exact goal, then follow grid parents backwards.
    for (int i = end; i >= 0; i = parent[i]) // Stop at the -1 start-connector sentinel without indexing it.
        reverse.push_back(point(i)); // Append each predecessor's metric position to the reverse path.
    reverse.push_back(start); // Preserve the exact continuous start rather than snapping it to a grid node.
    std::reverse(reverse.begin(), reverse.end()); // Put the reconstructed path into travel order from start to goal.
    std::vector<XY> smooth{start}; // Greedy visibility simplification, with the same continuous collision test. Seed the simplified route with the exact start.
    size_t i = 0; // Index of the last retained point in the unsimplified route.
    while (i + 1 < reverse.size()) { // Keep extending the simplified route until it reaches the final point.
        size_t j = reverse.size() - 1; // Try the farthest remaining point first to eliminate unnecessary bends.
        while (j > i + 1 && !space.clear(reverse[i], reverse[j])) // Back off until a direct segment from the retained point is collision-free.
            --j; // Consider the next closer waypoint when a farther shortcut is blocked.
        smooth.push_back(reverse[j]); // Keep the farthest visible waypoint as the next route bend.
        i = j; // Resume simplification from the newly retained waypoint.
    }
    return smooth; // Return the simplified, continuously checked polyline including both endpoints.
}

std::vector<XY> stable_plan(const Config &c, XY start, XY goal, const Space &hard) { // Attempt preferred-clearance routes before reducing only the extra padding. The hard collision boundary is never reduced when searching a narrow passage.
    Space preferred = planning_space(hard, start, goal); // Begin with additional clearance selected from start/goal geometry.
    double padding = preferred.radius - hard.radius; // Separate discretionary padding from the mandatory collision radius.
    for (double fraction : {1., .5, .25, 0.}) { // Narrow clear passages remain usable. Relax only the preference, never the original collision radius, and only when a wider route is impossible. Try full, half, quarter, then zero extra padding if no wider route fits.
        preferred.radius = hard.radius + padding * fraction; // Relax only the preference; never shrink below the hard obstacle boundary.
        auto route = plan(c, start, goal, preferred); // Run the same continuous-edge grid search for this clearance preference.
        if (!route.empty()) // Accept the first feasible route, preserving the widest available preference.
            return route; // Pass the selected polyline to the caller for later hard-boundary revalidation.
    }
    return {};
}

std::vector<Marker> markers(const unsigned char *bgr, int stride, const Config &c) { // Build an HSV mask within the configured ROI, then flood-fill eight-connected components. Return centroid/bounding-box candidates that pass area and aspect gates.
    constexpr int width = 640, height = 480; // The calibrated processing contract fixes camera coordinates at 640 by 480 pixels.
    std::vector<unsigned char> mask(width * height, 0); // Allocate one byte per pixel, initially rejecting all pixels outside the ROI.
    for (int y = c.roi[1]; y < c.roi[3]; ++y) // Iterate only the calibrated camera region's rows; the upper bound is exclusive.
        for (int x = c.roi[0]; x < c.roi[2]; ++x) { // Iterate ROI columns without changing the full-image coordinate system.
            const unsigned char *p = bgr + y * stride + 3 * x; // Locate this packed BGR pixel using the supplied row stride.
            double b = p[0], g = p[1], r = p[2], mx = std::max({r, g, b}), mn = std::min({r, g, b}),
                   d = mx - mn; // Read channels and extrema; their difference determines hue and saturation.
            double h = 0; // Achromatic pixels retain hue zero; saturation will also be zero.
            if (d > 0) { // Compute hue only when the channels differ, avoiding a zero denominator.
                if (mx == r) // Red-dominant colors use the red sector of the HSV conversion.
                    h = 60 * std::fmod((g - b) / d, 6.); // Compute the red-sector hue in degrees, allowing a temporary negative result.
                else if (mx == g) // Green-dominant colors occupy the sector offset by two sixths of a revolution.
                    h = 60 * ((b - r) / d + 2.); // Calculate the green-sector hue in degrees.
                else
                    h = 60 * ((r - g) / d + 4.); // Calculate the blue-sector hue in degrees.
                if (h < 0) // Normalize negative red-sector hue into the positive color wheel.
                    h += 360; // Add one full revolution without changing the represented color.
            }
            int H = int(std::lround(h / 2)) % 180, S = mx > 0 ? int(std::lround(255 * d / mx)) : 0,
                V = int(mx); // OpenCV's HSV convention: H 0..179, S/V 0..255. Quantize hue to half-degrees and saturation/value to 8-bit OpenCV conventions.
            mask[y * width + x] = (H >= c.hsv_low[0] && H <= c.hsv_high[0] && S >= c.hsv_low[1] &&
                                   S <= c.hsv_high[1] && V >= c.hsv_low[2] && V <= c.hsv_high[2]); // Accept a pixel only when hue, saturation, and value all fall inside their bounds.
        }
    std::vector<Marker> found; // The queue doubles as the component pixel list. Clearing visited mask bytes prevents revisits; the initially zero mask keeps traversal inside the image ROI. Collect components that satisfy the marker's area and aspect-ratio limits.
    std::vector<int> queue; // Breadth-first component traversal stores flat indices into the mask.
    queue.reserve(8192); // Reuse traversal storage between components to reduce allocation overhead.
    for (int y = c.roi[1]; y < c.roi[3]; ++y) // Iterate only the calibrated camera region's rows; the upper bound is exclusive.
        for (int x = c.roi[0]; x < c.roi[2]; ++x) { // Iterate ROI columns without changing the full-image coordinate system.
            int seed = y * width + x; // Convert the candidate component seed to its full-image mask index.
            if (!mask[seed]) // Ignore rejected pixels and pixels already consumed by another component.
                continue;
            mask[seed] = 0; // Mark the seed visited as soon as it is queued, preventing duplicate visits.
            queue.clear(); // Reuse the queue for the next disconnected component.
            queue.push_back(seed); // Start the flood fill at this accepted, previously unvisited pixel.
            int xmin = x, xmax = x, ymin = y, ymax = y; // Initialize the component bounding box to its first pixel.
            double sx = 0, sy = 0; // Accumulate coordinate sums for an area-weighted pixel centroid.
            for (size_t k = 0; k < queue.size(); ++k) { // Process the growing queue until all connected foreground pixels have been visited.
                int u = queue[k] % width, v = queue[k] / width; // Decode the queued pixel into image column and row.
                sx += u; // Add this pixel's column to the centroid numerator.
                sy += v; // Add this pixel's row to the centroid numerator.
                xmin = std::min(xmin, u); // Extend the component's left bound when a smaller column is encountered.
                xmax = std::max(xmax, u); // Extend the component's right bound when a larger column is encountered.
                ymin = std::min(ymin, v); // Extend the component's top bound when a smaller row is encountered.
                ymax = std::max(ymax, v); // Extend the component's bottom bound when a larger row is encountered.
                for (int dy = -1; dy <= 1; ++dy) // Visit neighboring rows, including the current row for horizontal connections.
                    for (int dx = -1; dx <= 1; ++dx) { // Visit neighboring columns; together with dy this covers the eight neighbors.
                        int xx = u + dx, yy = v + dy; // Form a neighboring pixel coordinate, including diagonals for eight- connectivity.
                        if (xx < 0 || xx >= width || yy < 0 || yy >= height) // Check image bounds before calculating or dereferencing a neighbor index.
                            continue;
                        int j = yy * width + xx; // Flatten the valid neighbor coordinate into a mask index.
                        if (mask[j]) { // Enqueue only foreground pixels not already visited.
                            mask[j] = 0; // Claim the pixel before enqueueing so other neighbors cannot enqueue it again.
                            queue.push_back(j); // Append the new pixel to the same connected component.
                        }
                    }
            }
            int area = int(queue.size()), w = xmax - xmin + 1, h = ymax - ymin + 1; // Pixel count is component area; inclusive bounds require +1 for width and height.
            if (area >= c.min_area && area <= c.max_area && double(w) / h >= c.min_aspect &&
                double(w) / h <= c.max_aspect) // Reject components too small, large, narrow, or wide to be the configured marker.
                found.push_back({sx / area, sy / area, xmin, ymin, w, h, area}); // Store centroid, image bounds, and area for later camera-to-bearing association.
        }
    return found; // Return every plausible component; multiple candidates deliberately remain ambiguous.
}

struct Return { // Internal return keeps transformed XY together with bearing and original metric range.
    XY p; // Cartesian location in the calibrated right/forward coordinate frame.
    double angle, range; // Calibrated bearing in degrees and radial distance in meters.
};
std::vector<Group> groups(const RawPoint *points, int count, const Config &c) { // Convert valid returns to the forward/right frame, clip to the metric ROI, then split angle-sorted returns at spatial or angular discontinuities. Each accepted group stores centroid, mean bearing, median range, extent, and count.
    std::vector<Return> valid; // Retain only usable LiDAR returns within the tabletop tracking region.
    for (int i = 0; i < count; ++i) { // Examine each supplied measurement without assuming SDK ordering.
        auto r = points[i]; // Copy one raw return for quality, range, and finite-value checks.
        if (!std::isfinite(r.angle) || !std::isfinite(r.range) || r.range <= 0 || r.quality <= 0 ||
            r.quality < c.min_quality) // Discard invalid ranges/angles and measurements below the quality threshold.
            continue;
        double a = wrap(r.angle - c.forward), t = a * PI / 180; // Rotate sensor-relative azimuth into calibrated forward, then convert to radians.
        XY p{r.range * std::sin(t), r.range * std::cos(t)}; // With forward at zero, sine gives rightward x and cosine gives forward y.
        if (p.x >= c.xmin && p.x <= c.xmax && p.y >= c.ymin && p.y <= c.ymax) // Keep only returns inside the inclusive metric tracking rectangle.
            valid.push_back({p, a, r.range}); // Retain transformed position plus bearing and range for clustering.
    }
    std::sort(valid.begin(), valid.end(),
              [](const Return &a, const Return &b) { return a.angle < b.angle; }); // Order returns by calibrated bearing so adjacent samples can form local clusters.
    std::vector<Group> result; // Store cluster summaries rather than exposing individual returns to association.
    size_t first = 0; // Index of the first return not yet assigned to a candidate cluster.
    while (first < valid.size()) { // Continue until every accepted return has been assigned to a cluster interval.
        size_t end = first + 1; // Start a half-open cluster interval containing at least its first return.
        while (end < valid.size() && distance(valid[end].p, valid[end - 1].p) <= c.cluster_gap &&
               valid[end].angle - valid[end - 1].angle <= c.angle_gap) // Extend only across neighbors that are close in both meters and degrees.
            ++end; // Include the next contiguous return in this candidate cluster.
        int n = int(end - first); // Number of returns in the half-open interval [first, end).
        if (n >= c.min_points) { // Require enough returns to reject isolated noise before calculating a summary.
            XY sum{0, 0}, mn = valid[first].p, mx = mn; // Initialize the centroid sum and Cartesian bounding-box extrema.
            double a = 0; // Accumulate calibrated bearing for an arithmetic mean within the small cluster.
            std::vector<double> ranges; // Collect radial distances so the cluster can report a robust median range.
            for (size_t i = first; i < end; ++i) { // Accumulate statistics over exactly this candidate cluster.
                XY p = valid[i].p; // Read the next transformed return in the cluster.
                sum = add(sum, p); // Accumulate x and y for the mean Cartesian position.
                a += valid[i].angle; // Add the calibrated bearing to the cluster's mean-bearing numerator.
                ranges.push_back(valid[i].range); // Keep this measurement's radial range for median calculation.
                mn.x = std::min(mn.x, p.x); // Expand the cluster's minimum x bound.
                mn.y = std::min(mn.y, p.y); // Expand the cluster's minimum y bound.
                mx.x = std::max(mx.x, p.x); // Expand the cluster's maximum x bound.
                mx.y = std::max(mx.y, p.y); // Expand the cluster's maximum y bound.
            }
            double w = distance(mn, mx); // Bounding-box diagonal is a conservative size measure for rejecting broad surfaces.
            std::sort(ranges.begin(), ranges.end()); // Order ranges before selecting the middle observation or middle pair.
            double median = (ranges[n / 2] + ranges[(n - 1) / 2]) / 2; // Average the two central entries; for odd counts they are the same entry.
            if (w <= c.max_width) // Exclude walls or wide objects that cannot be the configured figure.
                result.push_back({mul(sum, 1. / n), a / n, median, w, n}); // Store centroid, mean bearing, median range, bounding diagonal, and sample count.
        }
        first = end; // Start the next cluster immediately after the one just summarized.
    }
    return result; // Return only accepted clusters for camera-bearing association.
}

struct Engine { // State persists between scans: pending confirmation, filtered position, trail, robot pose, route, yield timer, and semantic hold. No member is shared across threads.
    Config c; // Per-engine copy of settings; live boundary edits update only its explicit exclusion radius.
    Result r{}; // Zero-initialized public snapshot, filled incrementally by tracking and robot decisions.
    std::deque<TrailPoint> trail; // Time-ordered position history supports efficient expiry from the front.
    bool have_position = false, have_pending = false, break_trail = true; // Track filter validity, acquisition-chain validity, and whether the trail must break.
    XY pending{}; // Last unconfirmed measurement used to check consistent reacquisition.
    double pending_at = 0; // Host scan time of that pending measurement, in steady-clock seconds.
    int pending_count = 0, segment = 0; // Consecutive acquisition confirmations and the current discontinuity segment ID.
    bool running = false, narrow = false, ever_started = false; // Delivery run toggle, lane selection, and whether the robot has ever started this run.
    double previous_time = -1, yield_start = -1, healthy_since = -1, clear_since = -1; // Negative sentinels mean the motion, yield, recovery, or sign-clear timer is inactive.
    int last_coral = -1, stop_count = 0; // Last consumed inference sequence and consecutive positive stop-sign observations.
    bool stop_latched = false; // Persistent semantic hold; stale inference cannot release it.
    std::vector<XY> route; // Remaining waypoints exclude the current robot position after planning.
    Engine(const Config &config) : c(config) { // Copy settings and initialize the perception and delivery state for this instance.
        r.selected = -1; // No LiDAR group is associated until the current scan passes every association gate.
        r.scan_number = -1; // Sentinel ensures the first real scan is processed once.
        r.last_seen = -1; // No valid figure position has been observed yet.
        reset(); // Reset delivery pose and timers while preserving independent perception and semantic state.
    }
    void reset() { // Reset delivery state and pose while leaving perception and the stop-sign latch intact. Toggling lane calls this after selecting the lane width.
        running = false; // Disable virtual movement until the next start command.
        ever_started = false; // A freshly reset robot is READY rather than a paused delivery.
        r.robot = c.start; // Place the virtual robot at the configured depot coordinate.
        r.goal = c.goal; // Publish the configured delivery coordinate in the result snapshot.
        r.robot_status = READY; // Show the waiting-to-start delivery state.
        r.reroutes = 0; // Start a new delivery's detour counter at zero.
        r.heading = 0; // Initial display heading points along positive y, the calibrated forward direction.
        route.clear(); // Discard waypoints so the next authorized movement uses freshly validated geometry.
        yield_start = -1; // Clear the obstacle-yield timer; a later blockage starts a new waiting interval.
        healthy_since = -1; // Require a new continuous healthy interval before movement can resume.
    }
    void lost(int status) { // Invalidate the active observation and confirmation chain. A later recovered track starts a separate trail segment instead of drawing through missing data.
        r.track_status = status; // Publish the specific reason this observation cannot authorize motion.
        r.selected = -1; // No LiDAR group is associated until the current scan passes every association gate.
        have_pending = false; // Break the confirmation chain across missing or rejected observations.
        break_trail = true; // Prevent the display from connecting positions across a tracking discontinuity.
    }
    void track(const unsigned char *image, int stride, double camera_at, const RawPoint *points,
               int count, double scan_at, int scan_number, double now) { // Process each new scan once. Reject stale inputs and timestamp mismatch, then require one marker and one bearing-compatible LiDAR group before estimating XY.
        while (!trail.empty() && now - trail.front().stamp > c.trail_s) // Expire history older than trail_s even when the current observation is unusable.
            trail.pop_front(); // Remove the oldest retained position to satisfy the age or capacity bound.
        if (!image || now - camera_at >= c.fresh) { // Missing pixels and stale camera frames both invalidate current tracking.
            lost(STALE_CAMERA); // Report camera staleness and interrupt the acquisition chain.
            return;
        }
        if (scan_number < 0 || now - scan_at >= c.fresh) { // An absent or expired scan cannot provide a current metric figure position.
            lost(STALE_LIDAR); // Hold tracking until a fresh complete scan becomes available.
            return;
        }
        if (scan_number == r.scan_number) // Avoid counting one scan multiple times as the faster application loop runs.
            return;
        r.scan_number = scan_number; // Record that this unique scan is being consumed.
        r.pair_delta = camera_at - scan_at; // Signed receive-time difference; this is host pairing, not hardware synchronization.
        auto ms = markers(image, stride, c); // Find all plausible blue-marker components in the paired camera frame.
        auto gs = groups(points, count, c); // Cluster accepted LiDAR returns in the metric region of interest.
        r.marker_count = int(std::min<size_t>(ms.size(), 16)); // Bound diagnostic marker output to the fixed public array capacity.
        r.group_count = int(std::min<size_t>(gs.size(), 64)); // Bound diagnostic cluster output to the fixed public array capacity.
        std::copy_n(ms.begin(), r.marker_count, r.markers); // Copy only the valid marker prefix; unused result slots are not meaningful.
        std::copy_n(gs.begin(), r.group_count, r.groups); // Copy only the accepted cluster prefix within the result array capacity.
        r.selected = -1; // No LiDAR group is associated until the current scan passes every association gate.
        if (std::abs(r.pair_delta) > c.pair_gap) { // Reject pairs whose receive-time separation exceeds the configured tolerance.
            lost(TIMING_GAP); // Report temporal incompatibility instead of associating unrelated observations.
            return;
        }
        if (ms.empty()) { // LiDAR alone cannot identify the marked prop when no camera marker is visible.
            lost(NO_MARKER); // Report the missing camera identifier and hold virtual movement.
            return;
        }
        if (ms.size() > 1) { // Multiple valid markers cannot be safely assigned to this single-prop tracker.
            lost(MULTIPLE_MARKERS); // Preserve ambiguity rather than selecting an arbitrary image component.
            return;
        }
        double u = ms[0].u; // Interpolate between the adjacent calibration anchors; allow only bounded extrapolation. A second matching group is ambiguous rather than a best-guess target. Use the unique marker's horizontal centroid as the calibrated bearing input.
        if (u < c.anchors[0] - c.extrapolation || u > c.anchors[4] + c.extrapolation) { // Limit extrapolation outside the measured left and right calibration anchors.
            lost(OUTSIDE_CALIBRATION); // Reject image positions beyond the supported calibration interval.
            return;
        }
        int k = u <= c.anchors[2] ? 0 : 2; // Select the left-center or center-right anchor pair for piecewise linear interpolation.
        r.expected_bearing = c.anchors[k + 1] + (u - c.anchors[k]) *
                                                    (c.anchors[k + 3] - c.anchors[k + 1]) /
                                                    (c.anchors[k + 2] - c.anchors[k]); // Convert marker pixel u into expected LiDAR bearing using the selected anchor slope.
        int candidate = -1; // Sentinel means no bearing-compatible cluster has been found yet.
        for (int i = 0; i < r.group_count; ++i) // Compare every published cluster against the camera-derived bearing.
            if (std::abs(gs[i].bearing - r.expected_bearing) <= c.bearing_gate) { // Accept only clusters within bearing_gate degrees of the expected direction.
                if (candidate != -1) { // A second match makes camera/LiDAR association ambiguous.
                    lost(AMBIGUOUS_LIDAR); // Reject competing range clusters rather than guessing the figure's distance.
                    return;
                }
                candidate = i; // Remember the sole matching cluster's index for position and display use.
            }
        if (candidate < 0) { // A camera marker with no compatible range cluster cannot produce fused XY.
            lost(MARKER_ONLY); // Report visual identification without a usable LiDAR association.
            return;
        }
        XY raw = gs[candidate].xy; // Reacquire after a long gap. Otherwise bound observation jumps by a fixed allowance plus elapsed-time motion, then apply a time-dependent low-pass filter. The selected cluster centroid supplies the measured Cartesian position.
        r.raw = raw; // Preserve the unsmoothed position for immediate collision checks as well as diagnostics.
        double age = r.last_seen < 0 ? 1e6 : scan_at - r.last_seen; // Time since the last accepted scan; the sentinel forces initial reacquisition.
        if (age >= c.reacquire) // A long observation gap invalidates the old filter state.
            have_position = false; // Require fresh confirmation before treating a recovered position as tracked.
        if (!have_position) { // Initialize through a confirmation chain rather than trusting one isolated return.
            int n = 1; // Start a new confirmation chain unless this observation agrees with the pending one.
            double dt = scan_at - pending_at; // Measure elapsed scan time for the reacquisition motion allowance.
            if (have_pending && dt > 0 && dt < c.reacquire &&
                distance(raw, pending) <= c.jump + c.max_speed * dt) // Extend confirmation only for timely observations within a plausible motion envelope.
                n = pending_count + 1; // Count this consistent scan as one additional acquisition confirmation.
            have_pending = true; // Retain this observation as the reference for the next confirmation attempt.
            pending = raw; // Save the candidate metric position without smoothing it yet.
            pending_at = scan_at; // Associate the pending position with its scan receive time.
            pending_count = n; // Persist the length of the current consecutive confirmation chain.
            r.selected = candidate; // Publish the associated cluster, including while acquisition is still pending.
            if (n < c.confirmation) { // Hold motion until the configured number of consistent scans has arrived.
                r.track_status = ACQUIRING; // Show that association succeeded but position confirmation is incomplete.
                return;
            }
            r.xy = raw; // Seed the filter at the newly confirmed measured position without interpolation from stale data.
            have_position = true; // Mark the smoothed state initialized for subsequent accepted observations.
        } else {
            if (age <= 0 ||
                distance(raw, r.xy) > c.jump + c.max_speed * std::min(age, c.reacquire)) { // Reject non-forward time or an implausible jump beyond allowance plus speed times age.
                lost(POSITION_JUMP); // Hold motion and break the trail when the candidate violates the motion model.
                return;
            }
            double alpha = 1 - std::exp(-age / c.smoothing); // Convert elapsed scan time and the smoothing time constant into an exponential filter gain.
            r.xy = add(mul(r.xy, 1 - alpha), mul(raw, alpha)); // Blend old estimate and new measurement; irregular scan intervals adjust the gain naturally.
        }
        if (break_trail) // A recovered track begins a new visual history segment.
            ++segment; // Increment the segment ID so rendering does not draw across tracking gaps.
        break_trail = false; // Subsequent uninterrupted observations may connect to this trail segment.
        have_pending = false; // Break the confirmation chain across missing or rejected observations.
        r.selected = candidate; // Publish the associated cluster, including while acquisition is still pending.
        r.last_seen = scan_at; // Advance the accepted-position timestamp only after all tracking gates succeed.
        r.track_status = TRACKING; // This is the sole perception status that permits robot motion.
        trail.push_back({scan_at, r.xy, segment}); // Record the filtered position with scan time and discontinuity segment ID.
        while (trail.size() > 256) // Keep history within the fixed Result trail capacity, independent of scan rate.
            trail.pop_front(); // Remove the oldest retained position to satisfy the age or capacity bound.
    }
    void robot(double now, const Control &control) { // State-machine order: commands, semantic latch, pause/tracking/TPU gates, recovery settling, overlap/goal checks, route validation, then swept movement.
        double dt = previous_time < 0 ? 0 : std::clamp(now - previous_time, 0., .05); // Cap integration at 50 ms so a delayed UI iteration cannot cause a large motion jump.
        previous_time = now; // Advance the integration clock even while the robot is held or paused.
        if (control.command == RESET) // A reset command restores the depot pose and delivery state.
            reset(); // Reset delivery pose and timers while preserving independent perception and semantic state.
        if (control.command == TOGGLE_LANE) { // Changing lane mode resets the delivery before using different geometry.
            narrow = !narrow; // Switch between the configured wide and narrow lane widths.
            reset(); // Reset delivery pose and timers while preserving independent perception and semantic state.
        }
        if (control.command == CLEAR_TRAIL) // Clearing history affects the visualization, not the current pose estimate.
            trail.clear(); // Remove retained trail samples without erasing the current tracked position.
        if (control.command == TOGGLE_RUN) { // Handle the one-shot start/pause command from keyboard or headless startup.
            if (r.robot_status == DELIVERED) // Starting after arrival begins a new delivery from the depot.
                reset(); // Reset delivery pose and timers while preserving independent perception and semantic state.
            running = !running; // Toggle requested motion; subsequent perception and semantic gates still apply.
            ever_started = true; // Distinguish a user-paused delivery from a never-started READY state.
        }
        if (control.coral_healthy && control.coral_sequence != last_coral) { // Count only distinct healthy result sequences. Two positive observations latch the sign; healthy clear observations must persist for 0.6 seconds to release it. Only a fresh, previously unseen inference may contribute a sign confirmation.
            last_coral = control.coral_sequence; // Remember this inference sequence so repeated main-loop reads cannot double-count it.
            if (control.stop_sign) { // Process a new positive semantic observation separately from a clear observation.
                ++stop_count; // Increment the count of consecutive healthy stop-sign detections.
                clear_since = -1; // Interrupt the sign-clear release interval on a positive or unhealthy observation.
                if (stop_count >= 2) // Require two distinct positive inference results before latching the semantic hold.
                    stop_latched = true; // Keep the stop hold active until healthy clear observations satisfy release timing.
            } else {
                stop_count = 0; // Break positive confirmation after a clear or unhealthy result.
                if (clear_since < 0) // Start the release interval only when no healthy-clear interval is already active.
                    clear_since = now; // Record when uninterrupted healthy clear observations began.
                if (now - clear_since >= .6) // Release the semantic latch only after 0.6 seconds of healthy clear evidence.
                    stop_latched = false; // Clear the semantic hold; all other movement gates must still pass.
            }
        } else if (!control.coral_healthy) { // Unavailable inference breaks confirmation and release timing without clearing a latched sign.
            stop_count = 0; // Break positive confirmation after a clear or unhealthy result.
            clear_since = -1; // Interrupt the sign-clear release interval on a positive or unhealthy observation.
        }
        r.coral_held = stop_latched ? 1 : 0; // Expose the persistent sign latch independently from the current inference health.
        r.lane_width = narrow ? c.narrow_width : c.lane_width; // Publish the selected full lane width for visualization and logging.
        r.obstacle_radius = exclusion_radius(c); // Publish the exact selected exclusion radius used by collision checking.
        r.narrow = narrow ? 1 : 0; // Convert lane selection to the integer flag used by the fixed-layout result.
        r.running = running ? 1 : 0; // Publish the user's run request even when safety gates temporarily hold movement.
        if (!running) { // A paused or unstarted robot cannot advance along its route.
            if (r.robot_status != DELIVERED) // Preserve the completed-delivery display when automatic arrival stops the run.
                r.robot_status = ever_started ? PAUSED : READY; // Distinguish an explicitly paused run from initial readiness.
            return;
        }
        if (r.track_status != TRACKING) { // Require a currently valid fused track before any virtual movement.
            r.robot_status = TRACK_LOST; // Surface the perception hold in the robot's displayed state.
            healthy_since = -1; // Require a new continuous healthy interval before movement can resume.
            return;
        }
        if (control.coral_required && !control.coral_healthy) { // Required semantic sensing must be available and fresh before movement resumes.
            r.robot_status = TPU_OFFLINE; // Explain that movement is held by inference availability or freshness.
            healthy_since = -1; // Require a new continuous healthy interval before movement can resume.
            return;
        }
        if (control.coral_required && stop_latched) { // Apply a confirmed stop-sign hold only when semantic sensing is enabled.
            r.robot_status = TPU_STOP; // Report the semantic stop independently from physical path blockage.
            healthy_since = -1; // Require a new continuous healthy interval before movement can resume.
            return;
        }
        if (healthy_since < 0) // Begin a recovery-settling interval after all required sensing gates pass.
            healthy_since = now; // Record the start of continuously usable perception and semantic state.
        if (now - healthy_since < .25) { // Require 250 ms of stable recovery before advancing the robot again.
            r.robot_status = YIELDING; // Show a temporary hold for recovery, obstacle waiting, or a newly blocked movement.
            return;
        }
        Space space(c, narrow, {r.raw, r.xy}); // Both raw and filtered positions block motion: filtering cannot hide a newly intruding figure. This is a one-prop, virtual-lane model. Check both raw and filtered figure positions so smoothing cannot conceal a new intrusion.
        if (!space.clear(r.robot, r.robot)) { // A figure boundary already overlapping the robot requires an immediate hold.
            r.robot_status = TOO_CLOSE; // Explain the overlap condition; moving the prop away can release this hold.
            yield_start = now; // Restart obstacle waiting at this newly detected conflict.
            route.clear(); // Discard waypoints so the next authorized movement uses freshly validated geometry.
            return;
        }
        if (distance(r.robot, c.goal) < .002) { // Treat a position within 2 mm of the delivery point as arrival.
            r.robot = c.goal; // Snap the tiny remaining arrival error to the exact configured goal.
            r.robot_status = DELIVERED; // Publish successful completion of the virtual delivery.
            running = false; // Disable virtual movement until the next start command.
            r.running = 0; // Reflect automatic stopping at the goal in the public result.
            return;
        }
        bool route_clear = !route.empty(); // Keep a route while all remaining segments pass hard collision checks. A blocked direct route yields before planning; no feasible route means hold. An empty route requires planning; an existing route starts provisionally valid.
        XY last = r.robot; // The first remaining route segment begins at the current robot center.
        for (XY p : route) { // Revalidate every remaining segment against current raw and filtered obstacles.
            if (!space.clear(last, p)) // A conflict anywhere ahead invalidates the existing route.
                route_clear = false; // Request replanning rather than following a route through an updated obstacle.
            last = p; // Advance the segment origin for the next waypoint check.
        }
        if (!route_clear) { // Replace only invalid routes, reducing unnecessary switching caused by measurement noise.
            route.clear(); // Discard waypoints so the next authorized movement uses freshly validated geometry.
            if (planning_space(space, r.robot, c.goal).clear(r.robot, c.goal)) { // Prefer a direct path only when it also meets the selected extra-clearance preference.
                route = {c.goal}; // Store the goal as the sole remaining waypoint for direct travel.
                yield_start = -1; // Clear the obstacle-yield timer; a later blockage starts a new waiting interval.
            } else {
                if (yield_start < 0) // Start waiting once for this blockage rather than resetting the timer every iteration.
                    yield_start = now; // Restart obstacle waiting at this newly detected conflict.
                if (now - yield_start < c.yield_s) { // Give the moving figure time to clear before committing to a detour.
                    r.robot_status = YIELDING; // Show a temporary hold for recovery, obstacle waiting, or a newly blocked movement.
                    return;
                }
                auto planned = stable_plan(c, r.robot, c.goal, space); // Search for a continuously valid route, preferring additional obstacle clearance.
                if (planned.empty()) { // No feasible route means the robot must remain stopped.
                    r.robot_status = BLOCKED; // Report that no path fits the lane and current figure boundary.
                    return;
                }
                route.assign(planned.begin() + 1, planned.end()); // Exclude the initial robot position and retain only future waypoints.
                if (planned.size() > 2) // Count a detour only when the path contains an intermediate waypoint.
                    ++r.reroutes; // Increment the delivery's detour counter for the newly selected indirect route.
                yield_start = -1; // Clear the obstacle-yield timer; a later blockage starts a new waiting interval.
            }
        }
        r.robot_status = (route.size() > 1 || r.reroutes > 0) ? DETOUR : DRIVING; // Label indirect travel, retaining the detour label after a reroute has occurred.
        double remaining = c.speed * c.speed_scale * dt; // Apply the pacing fraction once to this movement budget; keep stored speed unchanged so calibration saves cannot compound the slowdown. Collision checks still guard each segment.
        double desired_heading = r.heading; // Keep the existing heading unless actual motion establishes a new direction.
        bool moved = false; // Track whether display-heading interpolation should occur this iteration.
        while (remaining > 1e-9 && !route.empty()) { // Consume the travel budget across as many short waypoints as necessary.
            XY target = route.front(); // Select the next remaining waypoint as the current motion target.
            double d = distance(target, r.robot); // Measure remaining distance to this waypoint in meters.
            if (d < 1e-9) { // Discard a numerically coincident waypoint without dividing by its near-zero distance.
                route.erase(route.begin()); // Remove the reached waypoint so motion can proceed to the next one.
                continue;
            }
            double travel = std::min(d, remaining); // Limit this segment's movement to both waypoint distance and remaining time budget.
            XY next = add(r.robot, mul(sub(target, r.robot), travel / d)); // Interpolate along the current segment by the permitted metric travel distance.
            if (!space.clear(r.robot, next)) { // Recheck the actual swept movement immediately before changing the robot pose.
                r.robot_status = YIELDING; // Show a temporary hold for recovery, obstacle waiting, or a newly blocked movement.
                route.clear(); // Discard waypoints so the next authorized movement uses freshly validated geometry.
                yield_start = now; // Restart obstacle waiting at this newly detected conflict.
                break;
            }
            desired_heading = std::atan2(next.x - r.robot.x, next.y - r.robot.y); // Measure direction from forward (+y), so atan2 receives x displacement first.
            moved = true; // Mark that a nonzero valid movement provides a new display-heading target.
            r.robot = next; // Commit the continuously checked virtual robot-center position.
            remaining -= travel; // Deduct consumed distance from this iteration's movement budget.
            if (travel >= d - 1e-9) // Remove a waypoint reached within numerical tolerance.
                route.erase(route.begin()); // Remove the reached waypoint so motion can proceed to the next one.
        }
        if (moved) { // Rotate the displayed robot only when this iteration changed its position.
            double delta = std::atan2(std::sin(desired_heading - r.heading),
                                      std::cos(desired_heading - r.heading)); // The virtual footprint is circular, so display heading can turn smoothly without changing the swept-position collision check. Wrap heading error to the shortest signed rotation in radians.
            r.heading += std::clamp(delta, -3. * dt, 3. * dt); // Limit display rotation to 3 radians per second using the capped integration interval.
            r.heading = std::atan2(std::sin(r.heading), std::cos(r.heading)); // Normalize the accumulated display heading after the limited turn.
        }
    }
    void finish(Result &out) { // Copy bounded diagnostic arrays into the public Result. Counts specify the valid prefixes; consumers must not interpret unused array entries.
        r.trail_count = int(trail.size()); // Report the valid number of trail entries before copying them into the fixed array.
        std::copy(trail.begin(), trail.end(), r.trail); // Copy history in chronological order, preserving segment breaks for rendering.
        r.path_count = int(std::min<size_t>(route.size(), 128)); // Bound visible route length to the fixed result capacity of 128 waypoints.
        std::copy_n(route.begin(), r.path_count, r.path); // Copy only the visible waypoint prefix; the internal route remains complete.
        out = r; // Publish one coherent engine result after tracking and motion decisions finish.
    }
};
} // namespace

int core_abi() { // Expose the fixed-layout interface identifier and structure sizes for native consumers that validate data contracts before calling the engine.
    return 3; // Config includes the boundary radius and speed scale, so native consumers must use the matching layout.
}
int core_size(int which) {
    switch (which) { // Select the requested public structure size for native contract checks.
    case 0: // Selector zero reports the configuration structure size.
        return sizeof(Config); // Return the compiler's actual byte size, including any ABI padding.
    case 1: // Selector one reports the result snapshot size.
        return sizeof(Result); // Return the compiler's actual byte size, including any ABI padding.
    case 2: // Selector two reports the command and semantic-input size.
        return sizeof(Control); // Return the compiler's actual byte size, including any ABI padding.
    case 3: // Selector three reports the raw LiDAR return size.
        return sizeof(RawPoint); // Return the compiler's actual byte size, including any ABI padding.
    default:
        return 0;
    }
}
void *core_create(const Config *c) { // Allocate one isolated engine; failure is represented by null. Destroying a null handle is allowed. Exceptions never cross the exported C-shaped interface.
    try {
        return c && valid_config(*c) ? new Engine(*c) : nullptr; // Construct an independent engine only for a non-null, minimally valid configuration.
    } catch (...) {
        return nullptr;
    }
}
void core_destroy(void *core) {
    delete static_cast<Engine *>(core); // Recover the opaque handle's concrete type and destroy its owned state; null is allowed.
}
int core_set_boundary(void *core, double radius) { // Select a total 3–10 cm exclusion radius; invalidate the route while preserving pose, tracking, and hold state.
    if (!core || !valid_boundary_radius(radius)) // Reject missing engines and invalid radii before mutating any geometry or route state.
        return -1;
    auto &e = *static_cast<Engine *>(core); // Access the engine behind the opaque public handle on the calling thread.
    if (std::abs(radius - exclusion_radius(e.c)) < 1e-12) // Treat an effectively unchanged boundary as a no-op, preserving the current route.
        return 0;
    e.c.boundary_radius = radius; // Store the user's total exclusion radius independently of the nominal body sizes and margin.
    e.r.obstacle_radius = radius; // Update the reported total radius immediately for consistent UI feedback.
    e.route.clear(); // Invalidate waypoints planned for the previous boundary geometry.
    e.r.path_count = 0; // Clear the public path prefix so stale geometry is not displayed.
    e.yield_start = -1; // Start a fresh yield interval if the new geometry blocks the next movement.
    return 0;
}
int core_step(void *core, const unsigned char *bgr, int width, int height, int stride,
              double camera_at, const RawPoint *points, int count, double scan_at, int scan_number,
              double now, const Control *control, Result *result) { // Validate input dimensions and pointer/count pairs. Return 0 on success, -1 for invalid arguments, or -2 for an internal exception; always track before motion.
    if (!core || !control || !result || width != 640 || height != 480 || stride < 1920 ||
        count < 0 || count > 16384 || (!points && count > 0) || !std::isfinite(now) ||
        !std::isfinite(camera_at) || !std::isfinite(scan_at)) // Validate buffer dimensions, pointer/count pairs, and finite timestamps before dereferencing.
        return -1;
    try {
        auto &e = *static_cast<Engine *>(core); // Access the engine behind the opaque public handle on the calling thread.
        e.track(bgr, stride, camera_at, points, count, scan_at, scan_number, now); // Update fused perception first so robot decisions consume the current observation state.
        e.robot(now, *control); // Apply commands, health gates, and collision-checked virtual movement after tracking.
        e.finish(*result); // Copy bounded diagnostics and the final robot state to the caller's result structure.
        return 0;
    } catch (...) {
        return -2;
    }
}
int core_plan(const Config *c, XY start, XY goal, const XY *obstacles, int n, int narrow, XY *path,
              int capacity) { // Stateless planning entry point: path includes start and goal. Return its length, 0 for no route, or negative values for invalid input/capacity/exception.
    if (!c || !valid_config(*c) || !path || capacity < 1 || n < 0 || n > 1000 || (!obstacles && n)) // Reject invalid planner settings, output capacity, and obstacle pointer/count combinations.
        return -1;
    try {
        std::vector<XY> obs; // Own a temporary obstacle list for this stateless planning call.
        if (n) // Copy obstacles only when a positive count makes the source range meaningful.
            obs.assign(obstacles, obstacles + n); // Copy the caller's obstacle coordinates into the planner-owned list.
        auto route = stable_plan(*c, start, goal, Space(*c, narrow != 0, std::move(obs))); // Build collision space for the chosen lane and search with graduated clearance preference.
        if (int(route.size()) > capacity) // Fail without copying when the caller's output array cannot hold the complete route.
            return -2;
        std::copy(route.begin(), route.end(), path); // Copy the entire accepted route after checking output capacity.
        return int(route.size()); // Report the number of waypoints; zero represents no feasible route.
    } catch (...) {
        return -3;
    }
}
