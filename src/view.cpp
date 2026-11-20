/**
 * @file
 * OpenCV presentation, input widgets, and diagnostic session output.
 * Coordinates are converted to pixels only here. The display does not decide
 * whether motion is allowed; it renders the Result produced by the core.
 */

#include "app.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip> // Controlled numeric formatting for display text or serialized diagnostics.
#include <sstream> // In-memory text formatting for labels, paths, and test diagnostics.
#include <ctime> // Wall-clock calendar formatting for human-readable session directory names.
namespace tabletop {
namespace {
const cv::Scalar white(225, 230, 232), muted(145, 153, 158), cyan(225, 195, 80),
    green(140, 225, 135), amber(90, 190, 245), purple(225, 155, 225); // Central BGR palette shared by camera overlays, robot states, and display controls.
const cv::Rect boundary_minus(228, 718, 38, 36), boundary_plus(375, 718, 38, 36),
    boundary_save(865, 718, 90, 36); // Fixed pixel hit boxes for decrement, increment, and persistence controls.
const cv::Rect boundary_slider(430, 718, 405, 36); // Slider hit box is wider than the visible track to make dragging easier.
constexpr int slider_left = 445, slider_right = 820, slider_y = 736; // Visible slider endpoints and centerline in the fixed 1280x800 canvas coordinates.
std::string fixed(double x, int precision = 1) { // Format one numeric display value without changing the stored metric quantity.
    std::ostringstream s; // Build local formatted text without altering global stream formatting.
    s << std::fixed << std::setprecision(precision) << x; // Use a fixed number of fractional digits for readable lengths, ages, and timings.
    return s.str(); // Return an owned label string for drawing or composition.
}
void text(cv::Mat &image, const std::string &value, int x, int y, double scale = .5,
          cv::Scalar color = white, int weight = 1) { // Draw antialiased text using a consistent font, baseline position, and default palette.
    cv::putText(image, value, {x, y}, cv::FONT_HERSHEY_SIMPLEX, scale, color, weight, cv::LINE_AA); // Apply the requested label, scale, thickness, and BGR color to the canvas.
}
cv::Point map_point(XY xy, int shift = 0) { // Convert meters to display pixels, with forward up and right positive. Optional fixed-point bits let OpenCV draw slow motion without integer-pixel stepping.
    const double k = double(1 << shift); // OpenCV fixed-point drawing represents each pixel using 2^shift coordinate units.
    return {int(std::lround((1040 + xy.x * 460) * k)), int(std::lround((535 - xy.y * 460) * k))}; // Map meters at 460 pixels/m, with right increasing x and forward decreasing screen y.
}
void line(cv::Mat &im, XY a, XY b, cv::Scalar color, int weight = 1) { // Draw a metric segment through the shared map transform.
    cv::line(im, map_point(a, 8), map_point(b, 8), color, weight, cv::LINE_AA, 8); // Preserve eight fractional coordinate bits to reduce visible quantization during slow movement.
}
void circle(cv::Mat &im, XY p, double radius, cv::Scalar color, int weight = 1) { // Draw a metric radius around a calibrated XY position on the map.
    cv::circle(im, map_point(p, 8), int(std::lround(radius * 460 * 256)), color, weight,
               cv::LINE_AA, 8); // Convert radius to pixels and then 8-bit fixed-point units, matching the center's transform.
}
void state_file(const std::filesystem::path &path, const Frame &frame, const Scan &scan,
                const Result &r, const CoralState &coral) { // Serialize a diagnostic snapshot, including validity flags. XY values are only meaningful when their accompanying tracking/raw validity flag is true.
    cv::FileStorage f(path.string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON); // Open the snapshot state as structured JSON for later inspection alongside its images/scan.
    if (!f.isOpened()) // Fail the snapshot explicitly if its structured state file cannot be created.
        throw std::runtime_error("Cannot write snapshot state");
    f << "tracking" << track_name(r.track_status) << "robot_status" << robot_name(r.robot_status) // Serialize pose, validity flags, geometry, source timestamps, and the remaining path together.
      << "robot_xy_m" << "[" << r.robot.x << r.robot.y << "]" << "figure_surface_xy_m" << "["
      << r.xy.x << r.xy.y << "]"
      << "figure_valid" << int(r.track_status == TRACKING) << "raw_xy_m" << "[" << r.raw.x
      << r.raw.y << "]"
      << "raw_valid" << int(r.selected >= 0) << "reroutes" << r.reroutes << "lane_width_m"
      << r.lane_width << "boundary_radius_m" << r.obstacle_radius << "camera_received_s"
      << frame.received << "camera_sequence" << std::to_string(frame.sequence)
      << "camera_sensor_timestamp_ns" << std::to_string(frame.sensor_ns) << "lidar_received_s"
      << scan.received << "scan_sequence" << scan.sequence << "pair_delta_s" << r.pair_delta
      << "path_m" << "[";
    for (int i = 0; i < r.path_count; ++i) // Use only the valid route prefix reported by the core.
        f << "[" << r.path[i].x << r.path[i].y << "]"; // Write one remaining [x,y] waypoint in meters.
    f << "]" << "coral" << "{" << "ready" << int(coral.ready) << "healthy" << int(coral.healthy) // Append the independent inference result, freshness status, timing, and detection array.
      << "sequence" << coral.sequence << "stop_sign" << int(coral.stop_sign) << "inference_ms"
      << coral.inference_ms << "detail" << coral.detail << "source_frame_received_s"
      << coral.frame_time << "result_received_s" << coral.received << "detections" << "[";
    for (const auto &d : coral.detections) // Serialize each accepted inference detection with its source result's metadata.
        f << "{" << "class_id" << d.id << "score" << d.score << "box_xyxy_normalized" << "[" << d.x0
          << d.y0 << d.x1 << d.y1 << "]" << "}"; // Store class, confidence, and normalized [left,top,right,bottom] image coordinates.
    f << "]" << "}"; // Close the detection sequence and enclosing Coral object.
}
} // namespace
BoundaryUi::BoundaryUi(const Config &c) // Use integer millimeters so all controls enforce an exact 3–10 cm range with 0.1 cm steps.
    : minimum_mm(BOUNDARY_MIN_MM), maximum_mm(BOUNDARY_MAX_MM), // Share fixed inclusive bounds with configuration validation and the collision engine.
      current_mm(std::clamp(int(std::lround(exclusion_radius(c) * 1000)), minimum_mm, maximum_mm)), // Initialize the thumb from the validated effective radius in integer millimeters.
      requested_mm(current_mm.load()) {} // Begin with no pending geometry change: requested and applied values agree.
void BoundaryUi::adjust(int delta) { // Clamp an atomic adjustment so keyboard/mouse callbacks only request a value. The main loop owns the actual engine and configuration mutation.
    int before = requested_mm.load(); // Read the requested radius before attempting an atomic bounded increment/decrement.
    while (!requested_mm.compare_exchange_weak(
        before, std::clamp(before + delta, minimum_mm, maximum_mm))) { // Retry if another callback changed the value; failed compare-exchange refreshes before automatically.
    }
}
void BoundaryUi::mouse(int event, int x, int y, int flags) { // Hit-test buttons and slider. Dragging maps a clamped screen fraction to millimeters; saving only posts a request for main-thread persistence.
    const cv::Point p{x, y}; // Convert event coordinates into the same pixel type used by control hit boxes.
    if (event == cv::EVENT_LBUTTONDOWN) { // Buttons and initial slider grabs respond only to a new left-button press.
        if (boundary_minus.contains(p)) { // A press inside the minus button requests one millimeter less clearance radius.
            adjust(-1); // Apply the same bounded decrement used by the left-bracket keyboard shortcut.
            return;
        }
        if (boundary_plus.contains(p)) { // A press inside the plus button requests one millimeter more radius.
            adjust(1); // Apply the same bounded increment used by the right-bracket keyboard shortcut.
            return;
        }
        if (boundary_save.contains(p)) { // The Save button requests persistence without writing a file inside the callback.
            save_requested = true; // Let the main thread save the next applied radius before continuing its regular loop.
            return;
        }
        if (boundary_slider.contains(p)) // A click in the slider area begins a drag and also chooses an initial radius.
            dragging = true; // Remember that subsequent held-button mouse moves belong to this slider gesture.
    }
    if (event == cv::EVENT_LBUTTONUP) { // Releasing the button ends dragging regardless of cursor position.
        dragging = false; // Clear the gesture state so unpressed mouse motion cannot change the radius.
        return;
    }
    if (dragging && (event == cv::EVENT_LBUTTONDOWN ||
                     (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)))) { // Update only during a slider gesture with an initial click or left-button-held movement.
        const double fraction =
            std::clamp(double(x - slider_left) / (slider_right - slider_left), 0., 1.); // Convert the current mouse/applied-radius location into a normalized, clamped slider fraction.
        requested_mm = minimum_mm + int(std::lround(fraction * (maximum_mm - minimum_mm))); // Map the pointer fraction into integer millimeters across the configured slider bounds.
    }
    if (event == cv::EVENT_MOUSEMOVE && !(flags & cv::EVENT_FLAG_LBUTTON)) // Recover from a release outside the window when mouse motion arrives with no left button held.
        dragging = false; // Clear the gesture state so unpressed mouse motion cannot change the radius.
}
cv::Mat render(const Config &c, const Inputs &inputs, const Frame &frame, const Result &r,
               const CoralState &coral, bool coral_required, bool demo, bool mask, double now,
               double step_ms, const BoundaryUi *boundary, const EvaluationDisplay *evaluation) { // Compose camera, metric map, status, timing, and controls from one snapshot. Rendering does not change tracking or planner state and can run headlessly.
    cv::Mat image(800, 1280, CV_8UC3, cv::Scalar(22, 25, 28)); // Allocate the fixed-size BGR canvas with the common dark background color.
    text(image, "TABLETOP DELIVERY", 18, 32, .8, white, 2);
    text(image,
         demo ? "SIMULATION | CORAL OFF"
              : (coral_required ? "LIVE SENSORS | CORAL USB" : "LIVE SENSORS | CORAL DISABLED"),
         770, 30, .51, cyan);
    const bool yolo_controls = evaluation && evaluation->yolo_controls; // Detector control reuses candidate storage for YOLO boxes; it does not produce a second visual detection.
    const char *camera_heading = !evaluation ? "CAMERA + BLUE MARKER"
        : !yolo_controls ? "CAMERA + MARKER | YOLO COMPARISON"
        : evaluation->figure_model ? "CAMERA + YOLO FIGURE | EVALUATION"
        : "CAMERA + YOLO PERSON | EVALUATION"; // Match the heading to the active model's target class.
    text(image, camera_heading, 18, 64, .5, muted); // Describe which visual source controls the virtual robot.
    if (frame) { // Only wrap and display camera pixels when the frame contract is valid.
        cv::Mat camera(480, 640, CV_8UC3, const_cast<unsigned char *>(frame.pixels->data())); // Borrow the immutable packed BGR storage for read-only display/preprocessing operations.
        camera.copyTo(image(cv::Rect(18, 80, 640, 480))); // Copy pixels into the canvas so overlays never modify the shared sensor frame.
        if (mask && !yolo_controls) { // Replace the camera panel with a threshold preview when diagnostic mask mode is enabled.
            cv::Mat hsv, binary, colored; // Own HSV, single-channel threshold, and three-channel display intermediates.
            cv::cvtColor(camera, hsv, cv::COLOR_BGR2HSV); // Convert only the visualization input into OpenCV HSV for the diagnostic threshold preview.
            cv::inRange(hsv, cv::Scalar(c.hsv_low[0], c.hsv_low[1], c.hsv_low[2]),
                        cv::Scalar(c.hsv_high[0], c.hsv_high[1], c.hsv_high[2]), binary); // Apply the configured inclusive HSV limits to visualize candidate marker-colored pixels.
            cv::cvtColor(binary, colored, cv::COLOR_GRAY2BGR); // Expand the single-channel binary mask to BGR for the common display canvas.
            colored.copyTo(image(cv::Rect(18, 80, 640, 480))); // Replace the camera panel with the diagnostic mask without changing core processing input.
        }
        cv::rectangle(image, {18 + c.roi[0], 80 + c.roi[1]}, {18 + c.roi[2] - 1, 80 + c.roi[3] - 1},
                      cv::Scalar(85, 85, 60), 1); // Draw the calibrated camera ROI offset into the camera panel; upper limits are exclusive.
        if (!yolo_controls && r.track_status != STALE_CAMERA && r.track_status != STALE_LIDAR &&
            r.track_status != TIMING_GAP) // Draw green candidates only for marker control; detector candidates already receive one labeled purple box below.
            for (int i = 0; i < r.marker_count; ++i) { // Draw only marker candidates within the core's bounded valid output prefix.
                const auto &m = r.markers[i]; // Read this candidate's image bounds without copying the result structure.
                cv::rectangle(image, cv::Rect(18 + m.x, 80 + m.y, m.w, m.h), green, 2); // Offset the marker's camera-pixel rectangle into its display panel.
                text(image, "BLUE MARKER", 18 + m.x, std::max(95, 75 + m.y), .4, green); // This rectangle represents the independent color-marker observation, never a duplicate detector box.
            }
    } else
        text(image, "Waiting for camera...", 140, 300, .7, amber); // Coral results belong to an earlier frame: show their age and avoid drawing stale boxes over a newer image as if spatially synchronized.
    text(image, "LIDAR MAP / FORWARD UP", 705, 64, .5, muted); // Draw all visible returns as context, then overlay the configured lane and associated figure/route. Unassociated points are not an occupancy map.
    for (int x = -30; x <= 30; x += 10) { // Draw vertical map grid lines every 10 cm from -30 cm to +30 cm.
        line(image, {x / 100., 0}, {x / 100., 1.}, cv::Scalar(59, 65, 70)); // Convert centimeter tick locations to meters before using the map drawing helper.
        text(image, std::to_string(x), map_point({x / 100., 0}).x - 12, 556, .35, muted);
    }
    for (int y = 0; y <= 100; y += 10) { // Draw horizontal map grid lines over the visible zero-to-one-meter range.
        line(image, {-.32, y / 100.}, {.32, y / 100.}, cv::Scalar(59, 65, 70)); // Draw each horizontal grid line across the visible map width.
        if (y % 20 == 0) // Label every other horizontal grid line to keep y-axis text readable.
            text(image, std::to_string(y), 856, map_point({0, y / 100.}).y + 4, .35, muted);
    }
    text(image, "x (cm)", 1020, 579, .4, muted);
    text(image, "cm", 858, 97, .35, muted);
    for (const auto &point : inputs.scan.points) { // Show raw scan returns as contextual points, not as a validated occupancy map.
        if (point.quality < c.min_quality) // Omit returns below the configured quality floor from the context display.
            continue;
        const double angle = (point.angle - c.forward) * 3.141592653589793 / 180; // Rotate the raw sensor azimuth into the same calibrated forward frame as the core.
        XY p{point.range * std::sin(angle), point.range * std::cos(angle)}; // Convert polar range to rightward x and forward y for map placement.
        if (p.x < -.32 || p.x > .32 || p.y < 0 || p.y > 1) // Clip contextual returns to the visible map's metric extent.
            continue;
        circle(image, p, .0018, cv::Scalar(120, 125, 120), -1); // Draw a small filled context return without using it to authorize robot movement.
    }
    double w = r.lane_width > 0 ? r.lane_width : c.lane_width; // Prefer the engine's active lane width; use configured width before the first result is populated.
    for (double x : {-w / 2, w / 2}) // Draw the two side walls at half the active full lane width.
        line(image, {x, c.lane_ymin}, {x, c.lane_ymax}, cyan); // Draw one lane side between the configured near and far boundaries.
    for (double y : {c.lane_ymin, c.lane_ymax}) // Draw the near and far ends of the selected virtual lane.
        line(image, {-w / 2, y}, {w / 2, y}, cyan); // Connect lane sides across one configured endpoint boundary.
    for (int i = 1; i < r.trail_count; ++i) // Consider each consecutive pair of valid history samples.
        if (r.trail[i - 1].segment == r.trail[i].segment) // Connect history only when both samples belong to the same uninterrupted track segment.
            line(image, r.trail[i - 1].xy, r.trail[i].xy, cv::Scalar(95, 90, 55), 2); // Draw the accepted filtered-position trail without bridging observation gaps.
    XY previous = r.robot; // Begin remaining-path visualization at the actual current robot position.
    for (int i = 0; i < r.path_count; ++i) { // Use only the valid route prefix reported by the core.
        line(image, previous, r.path[i], green, 2); // Draw the next validated waypoint segment in the route color.
        previous = r.path[i]; // Advance the route drawing origin to the waypoint just drawn.
    }
    circle(image, c.start, .009, muted); // Mark the configured depot location with a small outlined circle.
    circle(image, c.goal, .012, green, 2); // Mark the delivery location with a larger highlighted circle.
    auto goal = map_point(c.goal); // Convert the delivery position once for nearby text placement.
    text(image, "DELIVERY", goal.x + 14, goal.y + 4, .35, green);
    text(image, "DEPOT", map_point(c.start).x + 14, map_point(c.start).y + 4, .35, muted);
    if (r.track_status == TRACKING) { // Show position-dependent figure geometry/text only while the fused track is valid.
        circle(image, r.xy, r.obstacle_radius, amber); // Draw the exact combined radius used by collision checking, centered on the filtered position.
        circle(image, r.xy, .014, cyan, 2); // Highlight the filtered figure position independently of its exclusion boundary.
        circle(image, r.raw, .004, white, -1); // Show the current unsmoothed measurement that also participates in collision checking.
    }
    const double sn = std::sin(r.heading), cs = std::cos(r.heading); // Cache sine/cosine of the displayed robot heading for footprint rotation.
    cv::Point corners[4]; // Store the four fixed-point screen vertices of the decorative robot body.
    const XY local[]{{-.015, -.022}, {.015, -.022}, {.015, .022}, {-.015, .022}}; // Local robot artwork corners in meters; collision checking uses the separate circular footprint.
    for (int i = 0; i < 4; ++i) // Rotate and translate each artwork corner into the world map frame.
        corners[i] = map_point({r.robot.x + local[i].x * cs + local[i].y * sn,
                                r.robot.y - local[i].x * sn + local[i].y * cs},
                               8); // Apply heading measured from +y, then convert to eight-bit-fraction screen coordinates.
    const bool moving = r.robot_status == DRIVING || r.robot_status == DETOUR; // Choose the moving visual style only for the core's two travel states.
    cv::fillConvexPoly(image, corners, 4, moving ? cyan : amber, cv::LINE_AA, 8); // Fill the oriented robot artwork using the same fixed-point scale as the corners.
    line(image, r.robot, {r.robot.x + .023 * sn, r.robot.y + .023 * cs}, cv::Scalar(20, 30, 30), 2); // Draw a short heading indicator from the robot center toward its displayed forward direction.
    text(image, evaluation && evaluation->yolo_controls ? detector_track_name(r.track_status, evaluation->figure_model) : track_name(r.track_status), 705, 110, .53,
         r.track_status == TRACKING ? green : amber, 2);
    if (r.track_status == TRACKING) { // Show position-dependent figure geometry/text only while the fused track is valid.
        text(image, "x " + fixed(r.xy.x * 100) + " cm", 705, 148, .65, cyan);
        text(image, "y " + fixed(r.xy.y * 100) + " cm", 705, 179, .65, cyan);
        text(image, "observed surface", 705, 203, .4, muted);
        if (r.selected >= 0) // Report cluster support only when association selected a valid LiDAR group.
            text(image, std::to_string(r.groups[r.selected].count) + " LiDAR returns", 705, 239,
                 .45);
    }
    text(image, "RECEIVE AGES", 705, 305, .43, muted);
    text(image,
         frame ? "Camera " + fixed((now - frame.received) * 1000, 0) + " ms" : "Camera waiting",
         705, 331, .43);
    text(image,
         inputs.scan.sequence >= 0
             ? "LiDAR " + fixed((now - inputs.scan.received) * 1000, 0) + " ms"
             : "LiDAR waiting",
         705, 355, .43);
    text(image, "Pair " + fixed(r.pair_delta * 1000, 0) + " ms", 705, 379, .43);
    text(image, "Host receive pairing", 705, 415, .35, muted);
    text(image, "not hardware sync", 705, 435, .35, muted);
    if(evaluation) {
        text(image,std::string(evaluation->figure_model ? "Figures " : "People ")+std::to_string(evaluation->people)+" / fit "+std::to_string(evaluation->eligible),705,463,.35,cyan);
        text(image,"Best score: "+fixed(evaluation->confidence,2),705,483,.35,cyan);
        text(image,detector_track_name(evaluation->yolo.track_status, evaluation->figure_model),705,503,.32,amber);
        if(evaluation->marker.track_status==TRACKING && evaluation->yolo.track_status==TRACKING)
            text(image,"Delta "+fixed(std::hypot(evaluation->marker.xy.x-evaluation->yolo.xy.x,evaluation->marker.xy.y-evaluation->yolo.xy.y)*100)+" cm",705,523,.35,cyan);
        if(frame && frame.sequence==coral.source_frame.sequence) for(const auto &d:coral.detections) {
            if(d.id!=0 && d.id!=(coral.figure_model ? 1 : 11)) continue; // Use the selected model's labels on the exact inference frame.
            cv::rectangle(image,{18+int(d.x0*640),80+int(d.y0*480)},{18+int(d.x1*640),80+int(d.y1*480)},purple,1);
            text(image,std::string(d.id==0?(coral.figure_model ? "figure " : "person "):"stop ")+fixed(d.score,2),18+int(d.x0*640),std::max(95,75+int(d.y0*480)),.37,purple);
        }
    }
    text(image, std::string("ROBOT: ") + robot_name(r.robot_status), 18, 610, .8,
         moving ? green : amber, 2);
    text(image,
         std::string(r.narrow ? "NARROW LANE" : "WIDE LANE") +
             "  |  reroutes: " + std::to_string(r.reroutes),
         735, 609, .5);
    const char *hint = "SPACE starts delivery. Move the figure slowly across the lane."; // Default user guidance applies while no more specific delivery state needs explanation.
    if (r.robot_status == TOO_CLOSE) // Explain how to release a current footprint overlap without changing planner behavior.
        hint = "Figure buffer overlaps robot. Move the figure away to release it.";
    else if (r.robot_status == TRACK_LOST) // Explain the sensing evidence needed before virtual movement can resume.
        hint = evaluation && evaluation->yolo_controls ? "Waiting for one fresh target detection with an unambiguous LiDAR match." : "Waiting for one fresh blue marker with an unambiguous LiDAR match.";
    else if (r.robot_status == BLOCKED) // Explain that no route fits the lane's current geometry.
        hint = "No route fits in this lane. Move the figure aside.";
    else if (r.robot_status == YIELDING) // Describe the temporary wait before recovery or detour selection.
        hint = "Yielding briefly before choosing a detour.";
    else if (r.robot_status == TPU_OFFLINE) // Identify inference freshness/availability as the reason movement is held.
        hint = "Coral result unavailable or stale. Robot motion is held.";
    else if (r.robot_status == TPU_STOP) // Identify the semantic stop-sign hold and its release action.
        hint = "Coral detected a stop sign. Remove it to release the hold.";
    else if (r.robot_status == DELIVERED) // Explain the restart/reset controls after arrival.
        hint = "Delivery complete. Press R to reset or SPACE to start again.";
    text(image, hint, 18, 644, .48);
    std::string coral_line =
        !coral_required ? "Coral: OFF (explicit)"
        : coral.healthy ? "Coral USB: LIVE | inference " + fixed(coral.inference_ms) + " ms | " +
                              std::to_string(coral.detections.size()) + " detections | age " +
                              fixed((now - coral.frame_time) * 1000, 0) + " ms"
                        : "Coral: HOLD | " + coral.detail; // Compose inference availability, measured Invoke time, detections, and source-image age.
    if(evaluation && coral.healthy) coral_line="Coral YOLO | invoke "+fixed(coral.inference_ms)+" ms | total "+fixed(coral.total_ms)+" ms | source age "+fixed((now-coral.frame_time)*1000,0)+" ms";
    text(image, coral_line.substr(0, 115), 18, 680, .45, purple);
    text(image, "C++ step " + fixed(step_ms) + " ms", 1040, 680, .4, muted);
    text(image,
         "SPACE start/pause   R reset   N lane   M mask   S snapshot   [ / ] radius   B save "
         "boundary   Q/Esc quit",
         18, 702, .43);
    BoundaryUi fallback(c); // Provide display limits even when no interactive widget instance was supplied.
    const auto &ui = boundary ? *boundary : fallback; // Prefer the active widget state; headless callers can render using the configuration-derived fallback.
    const double radius = exclusion_radius(c); // Show the selected total exclusion radius used by the collision checker.
    text(image, "FIGURE BOUNDARY", 18, 741, .48, amber);
    text(image, "Exclusion radius for robot center", 18, 770, .34, muted); // Describe the selected total distance without implying an additional footprint inflation.
    auto button = [&](const cv::Rect &rect, const char *label, int inset) { // Draw a control's filled background, outline, and label consistently with its hit box.
        cv::rectangle(image, rect, cv::Scalar(43, 49, 54), cv::FILLED); // Fill the button face using the control background color.
        cv::rectangle(image, rect, muted, 1); // Outline the clickable button boundary in a contrasting muted color.
        text(image, label, rect.x + inset, rect.y + 25, .58, white);
    };
    button(boundary_minus, "-", 12); // Render the decrement control at its mouse hit box.
    button(boundary_plus, "+", 10); // Render the increment control at its mouse hit box.
    button(boundary_save, "Save", 18); // Render the explicit persistence control at its mouse hit box.
    text(image, fixed(radius * 100) + " cm", 282, 743, .62, amber, 2);
    const double fraction =
        std::clamp((radius * 1000 - ui.minimum_mm) / (ui.maximum_mm - ui.minimum_mm), 0., 1.); // Convert the current mouse/applied-radius location into a normalized, clamped slider fraction.
    const int thumb = slider_left + int(std::lround(fraction * (slider_right - slider_left))); // Convert normalized applied radius to the visible slider thumb's pixel x coordinate.
    cv::line(image, {slider_left, slider_y}, {slider_right, slider_y}, muted, 3, cv::LINE_AA); // Draw the slider's full available range.
    cv::line(image, {slider_left, slider_y}, {thumb, slider_y}, amber, 3, cv::LINE_AA); // Highlight the portion corresponding to the currently applied radius.
    cv::circle(image, {thumb, slider_y}, 8, amber, cv::FILLED, cv::LINE_AA); // Draw the slider handle above the track at the applied-radius position.
    text(image, fixed(ui.minimum_mm / 10.) + " cm", slider_left - 10, 770, .34, muted);
    text(image, fixed(ui.maximum_mm / 10.) + " cm", slider_right - 38, 770, .34, muted);
    text(image, ui.notice, 975, 741, .42, ui.notice == "Saved" ? green : muted);
    text(image, "Changes apply live", 865, 770, .34, muted);
    return image; // Return an owned canvas ready for display or PNG serialization without mutating core state.
}
SessionLog::SessionLog(const std::filesystem::path &root, const Config &config, bool demo,
                       bool coral_required) { // Create a unique UTC-named run directory, write its startup settings and mode metadata, and open the CSV with write failures enabled as exceptions.
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); // Use wall time only for human-readable session naming, never sensor freshness or motion integration.
    std::tm utc{}; // Allocate a local UTC calendar structure for thread-safe platform-specific conversion.
#ifdef _WIN32
    gmtime_s(&utc, &t); // Convert wall-clock time to UTC using Windows' destination-first secure API.
#else
    gmtime_r(&t, &utc); // Convert wall-clock time to UTC using POSIX's caller-owned output structure.
#endif
    std::ostringstream name; // Build a sortable timestamp and mode suffix for the session directory.
    name << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << (demo ? "_demo" : "_live"); // Include UTC Z suffix and distinguish simulated from live data at the directory level.
    std::filesystem::create_directories(root); // Ensure the caller-selected output root exists before creating a unique session child.
    path_ = root / name.str(); // Form the preferred session directory beneath the output root.
    for (int i = 1; std::filesystem::exists(path_); ++i) // Avoid overwriting earlier sessions when launches share the same UTC second.
        path_ = root / (name.str() + "_" + std::to_string(i)); // Add a numeric suffix until an unused session directory name is found.
    std::filesystem::create_directory(path_); // Create the selected unique session directory.
    save_config(config, path_ / "effective_config.json"); // Persist resolved operational settings so the recorded run can be interpreted later.
    cv::FileStorage session((path_ / "session.json").string(),
                            cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON); // Create a metadata file alongside effective settings and the time-series CSV.
    session << "implementation" << "native C++ executable" << "simulation" << int(demo) // Record implementation mode, whether Coral is required, and the host-receive timing limitation.
            << "coral_required" << int(coral_required) << "timing"
            << "Host receive-time pairing; not hardware synchronization";
    csv_.exceptions(std::ios::badbit | std::ios::failbit); // Make failed CSV writes/open operations visible as exceptions rather than silently losing diagnostics.
    csv_.open(path_ / "delivery.csv"); // Open the time-series log within this session's unique directory.
    csv_ << "elapsed_s,tracking,robot,scan,figure_x_m,figure_y_m,raw_x_m,raw_y_m,robot_x_m,robot_y_"
            "m,reroutes,pair_delta_ms,camera_received_s,lidar_received_s,coral_healthy,coral_"
            "sequence,coral_inference_ms,coral_stop_held,boundary_radius_m\n"; // Define CSV columns and units once before writing state samples.
    csv_ << std::fixed << std::setprecision(6); // Use six fractional digits for repeatable metric coordinates and timestamps.
}
void SessionLog::record(double elapsed, const Frame &f, const Scan &s, const Result &r,
                        const CoralState &c) { // Append a state sample. Invalid positions become empty CSV fields rather than misleading zeros; live boundary radius is recorded on every sample.
    csv_ << elapsed << ',' << track_name(r.track_status) << ',' << robot_name(r.robot_status) << ','
         << s.sequence << ','; // Start the record with elapsed seconds, named states, and scan identity.
    if (r.track_status == TRACKING) // Show position-dependent figure geometry/text only while the fused track is valid.
        csv_ << r.xy.x << ',' << r.xy.y; // Write filtered figure coordinates only when tracking is valid.
    else
        csv_ << ','; // Emit CSV delimiters, including empty coordinate fields when the corresponding position is invalid.
    csv_ << ','; // Emit CSV delimiters, including empty coordinate fields when the corresponding position is invalid.
    if (r.selected >= 0) // Report cluster support only when association selected a valid LiDAR group.
        csv_ << r.raw.x << ',' << r.raw.y; // Write measured figure coordinates only when a current cluster was selected.
    else
        csv_ << ','; // Emit CSV delimiters, including empty coordinate fields when the corresponding position is invalid.
    csv_ << ',' << r.robot.x << ',' << r.robot.y << ',' << r.reroutes << ',' << r.pair_delta * 1000
         << ',' << f.received << ',' << s.received << ',' << int(c.healthy) << ',' << c.sequence
         << ',' << c.inference_ms << ',' << r.coral_held << ',' << r.obstacle_radius << '\n'; // Append robot pose, reroutes, timing, inference health, semantic latch, and live boundary radius.
    csv_.flush(); // Make each sample visible on disk promptly for live diagnosis and interrupted-run inspection.
}
void SessionLog::snapshot(const cv::Mat &screen, const Frame &frame, const Scan &scan,
                          const Result &result, const CoralState &coral) { // Save the rendered screen, exact paired camera image, raw scan, and structured state in a numbered directory. These are diagnostic observations, not ground truth.
    auto dir = path_ / ("snapshot_" + std::to_string(++snapshot_)); // Allocate the next snapshot number within this session, preserving earlier snapshots.
    std::filesystem::create_directory(dir); // Create the directory that groups one canvas, camera frame, scan, and result snapshot.
    if (!screen.empty() && !cv::imwrite((dir / "preview.png").string(), screen)) // Write the current rendered preview when available and report image encoding/write failure.
        throw std::runtime_error("Cannot save preview");
    if (frame &&
        !cv::imwrite((dir / "camera.png").string(),
                     cv::Mat(480, 640, CV_8UC3, const_cast<unsigned char *>(frame.pixels->data())))) // Save the exact paired source camera pixels only when a valid frame exists.
        throw std::runtime_error("Cannot save camera");
    std::ofstream scan_file; // Own the raw LiDAR CSV stream for this snapshot.
    scan_file.exceptions(std::ios::failbit | std::ios::badbit); // Turn incomplete raw-scan writes into explicit snapshot failures.
    scan_file.open(dir / "lidar.csv"); // Create the scan CSV inside the matching numbered snapshot directory.
    scan_file << "angle_deg,range_m,quality\n"
              << std::setprecision(10); // Label raw sensor degrees, metric range, and quality; preserve useful numeric precision.
    for (const auto &p : scan.points) // Serialize every raw scan return, including those the tracker may later reject.
        scan_file << p.angle << ',' << p.range << ',' << p.quality << '\n'; // Write one raw measurement without rotating it into calibrated coordinates.
    state_file(dir / "state.json", frame, scan, result, coral); // Save structured result and source timing metadata alongside the matching image and scan artifacts.
}
} // namespace tabletop
