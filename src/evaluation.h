/**
 * @file
 * Evaluation-only pairing, comparison, and logging. Histories are bounded and
 * detections always retain their original source image. Missing detections are
 * recorded as missing, never replaced by marker measurements in YOLO mode.
 */
#pragma once
#include "yolo.h"
#include <deque> // Keep a small recent scan history to compensate inference latency.
#include <fstream> // Write evaluation data without a Python runtime dependency.
#include <filesystem>
#include <iomanip>
#include <numeric>

namespace tabletop {
class DetectionPairer {
    std::deque<Scan> scans_; // At most 32 completed revolutions, copied once per new scan.
  public:
    void observe(const Scan &scan) {
        if (scan.sequence < 0 || !std::isfinite(scan.received)) return;
        if (!scans_.empty() && scan.sequence <= scans_.back().sequence) return;
        scans_.push_back(scan);
        while (scans_.size() > 32) scans_.pop_front();
    }
    Inputs pair(const CoralState &state) const {
        Inputs result; // An empty history keeps the normal stale-sensor holds active.
        if (!state.source_frame) return result;
        result.frames.push_back(state.source_frame);
        if (!scans_.empty()) {
            auto best = std::min_element(scans_.begin(),scans_.end(),[&](const Scan &a,const Scan &b) {
                return std::abs(a.received-state.frame_time)<std::abs(b.received-state.frame_time);
            });
            result.scan=*best; // Association later enforces both age and maximum timestamp separation.
        }
        return result;
    }
};
inline int step_detector(void *core, const Inputs &inputs, const CoralState &state, const DetectorOptions &options,
                         double now, const Control &control, Result &result) {
    auto observations=person_observations(state,options);
    return core_step_observations(core,observations.data(),int(observations.size()),
        state.source_frame.sequence,state.source_frame ? state.frame_time : now-100,
        inputs.scan.points.data(),int(inputs.scan.points.size()),
        inputs.scan.sequence>=0 ? inputs.scan.received : now-100,inputs.scan.sequence,now,&control,&result);
}
inline const char *detector_track_name(int state, bool figure = false) {
    switch(state) {
        case NO_MARKER: return figure ? "NO ELIGIBLE FIGURE" : "NO ELIGIBLE PERSON";
        case MULTIPLE_MARKERS: return figure ? "MULTIPLE FIGURES" : "MULTIPLE PEOPLE";
        case MARKER_ONLY: return figure ? "FIGURE / NO RANGE" : "PERSON / NO RANGE";
        default: return track_name(state);
    }
}
struct EvaluationDisplay {
    bool yolo_controls = false; // False means marker control with a YOLO shadow tracker.
    Result marker{}, yolo{}; // Compare independent trackers without changing either state.
    int people = 0;
    int eligible = 0; // Distinguish raw person detections from plausible tabletop-size targets.
    float confidence = 0; // Highest retained person confidence, zero when none passed the threshold.
    bool figure_model = false; // Select accurate user-facing names without changing the fusion state machine.
};
inline EvaluationDisplay evaluation_display(bool controls, const Result &marker, const Result &yolo,
                                             const CoralState &state, const DetectorOptions &options) {
    EvaluationDisplay result{controls,marker,yolo};
    result.figure_model = options.profile == YoloProfile::Figure;
    for(const auto &d:state.detections) if(d.id==0 && d.score>=options.person_confidence) {
        ++result.people; result.confidence=std::max(result.confidence,d.score);
        result.eligible+=eligible_person(d,options);
    }
    return result;
}
class EvaluationLog {
    std::ofstream csv_;
    std::filesystem::path path_;
    std::string mode_;
    int last_sequence_=-1, samples_=0, detections_=0, eligible_=0, fused_=0;
    std::vector<double> invoke_, total_, age_;
    static double mean(const std::vector<double> &values) {
        return values.empty()?0:std::accumulate(values.begin(),values.end(),0.)/values.size();
    }
    static double p95(std::vector<double> values) {
        if(values.empty()) return 0;
        std::sort(values.begin(),values.end());
        return values[std::size_t(std::ceil(values.size()*.95))-1];
    }
  public:
    EvaluationLog(const std::filesystem::path &directory, const std::string &mode,
                  const std::string &model, const DetectorOptions &options):path_(directory),mode_(mode) {
        csv_.open(path_/"evaluation.csv");
        if(!csv_) throw std::runtime_error("Cannot open evaluation.csv");
        csv_ << "elapsed_s,inference_sequence,camera_sequence,scan_sequence,people,eligible_people,best_person_confidence,"
                "yolo_status,marker_status,yolo_x_m,yolo_y_m,marker_x_m,marker_y_m,agreement_m,"
                "pair_delta_ms,preprocess_ms,invoke_ms,postprocess_ms,worker_total_ms,source_age_ms\n";
        std::ofstream settings(path_/"evaluation_settings.txt");
        if(!settings) throw std::runtime_error("Cannot record evaluation settings");
        settings<<"mode="<<mode<<"\nmodel="<<model<<"\nperson_confidence="<<options.person_confidence
                <<"\nyolo_profile="<<(options.profile == YoloProfile::Figure ? "figure" : "coco")<<"\nclass_count="<<yolo_classes(options)<<"\nstop_sign_class="<<stop_class(options)
                <<"\nmax_person_width="<<options.max_person_width<<"\nmax_person_height="<<options.max_person_height
                <<"\nbox_coordinates="<<(options.normalized_boxes?"normalized":"pixels")
                <<"\nassociation=bounding-box horizontal center using saved local bearing calibration\n"
                  "metrics=presence and association availability, not detection accuracy\n";
    }
    void record(double elapsed, double now, const Inputs &pair, const CoralState &state, const EvaluationDisplay &display) {
        if(state.sequence<0 || state.sequence==last_sequence_) return; // Count each completed inference exactly once.
        last_sequence_=state.sequence; ++samples_;
        detections_ += display.people>0; eligible_ += display.eligible>0; fused_ += display.yolo.track_status==TRACKING;
        invoke_.push_back(state.inference_ms); total_.push_back(state.total_ms);
        age_.push_back(std::max(0.,now-state.frame_time)*1000);
        csv_<<std::setprecision(9)<<elapsed<<','<<state.sequence<<','<<state.source_frame.sequence<<','<<pair.scan.sequence
            <<','<<display.people<<','<<display.eligible<<','<<display.confidence<<','<<detector_track_name(display.yolo.track_status, display.figure_model)
            <<','<<track_name(display.marker.track_status)<<',';
        if(display.yolo.track_status==TRACKING) csv_<<display.yolo.xy.x<<','<<display.yolo.xy.y;
        else csv_<<','; // Empty position fields remain distinguishable from real zero coordinates.
        csv_<<',';
        if(display.marker.track_status==TRACKING) csv_<<display.marker.xy.x<<','<<display.marker.xy.y;
        else csv_<<',';
        csv_<<',';
        if(display.marker.track_status==TRACKING && display.yolo.track_status==TRACKING)
            csv_<<std::hypot(display.marker.xy.x-display.yolo.xy.x,display.marker.xy.y-display.yolo.xy.y);
        csv_<<','<<(state.frame_time-pair.scan.received)*1000<<','<<state.preprocess_ms<<','<<state.inference_ms
            <<','<<state.postprocess_ms<<','<<state.total_ms<<','<<age_.back()<<'\n';
        csv_.flush(); // Keep evidence available if a live evaluation is interrupted.
        if(!csv_) throw std::runtime_error("Failed to write evaluation.csv");
    }
    void finish() {
        std::ofstream out(path_/"evaluation_summary.json");
        if(!out) throw std::runtime_error("Cannot write evaluation summary");
        out<<"{\n  \"mode\": \""<<mode_<<"\",\n  \"completed_inferences\": "<<samples_
           <<",\n  \"person_presence_fraction\": "<<(samples_?double(detections_)/samples_:0)
           <<",\n  \"eligible_person_fraction\": "<<(samples_?double(eligible_)/samples_:0)
           <<",\n  \"fused_tracking_fraction\": "<<(samples_?double(fused_)/samples_:0)
           <<",\n  \"mean_invoke_ms\": "<<mean(invoke_)<<",\n  \"p95_invoke_ms\": "<<p95(invoke_)
           <<",\n  \"mean_worker_total_ms\": "<<mean(total_)<<",\n  \"p95_worker_total_ms\": "<<p95(total_)
           <<",\n  \"p95_source_age_ms\": "<<p95(age_)
           <<",\n  \"accuracy_measured\": false\n}\n"; // No ground-truth labels means these are availability metrics, not accuracy.
        if(!out) throw std::runtime_error("Failed to write evaluation summary");
    }
};
} // namespace tabletop
