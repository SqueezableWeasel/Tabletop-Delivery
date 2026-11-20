/** @file Exercise comparison rendering and structured evidence without claiming real TPU inference. */
#include "app.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
using namespace tabletop;
int camera_color_count(const cv::Mat &image, const cv::Scalar &color) { // Count solid overlay pixels only inside the camera panel, excluding status text and the map.
    cv::Mat matches; // Hold the exact-color mask for deterministic synthetic rendering checks.
    cv::inRange(image(cv::Rect(18,80,640,480)),color,color,matches); // The synthetic camera does not contain either overlay color.
    return cv::countNonZero(matches); // Antialiased label edges are not needed to identify rectangle borders.
}
int main(int argc,char **argv) {
    try {
        if(argc!=2) throw std::runtime_error("Expected output directory");
        const std::filesystem::path directory=argv[1];
        std::filesystem::create_directories(directory);
        const auto c=default_config();
        CoralState state;
        state.ready=state.healthy=state.yolo=true;state.sequence=1;state.frame_time=10.;state.received=10.04;
        state.source_frame=synthetic_frame(c,{0,.6},10.,1);
        state.detections={{0,.82f,.44f,.4f,.57f,.7f},{11,.7f,.1f,.4f,.2f,.6f}};
        state.inference_ms=30;state.preprocess_ms=3;state.postprocess_ms=2;state.total_ms=35;
        Inputs input;input.frames.push_back(state.source_frame);input.scan=synthetic_scan(c,{0,.6},10.01,1);
        Result marker{},yolo{};
        marker.track_status=yolo.track_status=TRACKING;marker.last_seen=yolo.last_seen=10.01;
        marker.selected=yolo.selected=-1;marker.xy={0,.6};yolo.xy={.01,.6};marker.raw=marker.xy;yolo.raw=yolo.xy;
        marker.robot=yolo.robot=c.start;marker.goal=yolo.goal=c.goal;
        marker.obstacle_radius=yolo.obstacle_radius=exclusion_radius(c);
        DetectorOptions options{DetectorKind::YoloV8,.35f,true};
        marker.marker_count=1;marker.markers[0]={320,285,305,270,30,30,900}; // A real comparison has a separate blue marker inside the detector's larger target box.
        const auto observations=person_observations(state,options); // Populate the same detector-derived candidates the live fusion engine returns.
        if(observations.size()!=1) throw std::runtime_error("Fixture needs one eligible target");
        yolo.marker_count=1;yolo.markers[0]=observations.front(); // Expose the duplicate-overlay regression hidden by an empty candidate array.
        auto display=evaluation_display(false,marker,yolo,state,options);
        EvaluationLog log(directory,"compare","fixture-model",options);
        log.record(1.,10.05,input,state,display);log.record(1.01,10.06,input,state,display); // Repeated UI refresh must not double-count inference.
        log.finish();
        cv::FileStorage summary((directory/"evaluation_summary.json").string(),cv::FileStorage::READ);
        if(int(summary["completed_inferences"])!=1 || double(summary["p95_worker_total_ms"])!=35.)
            throw std::runtime_error("Summary must count one inference and include preprocessing/postprocessing");
        auto image=render(c,input,state.source_frame,marker,state,true,false,false,10.05,1.,nullptr,&display);
        if(!cv::imwrite((directory/"comparison.png").string(),image)) throw std::runtime_error("Cannot save comparison fixture");
        if(!camera_color_count(image,{140,225,135}) || !camera_color_count(image,{225,155,225}))
            throw std::runtime_error("Comparison must show both the independent marker and detector observations");
        display.yolo_controls=true;
        image=render(c,input,state.source_frame,yolo,state,true,false,false,10.05,1.,nullptr,&display);
        if(!cv::imwrite((directory/"yolo.png").string(),image)) throw std::runtime_error("Cannot save YOLO fixture");
        if(camera_color_count(image,{140,225,135}) || !camera_color_count(image,{225,155,225}))
            throw std::runtime_error("YOLO control must draw each detection once, without a duplicate green tracker box");
        const auto coco_header=image(cv::Rect(18,44,640,30)).clone(); // Preserve the stock-profile heading for comparison with the custom profile.
        state.figure_model=true;state.detections[1].id=1;options.profile=YoloProfile::Figure; // The custom model has figure 0 and stop sign 1, not a separate person class.
        display=evaluation_display(true,marker,yolo,state,options);
        image=render(c,input,state.source_frame,yolo,state,true,false,false,10.05,1.,nullptr,&display);
        if(!cv::imwrite((directory/"figure_yolo.png").string(),image)) throw std::runtime_error("Cannot save figure fixture");
        if(camera_color_count(image,{140,225,135}) || !camera_color_count(image,{225,155,225}))
            throw std::runtime_error("Custom figure control must also show a single detector overlay");
        if(cv::norm(coco_header,image(cv::Rect(18,44,640,30)),cv::NORM_INF)==0)
            throw std::runtime_error("Figure and person profiles must have different camera headings");
        auto different_frame=state.source_frame;++different_frame.sequence; // A detector box is valid only on its original source image.
        image=render(c,input,different_frame,yolo,state,true,false,false,10.05,1.,nullptr,&display);
        if(camera_color_count(image,{140,225,135}) || camera_color_count(image,{225,155,225}))
            throw std::runtime_error("A different displayed frame must not inherit detector or duplicate tracker boxes");
        std::cout<<"Comparison UI and measurement log fixture passed (synthetic inference)\n";
        return 0;
    } catch(const std::exception &e) { std::cerr<<e.what()<<'\n';return 1; }
}
