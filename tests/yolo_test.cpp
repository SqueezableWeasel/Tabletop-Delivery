/** @file Device-free checks for quantization, letterboxing, NMS, and temporal fusion. */
#include "evaluation.h"
#include <iostream>
#include <stdexcept>

using namespace tabletop;
void require(bool value,const char *message) { if(!value) throw std::runtime_error(message); }
template<class F> void rejects(F action,const char *message) { bool failed=false; try { action(); } catch(...) { failed=true; } require(failed,message); }
int main() {
    try {
        const auto box=letterbox(320,320);
        require(box.resized_width==320 && box.resized_height==240 && box.top==40,"letterbox geometry");
        require(quantized_rgb(0,1.f/255,-128,true)==128 && quantized_rgb(255,1.f/255,-128,true)==127,"signed input endpoints");
        require(quantized_rgb(255,1.f/255,0,false)==255,"unsigned input endpoint");
        rejects([]{quantized_rgb(10,0,0,false);},"invalid quantization rejected");
        std::vector<float> tensor(84*3,0);
        auto set=[&](int a,int c,float v) { tensor[c*3+a]=v; };
        for(int a=0;a<3;++a) { set(a,0,.5f);set(a,1,.5f);set(a,2,.2f);set(a,3,.3f); }
        set(0,4,.8f);set(1,4,.7f);set(2,15,.9f); // Two duplicate people plus an overlapping stop sign.
        YoloTensor view{tensor.data(),tensor.size()*sizeof(float),84,3,TensorScalar::Float32,1,0};
        DetectorOptions options{DetectorKind::YoloV8,.35f,true};
        auto result=decode_yolov8(view,box,options);
        require(result.size()==2 && result[0].id==11 && result[1].id==0,"class-aware NMS and stop-sign class");
        require(std::abs(result[1].x0-.4)<1e-5 && std::abs(result[1].y0-.3)<1e-5 && std::abs(result[1].y1-.7)<1e-5,"inverse letterbox coordinates");
        auto bad=view;bad.bytes-=1;rejects([&]{decode_yolov8(bad,box,options);},"tensor bounds");
        tensor[4*3]=std::numeric_limits<float>::quiet_NaN();
        rejects([&]{decode_yolov8(view,box,options);},"nonfinite probabilities");tensor[4*3]=.8f;
        std::vector<std::int8_t> signed_tensor(84*3);
        for(std::size_t i=0;i<tensor.size();++i) signed_tensor[i]=std::int8_t(std::round(tensor[i]*100)-128);
        YoloTensor quant{signed_tensor.data(),signed_tensor.size(),84,3,TensorScalar::Int8,.01f,-128};
        auto quant_result=decode_yolov8(quant,box,options);
        require(quant_result.size()==2 && std::abs(quant_result[1].score-.8)<.001,"int8 output dequantization");
        std::vector<float> transposed(84*3);
        for(int a=0;a<3;++a) for(int c=0;c<84;++c) transposed[a*84+c]=tensor[c*3+a];
        view={transposed.data(),transposed.size()*sizeof(float),3,84,TensorScalar::Float32,1,0};
        require(decode_yolov8(view,box,options).size()==2,"transposed output layout");
        for(int a=0;a<3;++a) for(int c=0;c<4;++c) transposed[a*84+c]*=320;
        options.normalized_boxes=false;
        auto pixels=decode_yolov8(view,box,options);
        require(std::abs(pixels[1].x0-.4)<1e-5,"explicit pixel output convention");

        // The custom profile must decode six channels and keep figure/sign semantics separate.
        DetectorOptions custom; custom.kind=DetectorKind::YoloV8; custom.profile=YoloProfile::Figure; custom.person_confidence=.25f;
        std::vector<std::uint8_t> two_class(6*3,0);
        for(int a=0;a<3;++a) { two_class[a]=50;two_class[3+a]=50;two_class[6+a]=20;two_class[9+a]=30; }
        two_class[12]=80;two_class[13]=70;two_class[17]=90; // Two overlapping figure boxes and an overlapping sign.
        YoloTensor custom_tensor{two_class.data(),two_class.size(),6,3,TensorScalar::UInt8,.01f,0};
        auto custom_result=decode_yolov8(custom_tensor,box,custom);
        require(custom_result.size()==2 && custom_result[0].id==1 && custom_result[1].id==0,"custom class-aware NMS and mapping");
        require(detected_stop(custom_result[0],custom) && !detected_stop(custom_result[1],custom),"only custom class 1 activates stop gate");
        require(!detected_stop({11,.99f,0,0,1,1},custom) && !detected_stop({1,.59f,0,0,1,1},custom),"custom gate rejects COCO stop id and low confidence");
        require(detected_stop({11,.9f,0,0,1,1},options),"stock YOLO stop gate retained");
        require(detected_stop({12,.9f,0,0,1,1},DetectorOptions{}),"SSD stop gate retained");
        require(std::abs(custom_result[1].y0-.3f)<1e-5,"custom inverse letterbox");
        rejects([&]{decode_yolov8(custom_tensor,box,options);},"custom tensor rejected by stock profile");
        rejects([&]{decode_yolov8(view,box,custom);},"stock tensor rejected by custom profile");
        std::vector<std::int8_t> custom_transposed(18);
        for(int a=0;a<3;++a) for(int c=0;c<6;++c) custom_transposed[a*6+c]=std::int8_t(int(two_class[c*3+a])-128);
        custom_tensor={custom_transposed.data(),custom_transposed.size(),3,6,TensorScalar::Int8,.01f,-128};
        require(decode_yolov8(custom_tensor,box,custom).size()==2,"custom transposed int8 output");
        CoralState custom_state;custom_state.detections=custom_result;
        require(person_observations(custom_state,custom).size()==1,"custom sign cannot become tracked figure");
        require(std::string(detector_track_name(NO_MARKER,true))=="NO ELIGIBLE FIGURE","custom display uses figure labels");

        Config config=default_config();
        auto *core=core_create(&config);require(core!=nullptr,"core initialization");
        Marker marker{config.anchors[2],300,int(config.anchors[2])-20,250,40,100,4000};
        Result tracked{};Control ctl{TOGGLE_RUN,0,0,-1,0};
        auto step=[&](double time,int scan_seq,std::uint64_t image_seq,const Marker *obs,int n,double frame_time=-1) {
            const auto scan=synthetic_scan(config,{0,.6},time,scan_seq);
            require(core_step_observations(core,obs,n,image_seq,frame_time<0?time:frame_time,scan.points.data(),int(scan.points.size()),time,scan_seq,time,&ctl,&tracked)==0,"external core step");
            ctl.command=NONE;
        };
        step(1.,1,1,&marker,1);require(tracked.track_status==ACQUIRING,"first camera/scan pair only acquires");
        step(1.1,2,1,&marker,1);require(tracked.track_status==ACQUIRING,"repeated camera cannot confirm");
        step(1.2,2,2,&marker,1);require(tracked.track_status==TRACKING,"two distinct pairs confirm");
        const auto seen=tracked.last_seen;
        step(1.3,2,3,&marker,1);require(tracked.last_seen==seen,"repeated scan cannot update position");
        step(1.4,3,3,nullptr,0);require(tracked.track_status==NO_MARKER && tracked.robot_status==TRACK_LOST,"missing person holds robot");
        Marker multiple[]{marker,marker};step(1.5,4,4,multiple,2);
        require(tracked.track_status==MULTIPLE_MARKERS && tracked.robot_status==TRACK_LOST,"multiple people hold robot");
        step(2.,5,5,&marker,1,1.);require(tracked.track_status==STALE_CAMERA,"old source frame rejected");
        step(2.1,6,6,&marker,1,2.2);require(tracked.track_status==STALE_CAMERA,"future source frame rejected");
        step(3.,7,7,&marker,1,2.7);require(tracked.track_status==TIMING_GAP,"bad temporal pair rejected");
        core_destroy(core);

        DetectionPairer pairer;
        pairer.observe(synthetic_scan(config,{0,.6},10.,10));
        pairer.observe(synthetic_scan(config,{0,.6},10.15,11));
        CoralState state;state.frame_time=10.03;state.source_frame=synthetic_frame(config,{0,.6},10.03,1);
        const auto pair=pairer.pair(state);
        require(pair.scan.sequence==10 && pair.frames.front().sequence==1,"pair detector source to historical nearest scan");
        require(pairer.pair(CoralState{}).frames.empty(),"no source is not a fabricated observation");
        state.detections={{0,.8f,.4f,.3f,.6f,.7f},{11,.9f,.4f,.3f,.6f,.7f}};
        const auto observations=person_observations(state,options);
        require(observations.size()==1 && std::abs(observations[0].u-320)<1e-4,"only person boxes become figure observations");
        state.detections={{0,.99f,0.f,0.f,.9f,.8f}};
        require(person_observations(state,options).empty(),"large arm/person box excluded from tabletop tracking");
        std::cout<<"YOLO decoding, pairing, and fail-closed tracking checks passed\n";
        return 0;
    } catch(const std::exception &error) { std::cerr<<error.what()<<'\n';return 1; }
}
