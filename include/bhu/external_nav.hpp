#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <mavlink/ardupilotmega/mavlink.h>

namespace bhu::extnav {

// All transforms map coordinates from the suffix frame into the prefix frame.
// W = ORB map, C = optical camera, B = vehicle FRD, L = gravity-aligned local FRD.
// T_L_W must include a measured gravity alignment; ORB's initial camera frame
// must never be relabelled as a level navigation frame.
struct Alignment {
    Eigen::Isometry3d T_L_W = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R_B_C = Eigen::Matrix3d::Identity();
};

inline bool rotation_valid(const Eigen::Matrix3d& r) {
    return r.allFinite() && (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-5
        && std::abs(r.determinant()-1)<1e-5;
}

struct Sample {
    uint64_t capture_host_us=0; // CLOCK_MONOTONIC at image capture, not send time
    uint64_t reset_epoch=0;     // increment for reset, map switch or pose correction
    Eigen::Isometry3d T_W_C=Eigen::Isometry3d::Identity(); // inverse of TrackStereo's T_C_W
    Eigen::Vector3d camera_velocity_W=Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_C=Eigen::Vector3d::Zero();
    int quality=0; // caller-provided diagnostic, not automatically 100 for tracking OK
    bool tracking_ok=false, imu_initialized=false, frame_alignment_valid=false;
    bool velocity_valid=false;
    bool imu_healthy=true;
};

struct Pose {
    Eigen::Vector3d camera_position_L;
    Eigen::Quaterniond q_L_B;
    Eigen::Vector3d camera_velocity_B, angular_velocity_B;
};

inline Pose convert(const Sample& s,const Alignment& a) {
    if(!s.T_W_C.matrix().allFinite() || !a.T_L_W.matrix().allFinite()
       || !rotation_valid(s.T_W_C.rotation()) || !rotation_valid(a.T_L_W.rotation())
       || !rotation_valid(a.R_B_C) || !s.camera_velocity_W.allFinite()
       || !s.angular_velocity_C.allFinite())
        throw std::invalid_argument("Invalid external-navigation transform or motion");
    Eigen::Matrix3d R_L_B=a.T_L_W.rotation()*s.T_W_C.rotation()*a.R_B_C.transpose();
    // Position remains the camera origin. VISO_POS_X/Y/Z supply its lever arm
    // on the Cube. Applying that translation here too would correct it twice.
    return {a.T_L_W*s.T_W_C.translation(),Eigen::Quaterniond(R_L_B).normalized(),
            R_L_B.transpose()*a.T_L_W.rotation()*s.camera_velocity_W,
            a.R_B_C*s.angular_velocity_C};
}

// Unknown covariance is explicit NaN, never a zero uncertainty claim.
// Measured covariance may replace this only in the documented output frames.
inline mavlink_message_t encode(const Sample& s,const Pose& p,uint8_t reset_counter,
                                uint8_t system=1,uint8_t component=197) {
    if(!system || !component || s.quality<1 || s.quality>100 || !s.capture_host_us
       || !p.camera_position_L.allFinite() || !p.q_L_B.coeffs().allFinite()
       || std::abs(p.q_L_B.norm()-1)>1e-5 || !p.camera_velocity_B.allFinite()
       || !p.angular_velocity_B.allFinite())
        throw std::invalid_argument("Invalid odometry fields");
    const auto& q=p.q_L_B;
    float quaternion[4]={float(q.w()),float(q.x()),float(q.y()),float(q.z())};
    std::array<float,21> unknown{};
    unknown[0]=std::numeric_limits<float>::quiet_NaN();
    mavlink_message_t message{};
    mavlink_msg_odometry_pack(system,component,&message,s.capture_host_us,
        MAV_FRAME_LOCAL_FRD,MAV_FRAME_BODY_FRD,
        p.camera_position_L.x(),p.camera_position_L.y(),p.camera_position_L.z(),quaternion,
        p.camera_velocity_B.x(),p.camera_velocity_B.y(),p.camera_velocity_B.z(),
        p.angular_velocity_B.x(),p.angular_velocity_B.y(),p.angular_velocity_B.z(),
        unknown.data(),unknown.data(),reset_counter,MAV_ESTIMATOR_TYPE_VIO,int8_t(s.quality));
    return message;
}

// Stateful publication gate: no retransmission of stale/lost/uninitialized data.
// Recovery requires consecutive valid samples and marks a discontinuity. A live
// producer must report every tracking failure, even when it publishes no packet.
class Gate {
    uint64_t last_seen_=0,epoch_=0,max_age_us_;
    unsigned good_=0,required_;
    bool seen_=false,published_=false,pending_reset_=false;
    uint8_t resets_=0;
public:
    explicit Gate(unsigned required_good_samples=30,uint64_t max_age_us=150000)
        :max_age_us_(max_age_us),required_(required_good_samples) {
        if(!required_)throw std::invalid_argument("Invalid odometry gate limits");
    }
    std::optional<uint8_t> accept(const Sample& s,uint64_t host_now_us) {
        const bool changed=seen_ && s.reset_epoch!=epoch_;
        const bool gap=max_age_us_ && seen_ && s.capture_host_us>last_seen_ && s.capture_host_us-last_seen_>max_age_us_;
        const bool monotonic=!seen_ || s.capture_host_us>last_seen_;
        if(changed || gap) {good_=0;pending_reset_=published_;}
        if(monotonic) {last_seen_=s.capture_host_us;epoch_=s.reset_epoch;seen_=true;}
        bool valid=monotonic && s.capture_host_us && s.capture_host_us<=host_now_us
            && (!max_age_us_ || host_now_us-s.capture_host_us<=max_age_us_) && s.tracking_ok && s.imu_initialized && s.imu_healthy
            && s.frame_alignment_valid && s.velocity_valid && s.quality>=1 && s.quality<=100
            && s.T_W_C.matrix().allFinite() && rotation_valid(s.T_W_C.rotation())
            && s.camera_velocity_W.allFinite() && s.angular_velocity_C.allFinite();
        if(!valid) {good_=0;pending_reset_=published_;return std::nullopt;}
        if(good_<required_)++good_;
        if(good_<required_)return std::nullopt;
        if(pending_reset_) {++resets_;pending_reset_=false;}
        published_=true;return resets_;
    }
};

// Publication recovery is not a coordinate-frame reset. Monitor all usable
// estimator poses, including poses withheld for insufficient visual support.
// The motion bounds below are engineering guards, not calibrated covariance.
class DurationGate {
    Gate guards_{1};
    uint64_t duration_,begin_=0,brief_duration_=0,brief_limit_=0,target_duration_=0,last_published_capture_=0;
    bool full_recovery_required_=false;
    uint8_t guard_reset_=0,resets_=0;
    bool published_=false,pending_reset_=false;
    std::optional<Sample> previous_pose_;
    bool seen_epoch_=false, recovering_=false;
    uint64_t epoch_=0, recoveries_=0,last_observed_=0;
    unsigned pending_causes_=0,last_causes_=0;
    void observe(const Sample& s,uint64_t now) {
        if(!s.capture_host_us || s.capture_host_us>now || s.capture_host_us<=last_observed_)return;
        last_observed_=s.capture_host_us;
        auto discontinuity=[&](unsigned cause){begin_=0;if(published_){pending_reset_=true;pending_causes_|=cause;recovering_=true;}};
        if(seen_epoch_ && s.reset_epoch!=epoch_) {
            discontinuity(1);previous_pose_.reset();
        }
        seen_epoch_=true;epoch_=s.reset_epoch;
        const bool usable=s.capture_host_us && s.capture_host_us<=now &&
            s.imu_initialized && s.imu_healthy && s.frame_alignment_valid && s.velocity_valid &&
            s.T_W_C.matrix().allFinite() && rotation_valid(s.T_W_C.rotation()) &&
            s.camera_velocity_W.allFinite() && s.angular_velocity_C.allFinite();
        if(!usable)return;
        if(previous_pose_ && s.capture_host_us<=previous_pose_->capture_host_us)return;
        if(previous_pose_) {
            const auto& p=*previous_pose_;
            const double dt=(s.capture_host_us-p.capture_host_us)*1e-6;
            if(dt>1.) {if(duration_)discontinuity(8);} // beyond the bounded continuity-prediction horizon
            else {
                const Eigen::Vector3d predicted=p.T_W_C.translation()+p.camera_velocity_W*dt;
                if((s.T_W_C.translation()-predicted).norm()>.20+5.*dt*dt)discontinuity(2);
                Eigen::Matrix3d rotation=p.T_W_C.rotation();
                const double rate=p.angular_velocity_C.norm();
                if(rate>1e-9)rotation=rotation*Eigen::AngleAxisd(rate*dt,p.angular_velocity_C/rate).toRotationMatrix();
                if(Eigen::Quaterniond(rotation).angularDistance(Eigen::Quaterniond(s.T_W_C.rotation()))>
                   .2617993877991494+10.*dt*dt)discontinuity(4);
            }
        }
        previous_pose_=s;
    }
public:
    // Zero duration/age selects immediate publication and delegates age to ArduCopter.
    explicit DurationGate(uint64_t duration_us=500000,uint64_t brief_duration_us=0,uint64_t brief_limit_us=0,uint64_t max_age_us=150000)
        :guards_(1,max_age_us),duration_(duration_us),brief_duration_(brief_duration_us),brief_limit_(brief_limit_us),target_duration_(duration_us) {
        if((!duration_ && brief_duration_) || bool(brief_duration_)!=bool(brief_limit_) || brief_duration_>duration_ || brief_limit_>1000000)throw std::invalid_argument("Invalid publication recovery duration");
    }
    std::optional<uint8_t> accept(const Sample& sample,uint64_t now) {
        observe(sample,now);
        // Only the opt-in comparison policy can shorten a continuous brief recovery.
        // Lost initialization, unhealthy IMU and invalid frame/pose require full qualification.
        if(!sample.imu_initialized || !sample.imu_healthy || !sample.frame_alignment_valid ||
           !sample.T_W_C.matrix().allFinite() || !rotation_valid(sample.T_W_C.rotation()) ||
           !sample.camera_velocity_W.allFinite() || !sample.angular_velocity_C.allFinite() ||
           sample.quality<1 || sample.quality>100 || sample.capture_host_us>now)full_recovery_required_=true;
        auto reset=guards_.accept(sample,now);
        if(!reset){begin_=0;recovering_=published_;return std::nullopt;}
        if(!begin_ || *reset!=guard_reset_){
            begin_=sample.capture_host_us;guard_reset_=*reset;
            recovering_=published_;
            const bool brief=brief_duration_ && published_ && !pending_reset_ && !full_recovery_required_ &&
                sample.capture_host_us>=last_published_capture_ && sample.capture_host_us-last_published_capture_<=brief_limit_;
            target_duration_=brief?brief_duration_:duration_;
        }
        if(sample.capture_host_us-begin_<target_duration_)return std::nullopt;
        if(recovering_){++recoveries_;recovering_=false;}
        if(pending_reset_){++resets_;pending_reset_=false;last_causes_=pending_causes_;pending_causes_=0;}
        published_=true;last_published_capture_=sample.capture_host_us;full_recovery_required_=false;return resets_;
    }
    uint64_t qualification_duration_us()const{return target_duration_;}
    uint8_t reset_counter()const{return resets_;}
    uint64_t recoveries()const{return recoveries_;}
    unsigned last_reset_causes()const{return last_causes_;} // epoch=1, position=2, attitude=4, unseen interval=8
    unsigned pending_reset_causes()const{return pending_causes_;}
};

} // namespace bhu::extnav
