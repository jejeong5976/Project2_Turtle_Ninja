#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
namespace mapping_trial {
inline void require(bool good,const std::string &why){if(!good)throw std::runtime_error(why);}
inline double wrap(double v){return std::remainder(v,2*std::acos(-1.));}
struct Pose {int64_t stamp=0;double x=0,y=0,yaw=0;};
struct Delta {double forward,lateral,yaw,distance;};
enum class Motion {forward,rotation};
struct Pulse {const char *name;double forward,yaw;size_t messages;double deadline;};
inline Pulse pulse(Motion motion){
  return motion==Motion::rotation?Pulse{"rotation",0,.10,20,2.2}:Pulse{"forward",.025,0,40,4.3};
}
inline Motion motion_from_flag(const std::string &flag){
  require(flag=="--execute-forward-pulse"||flag=="--execute-rotation-pulse",
    "Choose exactly --execute-forward-pulse or --execute-rotation-pulse (user-run hardware action)");
  return flag=="--execute-rotation-pulse"?Motion::rotation:Motion::forward;
}
inline Delta relative(const Pose &p,const Pose &origin){
  const double x=p.x-origin.x,y=p.y-origin.y,c=std::cos(origin.yaw),s=std::sin(origin.yaw);
  return {c*x+s*y,-s*x+c*y,wrap(p.yaw-origin.yaw),std::hypot(x,y)};
}
inline void bounds(const Pose &p,const Pose &origin,Motion motion=Motion::forward){
  const auto d=relative(p,origin);
  if(motion==Motion::rotation){
    require(d.distance<.030&&d.yaw>-.020&&d.yaw<.250,
      "Odom rotation limit: 30 mm translation, -0.020 to +0.250 rad yaw");
    return;
  }
  require(d.distance<.15&&d.forward>-.01&&std::abs(d.lateral)<.03&&std::abs(d.yaw)<.10,
    "Odom motion limit: 150 mm total, -10 mm reverse, 30 mm lateral, 0.10 rad yaw");
}
inline void completed(const Pose &p,const Pose &origin,Motion motion){
  const auto d=relative(p,origin);
  if(motion==Motion::rotation)
    require(d.yaw>=.100,"Less than +0.100 rad rotation; do not extend or retry");
  else require(d.forward>=.030,"Less than 30 mm forward movement; do not extend or retry");
}
inline void advancing(const Pose &p,const Pose &last){
  const double dt=(p.stamp-last.stamp)/1e9;
  require(dt>0&&dt<=.040,"Odometry regressed or gap exceeded 40 ms");
  require(std::hypot(p.x-last.x,p.y-last.y)<=.04*dt&&std::abs(wrap(p.yaw-last.yaw))<=.15*dt,
    "Odom speed exceeds mapping envelope: 0.04 m/s or 0.15 rad/s");
}
inline bool stationary(const std::deque<Pose> &poses){
  if(poses.size()<25)return false;
  const auto &a=poses[poses.size()-25];
  if(poses.back().stamp-a.stamp<400000000)return false;
  for(size_t i=poses.size()-25;i<poses.size();++i)
    if(std::hypot(poses[i].x-a.x,poses[i].y-a.y)>=.0005||std::abs(wrap(poses[i].yaw-a.yaw))>=.005)return false;
  return true;
}
struct Status {int64_t published,rejected,reference;};
inline Status status(const std::string &text){
  std::istringstream in(text);std::string item;std::map<std::string,std::string> fields;
  while(in>>item){const auto i=item.find('=');if(i!=std::string::npos)require(fields.emplace(item.substr(0,i),item.substr(i+1)).second,"Duplicate status field");}
  require(fields.at("model")=="nominal"&&fields.at("assumed_delay_ns")=="0"&&fields.at("acquisition_time_valid")=="0"&&
    fields.at("bins")=="1600"&&fields.at("fault")=="0"&&fields.at("fresh")=="1","Mapping status fault/stale or changed hypothesis");
  auto number=[&](const char *key){size_t used=0;const auto value=std::stoll(fields.at(key),&used);
    require(used==fields.at(key).size()&&value>=0,"Invalid status counter");return value;};
  return {number("published"),number("rejected"),number("reference_ns")};
}
} // namespace mapping_trial
