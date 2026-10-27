#include "lidar_cloud_merge/core.hpp"
#include <limits>
#include <iostream>
void check(bool ok){if(!ok)throw std::runtime_error("Test failed");}
int main(){using namespace lidar_cloud_merge;
 sensor_msgs::msg::LaserScan s;s.angle_increment=float(std::acos(-1.)/2);s.range_min=.05;s.range_max=12;
 s.ranges={1,2,std::numeric_limits<float>::quiet_NaN(),13,0};
 geometry_msgs::msg::Transform tf;tf.rotation.z=1;tf.rotation.w=0;tf.translation.x=.22;
 std::vector<Point> pts;append(s,tf,1,pts);check(pts.size()==2);
 check(std::abs(pts[0][0]+.78)<1e-6&&std::abs(pts[1][0]-.22)<1e-6&&std::abs(pts[1][1]+2)<1e-6);
 check(pts[0][3]==0&&pts[0][4]==1);
 auto c=cloud(pts,std_msgs::msg::Header());check(c.width==2&&c.point_step==20&&c.data.size()==40);
 check(eligible(10,9.95,9.98,.3,.075));check(!eligible(10,9,9.98,.3,.075));
 check(!eligible(10,10.1,9.98,.3,.075));check(!eligible(10,9.8,9.98,.3,.075));
 BodyMask mask;mask.validate();
 // Both sensor origins, body corners and expanded boundary are excluded.
 check(mask.contains({0,0,0,0,0})&&mask.contains({.22f,0,0,0,1}));
 for(float x:{-.028f,.248f})for(float y:{-.133f,.143f})check(mask.contains({x,y,10,0,0}));
 for(Point p:std::vector<Point>{{-.029f,0,0,0,0},{.249f,0,0,0,1},{0,-.134f,0,0,0},{0,.144f,0,0,1}})check(!mask.contains(p));
 auto outside=std::vector<Point>{{-.029f,0,0,12,0},{.249f,0,0,34,1}};
 auto filtered=outside;filtered.push_back({0,0,0,7,0});filtered.push_back({.22f,0,0,8,1});
 check(exclude_body(filtered,mask)==2&&filtered==outside);
 BodyMask invalid=mask;invalid.margin=-.01;bool rejected=false;
 try{invalid.validate();}catch(const std::runtime_error&){rejected=true;}check(rejected);
 // Plate -> front: (xf,yf)=(.110-xp,.005-yp); mask must agree in both frames.
 for(double xp:{-.139,-.128,0.,.128,.139})for(double yp:{-.139,-.128,0.,.128,.139})
   check(mask.contains({float(.110-xp),float(.005-yp),0,0,0})==(std::abs(xp)<=.138&&std::abs(yp)<=.138));
 // Filtering occurs after rear transformation, not around each sensor separately.
 sensor_msgs::msg::LaserScan near=s;near.angle_increment=0;near.ranges={.1f,.3f};
 std::vector<Point> rear_points;append(near,tf,1,rear_points);
 check(exclude_body(rear_points,mask)==1&&rear_points.size()==1&&std::abs(rear_points[0][0]+.08)<1e-6);
 // Revised installed-robot envelope; leave the desk mask above unchanged.
 BodyMask swept{.110,.005,.2552,.3050,.010};swept.validate();
 for(float x:{-.0276f,.2476f})for(float y:{-.1575f,.1675f})check(swept.contains({x,y,0,0,0}));
 for(Point p:std::vector<Point>{{-.0277f,0,0,0,0},{.2477f,0,0,0,0},
                              {0,-.1576f,0,0,0},{0,.1676f,0,0,0}})check(!swept.contains(p));
 // Outward-steered wheel region now excluded; narrower X must remain exact.
 check(swept.contains({.11f,.16f,0,0,0})&&!mask.contains({.11f,.16f,0,0,0}));
 check(!swept.contains({-.0278f,0,0,0,0})&&mask.contains({-.0278f,0,0,0,0}));
 // +5 mm sensor offset makes the two scan-frame Y intervals asymmetric.
 for(double xr:{-.0277,.0,.11,.2477})for(double yr:{-.1676,-.16,.0,.16,.1576})
   check(swept.contains({float(.22-xr),float(-yr),0,0,1})==
         (xr>=-.0276&&xr<=.2476&&yr>=-.1675&&yr<=.1575));
 for(double xp:{-.138,-.1375,0.,.1375,.138})for(double yp:{-.163,-.1624,0.,.1624,.163})
   check(swept.contains({float(.110-xp),float(.005-yp),0,0,0})==
         (std::abs(xp)<=.1376&&std::abs(yp)<=.1625));
 check(cloud({},std_msgs::msg::Header()).data.empty());std::cout<<"PASS projection, filtering, fields, empty cloud, freshness, skew, desk and revised steering envelopes\n";
}
