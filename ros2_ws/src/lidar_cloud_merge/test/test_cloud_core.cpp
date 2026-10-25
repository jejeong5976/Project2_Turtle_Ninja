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
 check(cloud({},std_msgs::msg::Header()).data.empty());std::cout<<"PASS projection, filtering, fields, empty cloud, freshness and skew\n";
}
