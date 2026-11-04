#include "body_route_controller/layout.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <map>
#include <stdexcept>

namespace body_route_controller::layout
{
namespace
{
using V = Eigen::Vector2d;
using State = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
V v(Point p) {return {p.x,p.y};}
Point point(V p) {return {p.x(),p.y()};}
V rotate(double a, V p) {return {std::cos(a)*p.x()-std::sin(a)*p.y(),std::sin(a)*p.x()+std::cos(a)*p.y()};}
bool finite(Point p) {return std::isfinite(p.x)&&std::isfinite(p.y);}
bool finite(Pose p) {return finite(Point{p.x,p.y})&&std::isfinite(p.yaw);}
double distance(Point p, Point a, Point b) {
  const V d=v(b)-v(a); return (v(p)-v(a)-std::clamp((v(p)-v(a)).dot(d)/d.squaredNorm(),0.,1.)*d).norm();
}
Pose pose(const State & x, int i) {return {x[3*i],x[3*i+1],x[3*i+2]};}
Pose relative(Pose a, Pose b) {
  const V d=rotate(-a.yaw,V{b.x-a.x,b.y-a.y}); return {d.x(),d.y(),wrap(b.yaw-a.yaw)};
}
Pose compose(Pose a, Pose b) {const auto p=transform(a,{b.x,b.y});return {p.x,p.y,wrap(a.yaw+b.yaw)};}
Wall moved(const Wall & w, const State & x, int offset,Point pivot) {
  const V c=v(pivot), d{x[offset],x[offset+1]};
  return {w.id,point(c+d+rotate(x[offset+2],v(w.a)-c)),point(c+d+rotate(x[offset+2],v(w.b)-c)),w.assembly};
}
struct Match {int frame, wall; Segment segment; int edge_a=-1,edge_b=-1;};
struct Linear {
  Matrix a; State b; int rows=0;
  Linear(int maximum,int n):a(Matrix::Zero(maximum,n)),b(State::Zero(maximum)) {}
  void add(const State & x,const std::vector<int>& ids,const std::function<double(const State&)>& f,
    double sigma,bool robust=true) {
    const double r=f(x), weight=robust?std::sqrt(std::min(1.,2.5*sigma/std::max(std::abs(r),1e-12))):1.;
    b[rows]=-r*weight/sigma;
    for(int id:ids) {State shifted=x;shifted[id]+=1e-6;a(rows,id)=(f(shifted)-r)*weight/(sigma*1e-6);}
    ++rows;
  }
};
bool independent(Pose a,Pose b,const Settings & s) {
  return std::hypot(a.x-b.x,a.y-b.y)>=s.min_view_baseline||
    std::abs(wrap(a.yaw-b.yaw))>=s.min_view_angle;
}
void validateScan(const Scan & scan) {
  if(scan.sensor_id.empty()||!finite(scan.sensor)||!std::isfinite(scan.stamp)||!std::isfinite(scan.angle_min)||
    !std::isfinite(scan.angle_increment)||scan.angle_increment<=0||scan.angle_increment>.1||
    !std::isfinite(scan.range_min)||!std::isfinite(scan.range_max)||scan.range_min<0||
    scan.range_max<=scan.range_min||scan.range_max>50||scan.ranges.size()<10||scan.ranges.size()>20000||
    scan.angle_increment*(scan.ranges.size()-1)>2*3.14159265358979323846+.02) {
    throw std::invalid_argument("invalid laser scan geometry");
  }
  const auto valid=std::count_if(scan.ranges.begin(),scan.ranges.end(),[&](double r){return std::isfinite(r)&&r>=scan.range_min&&r<scan.range_max;});
  if(static_cast<std::size_t>(valid)<std::max<std::size_t>(10,scan.ranges.size()/50)) {throw std::invalid_argument("insufficient valid lidar returns");}
}
bool validRange(const Scan & s,double r) {return std::isfinite(r)&&r>=s.range_min&&r<s.range_max;}
}

Point transform(Pose p,Point q) {return point(V{p.x,p.y}+rotate(p.yaw,v(q)));}
void Settings::validate() const {
  for(double x:{max_wall_shift,max_wall_yaw,split_error,cluster_jump,min_segment,
    association_distance,association_angle,association_margin,line_sigma,edge_sigma,odom_sigma,
    odom_yaw_sigma,start_sigma,start_yaw_sigma,max_position_sigma,max_yaw_sigma,max_residual,
    min_view_baseline,min_view_angle,max_scan_skew}) {
    if(!std::isfinite(x)||x<=0) {throw std::invalid_argument("layout settings must be finite and positive");}
  }
  if(max_wall_shift>.05||max_wall_yaw>.2||min_points<4||max_frames<2||max_frames>64||max_walls<1||max_walls>100) {
    throw std::invalid_argument("layout settings outside supported envelope");
  }
}

std::vector<Segment> extract(const Scan & scan,const Settings & cfg) {
  cfg.validate(); validateScan(scan); std::vector<Segment> out;
  std::vector<V> points(scan.ranges.size()); std::vector<bool> valid(scan.ranges.size());
  for(std::size_t i=0;i<points.size();++i) {
    valid[i]=validRange(scan,scan.ranges[i]);
    const double a=scan.angle_min+i*scan.angle_increment;
    if(valid[i]) {points[i]=v(transform(scan.sensor,{scan.ranges[i]*std::cos(a),scan.ranges[i]*std::sin(a)}));}
  }
  std::function<void(int,int)> split=[&](int begin,int end) {
    if(end-begin<cfg.min_points) {return;}
    V c=V::Zero();for(int i=begin;i<end;++i) {c+=points[i];} c/=end-begin;
    Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();
    for(int i=begin;i<end;++i) {const V d=points[i]-c;cov+=d*d.transpose();}
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov); const V n=eig.eigenvectors().col(0);
    double worst=0;int at=begin;
    for(int i=begin;i<end;++i) {const double e=std::abs(n.dot(points[i]-c));if(e>worst) {worst=e;at=i;}}
    if(worst>cfg.split_error) {
      // Drop isolated endpoint outliers; interior corners are split into two fits.
      if(at-begin<cfg.min_points) {split(at+1,end);} else if(end-at<cfg.min_points) {split(begin,at);}
      else {split(begin,at+1);split(at+1,end);} return;
    }
    if((points[end-1]-points[begin]).norm()<cfg.min_segment) {return;}
    auto edge=[&](int endpoint,int neighbour) {
      return neighbour>=0&&neighbour<static_cast<int>(points.size())&&valid[neighbour]&&
        scan.ranges[neighbour]-scan.ranges[endpoint]>.10;
    };
    out.push_back({point(points[begin]-n*n.dot(points[begin]-c)),
      point(points[end-1]-n*n.dot(points[end-1]-c)),edge(begin,begin-1),edge(end-1,end),0});
  };
  int begin=-1;
  for(int i=0;i<static_cast<int>(points.size());++i) {
    const bool cut=!valid[i]||(begin>=0&&(points[i]-points[i-1]).norm()>cfg.cluster_jump);
    if(cut&&begin>=0) {split(begin,i);begin=-1;}
    if(valid[i]&&begin<0) {begin=i;}
  }
  if(begin>=0) {split(begin,points.size());} return out;
}

Estimate estimate(const std::vector<Wall> & walls,const std::vector<Frame> & frames,Pose start,const Settings & cfg) {
  cfg.validate();
  if(!finite(start)||frames.size()<2||frames.size()>static_cast<std::size_t>(cfg.max_frames)||
    walls.empty()||walls.size()>static_cast<std::size_t>(cfg.max_walls)) {throw std::invalid_argument("invalid layout dimensions/start");}
  std::set<std::string> names;
  for(const auto & w:walls) {
    if(w.id.empty()||!names.insert(w.id).second||!finite(w.a)||!finite(w.b)||(v(w.b)-v(w.a)).norm()<cfg.min_segment) {
      throw std::invalid_argument("invalid or duplicate wall");
    }
  }
  for(std::size_t i=0;i<frames.size();++i) {
    const auto & f=frames[i];
    if(!std::isfinite(f.stamp)||!finite(f.odom)||f.scans.size()!=2||(i&&f.stamp<=frames[i-1].stamp)||
      !std::isfinite(f.settled_seconds)||f.settled_seconds<1||
      !std::isfinite(f.measured.x)||!std::isfinite(f.measured.y)||!std::isfinite(f.measured.yaw)||
      std::hypot(f.measured.x,f.measured.y)>.003||std::abs(f.measured.yaw)>.01) {
      throw std::invalid_argument("requires increasing stationary dual-lidar keyframes");
    }
    if(f.scans[0].sensor_id==f.scans[1].sensor_id||std::abs(f.scans[0].stamp-f.scans[1].stamp)>cfg.max_scan_skew) {
      throw std::invalid_argument("duplicate sensor or unsynchronized scans");
    }
    for(const auto & scan:f.scans) {
      validateScan(scan);
      if(std::abs(scan.stamp-f.stamp)>cfg.max_scan_skew) {throw std::invalid_argument("scan/keyframe time mismatch");}
    }
    if(i) {for(int sensor=0;sensor<2;++sensor) {
      const auto & a=frames[0].scans[sensor];const auto & b=f.scans[sensor];
      if(a.sensor_id!=b.sensor_id||std::hypot(a.sensor.x-b.sensor.x,a.sensor.y-b.sensor.y)>1e-6||
        std::abs(wrap(a.sensor.yaw-b.sensor.yaw))>1e-6) {throw std::invalid_argument("sensor extrinsics/order changed during survey");}
    }}
  }
  std::map<std::string,int> group_ids;std::vector<std::vector<int>> groups;std::vector<int> group(walls.size());
  std::vector<Point> pivots(walls.size());
  for(std::size_t j=0;j<walls.size();++j) {
    const std::string key=walls[j].assembly.empty()?"wall:"+walls[j].id:"assembly:"+walls[j].assembly;
    if(!group_ids.count(key)) {group_ids[key]=groups.size();groups.emplace_back();}
    group[j]=group_ids.at(key);groups[group[j]].push_back(j);
  }
  for(const auto & members:groups) {
    V center=V::Zero();for(int j:members) {center+=v(walls[j].a)+v(walls[j].b);}center/=2*members.size();
    for(int j:members) {pivots[j]=point(center);}
  }
  const int count=3*(frames.size()+groups.size()), wall_base=3*frames.size();
  State state=State::Zero(count);
  for(std::size_t i=0;i<frames.size();++i) {
    const Pose p=compose(start,relative(frames[0].odom,frames[i].odom));state.segment<3>(3*i)<<p.x,p.y,p.yaw;
  }
  Estimate result; result.variables=count; std::vector<Match> matches;
  for(std::size_t i=0;i<frames.size();++i) {
    for(std::size_t sensor=0;sensor<2;++sensor) {
      for(auto segment:extract(frames[i].scans[sensor],cfg)) {
        segment.sensor=sensor; const V a=v(transform(pose(state,i),segment.a)),b=v(transform(pose(state,i),segment.b));
        const V t=(b-a).normalized(); double best=1e9,second=1e9;int chosen=-1;
        for(std::size_t j=0;j<walls.size();++j) {
          const V u=(v(walls[j].b)-v(walls[j].a)).normalized(),n{-u.y(),u.x()};
          if(std::acos(std::clamp(std::abs(t.dot(u)),0.,1.))>cfg.association_angle) {continue;}
          const double d=std::max(std::abs(n.dot(a-v(walls[j].a))),std::abs(n.dot(b-v(walls[j].a))));
          const double lo=std::min(u.dot(a-v(walls[j].a)),u.dot(b-v(walls[j].a)));
          const double hi=std::max(u.dot(a-v(walls[j].a)),u.dot(b-v(walls[j].a)));
          const double overlap=std::min(hi,(v(walls[j].b)-v(walls[j].a)).norm())-std::max(lo,0.);
          if(d>cfg.association_distance||overlap<cfg.min_segment*.5) {continue;}
          const double score=d+.02*(1-std::abs(t.dot(u)));
          if(score<best) {second=best;best=score;chosen=j;} else {second=std::min(second,score);}
        }
        if(chosen<0) {++result.unmatched;continue;}
        if(second-best<cfg.association_margin) {++result.ambiguous;continue;}
        const auto & w=walls[chosen];
        auto endpoint=[&](V p,bool visible) {
          if(!visible) {return -1;}
          const double da=(p-v(w.a)).norm(),db=(p-v(w.b)).norm();
          return std::min(da,db)<cfg.association_distance?(da<db?0:1):-1;
        };
        matches.push_back({static_cast<int>(i),chosen,segment,endpoint(a,segment.edge_a),endpoint(b,segment.edge_b)});
      }
    }
  }
  result.matched=matches.size();
  if(matches.empty()) {result.reason="no_unambiguous_wall_matches";return result;}
  auto linearize=[&](const State & x,bool geometry) {
    Linear linear(4*matches.size()+count+3*frames.size()+8,count);
    for(int k=0;k<3;++k) {
      const double target=k==0?start.x:k==1?start.y:start.yaw;
      linear.add(x,{k},[=](const State & y){return k==2?wrap(y[k]-target):y[k]-target;},
        k==2?cfg.start_yaw_sigma:cfg.start_sigma,false);
    }
    for(const auto & match:matches) {
      const int a=3*match.frame,b=wall_base+3*group[match.wall];
      const std::vector<int> ids{a,a+1,a+2,b,b+1,b+2};
      for(const auto & sample:std::vector<std::pair<Point,int>>{{match.segment.a,match.edge_a},{match.segment.b,match.edge_b}}) {
        linear.add(x,ids,[&](const State & y){
          const Wall w=moved(walls[match.wall],y,b,pivots[match.wall]);const V t=(v(w.b)-v(w.a)).normalized(),n{-t.y(),t.x()};
          return n.dot(v(transform(pose(y,match.frame),sample.first))-v(w.a));
        },cfg.line_sigma);
        if(sample.second>=0) {
          linear.add(x,ids,[&](const State & y){
            const Wall w=moved(walls[match.wall],y,b,pivots[match.wall]);const V t=(v(w.b)-v(w.a)).normalized();
            return t.dot(v(transform(pose(y,match.frame),sample.first))-v(sample.second==0?w.a:w.b));
          },cfg.edge_sigma);
        }
      }
    }
    if(!geometry) {
      for(std::size_t i=1;i<frames.size();++i) {
        const Pose delta=relative(frames[i-1].odom,frames[i].odom);const int a=3*(i-1),b=3*i;
        for(int k=0;k<3;++k) {linear.add(x,{a,a+1,a+2,b,b+1,b+2},[=](const State & y){
          const Pose d=relative(pose(y,i-1),pose(y,i));return k==0?d.x-delta.x:k==1?d.y-delta.y:wrap(d.yaw-delta.yaw);
        },k==2?cfg.odom_yaw_sigma:cfg.odom_sigma);}
      }
      for(std::size_t j=0;j<groups.size();++j) {
        for(int k=0;k<3;++k) {const int index=wall_base+3*j+k;
          linear.add(x,{index},[=](const State & y){return y[index];},k==2?.3:.2,false);}
      }
    }
    linear.a.conservativeResize(linear.rows,Eigen::NoChange);linear.b.conservativeResize(linear.rows);return linear;
  };
  bool converged=false;
  for(int iteration=0;iteration<30;++iteration) {
    const auto system=linearize(state,false);
    const State step=system.a.colPivHouseholderQr().solve(system.b);
    if(!step.allFinite()||step.norm()>5) {result.reason="optimizer_diverged";return result;}
    State next=state+step;
    for(std::size_t j=0;j<groups.size();++j) {
      const int b=wall_base+3*j;next[b+2]=std::clamp(next[b+2],-cfg.max_wall_yaw,cfg.max_wall_yaw);
      const State candidate=next;
      auto displacement=[&](double fraction) {
        State test=candidate;test.segment<3>(b)*=fraction;double maximum=0;
        for(int member:groups[j]) {const auto w=moved(walls[member],test,b,pivots[member]);
          maximum=std::max({maximum,(v(w.a)-v(walls[member].a)).norm(),(v(w.b)-v(walls[member].b)).norm()});}
        return maximum;
      };
      if(displacement(1)>cfg.max_wall_shift) {
        double low=0,high=1;
        for(int n=0;n<40;++n) {const double mid=(low+high)/2;if(displacement(mid)<=cfg.max_wall_shift) {low=mid;}else {high=mid;}}
        next.segment<3>(b)*=low;
      }
    }
    const double change=(next-state).norm();state=next;
    if(change<1e-7) {converged=true;break;}
  }
  if(!converged) {result.reason="optimizer_not_converged";return result;}
  const auto geometric=linearize(state,true);
  const Matrix h=geometric.a.transpose()*geometric.a;
  Eigen::SelfAdjointEigenSolver<Matrix> eigen(h);
  if(eigen.info()!=Eigen::Success) {result.reason="uncertainty_decomposition_failed";return result;}
  Matrix covariance=Matrix::Zero(count,count),nullspace=covariance;
  const double threshold=std::max(1e-8,eigen.eigenvalues().maxCoeff()*1e-9);
  for(int i=0;i<count;++i) {
    const State u=eigen.eigenvectors().col(i);
    if(eigen.eigenvalues()[i]>threshold) {covariance+=u*u.transpose()/eigen.eigenvalues()[i];++result.geometric_rank;}
    else {nullspace+=u*u.transpose();}
  }
  for(std::size_t i=0;i<frames.size();++i) {
    const int a=3*i;const double ps=std::sqrt(std::max(0.,covariance(a,a)+covariance(a+1,a+1))),ys=std::sqrt(std::max(0.,covariance(a+2,a+2)));
    const bool observable=nullspace.block<3,3>(a,a).trace()<1e-6;
    result.poses.push_back({pose(state,i),observable&&ps<=cfg.max_position_sigma&&ys<=cfg.max_yaw_sigma,
      observable?ps:std::numeric_limits<double>::infinity(),observable?ys:std::numeric_limits<double>::infinity()});
  }
  bool bounded=true;
  for(std::size_t j=0;j<walls.size();++j) {
    const int b=wall_base+3*group[j];const Wall w=moved(walls[j],state,b,pivots[j]);
    WallEstimate e;e.wall=w;e.max_displacement=std::max((v(w.a)-v(walls[j].a)).norm(),(v(w.b)-v(walls[j].b)).norm());
    std::set<int> views,edge_a_views,edge_b_views;bool separate=false;double residual=0;
    for(const auto & m:matches) {if(m.wall==static_cast<int>(j)) {
      views.insert(m.frame);
      if(m.edge_a==0||m.edge_b==0) {edge_a_views.insert(m.frame);}
      if(m.edge_a==1||m.edge_b==1) {edge_b_views.insert(m.frame);}
      const V t=(v(w.b)-v(w.a)).normalized(),n{-t.y(),t.x()};
      for(const auto & sample:std::vector<std::pair<Point,int>>{{m.segment.a,m.edge_a},{m.segment.b,m.edge_b}}) {
        const V p=v(transform(result.poses[m.frame].pose,sample.first));
        const double r=std::abs(n.dot(p-v(w.a)));residual=std::max(residual,r);
        result.residual_rms+=r*r;result.max_residual=std::max(result.max_residual,r);
        if(sample.second>=0) {const double er=std::abs(t.dot(p-v(sample.second==0?w.a:w.b)));residual=std::max(residual,er);result.max_residual=std::max(result.max_residual,er);}
      }
    }}
    for(int a:views) {for(int bview:views) {if(independent(result.poses[a].pose,result.poses[bview].pose,cfg)) {separate=true;}}}
    for(int a:edge_a_views) {for(int bview:edge_a_views) {if(independent(result.poses[a].pose,result.poses[bview].pose,cfg)) {e.edge_a=true;}}}
    for(int a:edge_b_views) {for(int bview:edge_b_views) {if(independent(result.poses[a].pose,result.poses[bview].pose,cfg)) {e.edge_b=true;}}}
    e.views=views.size();const double lever=std::max((v(walls[j].a)-v(pivots[j])).norm(),(v(walls[j].b)-v(pivots[j])).norm());
    e.position_sigma=std::sqrt(std::max(0.,covariance(b,b)+covariance(b+1,b+1)))+lever*std::sqrt(std::max(0.,covariance(b+2,b+2)));
    const bool observable=nullspace.block<3,3>(b,b).trace()<1e-6;
    e.verified=observable&&separate&&e.position_sigma<=cfg.max_position_sigma&&residual<=cfg.max_residual;
    const V tangent=(v(w.b)-v(w.a)).normalized();State normal=State::Zero(count);normal[b]=-tangent.y();normal[b+1]=tangent.x();
    const bool line_observable=normal.dot(nullspace*normal)<1e-6&&nullspace(b+2,b+2)<1e-6;
    e.line_sigma=std::sqrt(std::max(0.,normal.dot(covariance*normal)))+lever*std::sqrt(std::max(0.,covariance(b+2,b+2)));
    e.line_verified=line_observable&&separate&&e.line_sigma<=cfg.max_position_sigma&&residual<=cfg.max_residual;
    if(!line_observable) {e.line_sigma=std::numeric_limits<double>::infinity();}
    if(!observable) {e.position_sigma=std::numeric_limits<double>::infinity();}
    if(e.max_displacement>cfg.max_wall_shift+1e-5||std::abs(state[b+2])>cfg.max_wall_yaw+1e-8) {bounded=false;e.verified=false;e.line_verified=false;}
    result.walls.push_back(e);
  }
  result.residual_rms=std::sqrt(result.residual_rms/(2*matches.size()));
  const bool poses_good=std::all_of(result.poses.begin(),result.poses.end(),[](const auto & p){return p.verified;});
  result.accepted=bounded&&poses_good&&result.max_residual<=cfg.max_residual&&result.ambiguous==0;
  result.reason=!bounded?"wall_displacement_outside_envelope":!poses_good?"pose_geometry_unobservable_or_uncertain":
    result.max_residual>cfg.max_residual?"inconsistent_scan_geometry":result.ambiguous?"ambiguous_wall_association":"accepted";
  return result;
}

void Grid::validate() const {
  if(width<1||height<1||width>10000||height>10000||std::int64_t(width)*height>10000000||
    cells.size()!=static_cast<std::size_t>(width)*height||!finite(origin)||!std::isfinite(resolution)||resolution<.005||resolution>.05||
    std::any_of(cells.begin(),cells.end(),[](int x){return x!=-1&&x!=0&&x!=100;})) {throw std::invalid_argument("invalid trinary layout grid");}
}
int Grid::index(Point p) const {
  if(!finite(p)||p.x<origin.x||p.y<origin.y||p.x>=origin.x+width*resolution||p.y>=origin.y+height*resolution) {return -1;}
  return static_cast<int>((p.y-origin.y)/resolution)*width+static_cast<int>((p.x-origin.x)/resolution);
}
Point Grid::center(int i) const {return {origin.x+(i%width+.5)*resolution,origin.y+(i/width+.5)*resolution};}

MapUpdate updateMap(const Grid & prior,const std::vector<Wall> & walls,const std::vector<Frame> & frames,
  const Estimate & estimate,const Settings & cfg) {
  prior.validate();cfg.validate();MapUpdate result{prior,0,0,0};
  if(!estimate.accepted||estimate.poses.size()!=frames.size()||estimate.walls.size()!=walls.size()) {
    throw std::invalid_argument("map replacement requires an accepted matching estimate");
  }
  for(const auto & p:estimate.poses) {if(prior.index({p.pose.x,p.pose.y})<0) {throw std::invalid_argument("survey pose outside map");}}
  std::vector<std::vector<int>> free_views(prior.cells.size());std::vector<bool> hit(prior.cells.size());
  for(std::size_t i=0;i<frames.size();++i) {for(const auto & scan:frames[i].scans) {
    const Pose sensor=compose(estimate.poses[i].pose,scan.sensor);const V origin{sensor.x,sensor.y};
    for(std::size_t k=0;k<scan.ranges.size();++k) {
      const double range=scan.ranges[k];if(!validRange(scan,range)) {continue;}
      const double angle=sensor.yaw+scan.angle_min+k*scan.angle_increment;const V direction{std::cos(angle),std::sin(angle)};
      const int endpoint=prior.index(point(origin+range*direction));
      if(endpoint>=0) {hit[endpoint]=true;}
      // No clearing at/behind the return; no-return, NaN and max-range beams are unknown.
      for(double d=scan.range_min;d<range-2*prior.resolution;d+=prior.resolution*.5) {
        const int cell=prior.index(point(origin+d*direction));if(cell<0) {continue;}
        auto & views=free_views[cell];if(views.empty()||views.back()!=static_cast<int>(i)) {views.push_back(i);}
      }
    }
  }}
  for(std::size_t cell=0;cell<prior.cells.size();++cell) {
    const Point p=prior.center(cell);bool changed_wall=false,uncertain_wall=false;
    for(std::size_t j=0;j<walls.size();++j) {
      const double d=distance(p,walls[j].a,walls[j].b);
      const auto & w=estimate.walls[j];const V tangent=(v(walls[j].b)-v(walls[j].a)).normalized();
      const double along=tangent.dot(v(p)-v(walls[j].a)),length=(v(walls[j].b)-v(walls[j].a)).norm();
      const bool interior=along>cfg.max_wall_shift+prior.resolution&&along<length-cfg.max_wall_shift-prior.resolution;
      if((!w.line_verified||(!w.verified&&!interior))&&d<=cfg.max_wall_shift+prior.resolution) {uncertain_wall=true;}
      if(w.line_verified&&(w.verified||interior)&&d<=2*prior.resolution&&
        distance(p,estimate.walls[j].wall.a,estimate.walls[j].wall.b)>2*prior.resolution) {changed_wall=true;}
    }
    bool separate=false;
    for(int a:free_views[cell]) {for(int b:free_views[cell]) {if(independent(estimate.poses[a].pose,estimate.poses[b].pose,cfg)) {separate=true;}}}
    if(separate&&(prior.cells[cell]!=100||changed_wall)) {
      result.map.cells[cell]=0;
    }
    if(uncertain_wall) {result.map.cells[cell]=100;++result.uncertain_cells;}
    if(hit[cell]) {result.map.cells[cell]=100;++result.hits;}
    if(prior.cells[cell]==100&&result.map.cells[cell]==0) {++result.cleared;}
  }
  return result;
}

Crossing crossing(const Portal & p,const Estimate & e,double width) {
  Crossing out;
  if(!e.accepted||!std::isfinite(p.heading)||!std::isfinite(p.approach)||!std::isfinite(p.exit)||
    p.approach<=0||p.exit<=0||p.approach>2||p.exit>2||!std::isfinite(width)||width<=0||p.first==p.second) {
    out.reason="invalid_portal_or_estimate";return out;
  }
  const WallEstimate *first=nullptr,*second=nullptr;
  for(const auto & w:e.walls) {if(w.wall.id==p.first) {first=&w;}if(w.wall.id==p.second) {second=&w;}}
  if(!first||!second||!first->verified||!second->verified||
    !(p.first_b?first->edge_b:first->edge_a)||!(p.second_b?second->edge_b:second->edge_a)) {
    out.reason="opening_edges_not_verified";return out;
  }
  const V a=v(p.first_b?first->wall.b:first->wall.a),b=v(p.second_b?second->wall.b:second->wall.a);
  const V tangent=(b-a).normalized();const V expected{std::cos(p.heading),std::sin(p.heading)};
  V normal{-tangent.y(),tangent.x()};if(normal.dot(expected)<0) {normal=-normal;}
  if(std::acos(std::clamp(normal.dot(expected),-1.,1.))>.15) {out.reason="opening_orientation_mismatch";return out;}
  out.width=(b-a).norm();
  const double uncertainty=3*(first->position_sigma+second->position_sigma);
  if(out.width<=width+uncertainty) {out.reason="opening_too_narrow_or_uncertain";return out;}
  const V c=(a+b)/2;const double yaw=std::atan2(normal.y(),normal.x());
  for(double distance:{-p.approach,0.,p.exit}) {const V q=c+distance*normal;out.path.push_back({q.x(),q.y(),yaw});}
  out.accepted=true;out.reason="geometry_proposal_requires_swept_collision_check";return out;
}
}
