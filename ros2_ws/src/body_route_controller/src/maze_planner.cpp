#include "body_route_controller/maze_planner.hpp"
#include "body_route_controller/observation_layer.hpp"
#include <nav2_core/global_planner.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

namespace body_route_controller
{
namespace {
constexpr double pi = 3.14159265358979323846;
Point transform(Point p, Pose q) {return {q.x+std::cos(q.yaw)*p.x-std::sin(q.yaw)*p.y,
    q.y+std::sin(q.yaw)*p.x+std::cos(q.yaw)*p.y};}
double cross(Point a, Point b, Point c) {return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
// Convex hull of the endpoints contains every intermediate translated footprint.
Footprint swept(const Footprint & f, Pose a, Pose b) {
  std::vector<Point> p;
  for(auto q:f) {p.push_back(transform({q.x,q.y},a));p.push_back(transform({q.x,q.y},b));}
  std::sort(p.begin(),p.end(),[](auto u,auto v){return u.x==v.x?u.y<v.y:u.x<v.x;});
  std::vector<Point> h;
  for(auto q:p) {while(h.size()>1&&cross(h[h.size()-2],h.back(),q)<=0)h.pop_back();h.push_back(q);}
  const auto lower=h.size();
  for(auto i=p.rbegin()+1;i!=p.rend();++i) {while(h.size()>lower&&cross(h[h.size()-2],h.back(),*i)<=0)h.pop_back();h.push_back(*i);}
  if(!h.empty())h.pop_back();
  Footprint out;for(auto q:h) {geometry_msgs::msg::Point v;v.x=q.x;v.y=q.y;out.push_back(v);}return out;
}
std::vector<double> axis(double origin,double end,double step,double start,double goal) {
  std::vector<double> out{start,goal};
  for(double v=origin+step/2;v<end;v+=step)out.push_back(v);
  std::sort(out.begin(),out.end());
  out.erase(std::unique(out.begin(),out.end(),[](double a,double b){return std::abs(a-b)<1e-7;}),out.end());return out;
}
}
std::vector<Pose> planMaze(nav2_costmap_2d::Costmap2D & input,const Footprint & footprint,
  Pose start,Pose goal,MazePlannerSettings cfg,std::function<bool()> cancelled,const nav2_costmap_2d::Costmap2D * identity,const nav2_costmap_2d::Costmap2D * observed)
{
  nav2_costmap_2d::Costmap2D optimistic;
  if(!std::isfinite(cfg.unknown_penalty)||cfg.unknown_penalty<=0)throw nav2_core::PlannerException("unknown penalty must be positive");
  if(cfg.unknown_policy==UnknownPolicy::ObserveBeforeEnter){
    if(!identity||identity->getSizeInCellsX()!=input.getSizeInCellsX()||identity->getSizeInCellsY()!=input.getSizeInCellsY()||
      std::abs(identity->getResolution()-input.getResolution())>1e-8||std::abs(identity->getOriginX()-input.getOriginX())>1e-8||
      std::abs(identity->getOriginY()-input.getOriginY())>1e-8)throw nav2_core::PlannerException("missing or mismatched raw map identity");
    optimistic=input;
    for(unsigned y=0;y<input.getSizeInCellsY();++y)for(unsigned x=0;x<input.getSizeInCellsX();++x){
      if(identity->getCost(x,y)==254)optimistic.setCost(x,y,254);
      else if(identity->getCost(x,y)==255&&input.getCost(x,y)==255)optimistic.setCost(x,y,0);
    }
  }
  if(observed&&(observed->getSizeInCellsX()!=input.getSizeInCellsX()||observed->getSizeInCellsY()!=input.getSizeInCellsY()||
    std::abs(observed->getResolution()-input.getResolution())>1e-8||std::abs(observed->getOriginX()-input.getOriginX())>1e-8||
    std::abs(observed->getOriginY()-input.getOriginY())>1e-8))throw nav2_core::PlannerException("mismatched independent observation grid");
  auto &map=cfg.unknown_policy==UnknownPolicy::ObserveBeforeEnter?optimistic:input;
  // Integral raw-identity mask gives an O(1) footprint bounding-box penalty.
  // This approximation affects preference only; collision sweeps remain exact.
  const unsigned stride=map.getSizeInCellsX()+1;
  std::vector<unsigned> prefix;
  if(cfg.unknown_policy==UnknownPolicy::ObserveBeforeEnter){
    prefix.assign(std::size_t(stride)*(map.getSizeInCellsY()+1),0);
    for(unsigned y=0;y<map.getSizeInCellsY();++y)for(unsigned x=0;x<map.getSizeInCellsX();++x)
      prefix[(y+1)*stride+x+1]=(identity->getCost(x,y)==255&&(!observed||observed->getCost(x,y)!=0))+prefix[y*stride+x+1]+prefix[(y+1)*stride+x]-prefix[y*stride+x];
  }
  auto unknown_fraction=[&](const Footprint & f,Pose p){
    if(prefix.empty())return 0.0;
    double xmin=std::numeric_limits<double>::infinity(),ymin=xmin,xmax=-xmin,ymax=-xmin;
    for(auto v:f){auto q=transform({v.x,v.y},p);xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);}
    unsigned x0,y0,x1,y1;if(!map.worldToMap(xmin,ymin,x0,y0)||!map.worldToMap(xmax,ymax,x1,y1))return 1.0;
    const unsigned count=prefix[(y1+1)*stride+x1+1]-prefix[y0*stride+x1+1]-prefix[(y1+1)*stride+x0]+prefix[y0*stride+x0];
    return double(count)/((x1-x0+1)*(y1-y0+1));
  };
  for(double n:{start.x,start.y,start.yaw,goal.x,goal.y,goal.yaw,cfg.grid_step,cfg.turn_cost,cfg.cost_weight,cfg.max_seconds})
    if(!std::isfinite(n))throw nav2_core::PlannerException("Non-finite maze planning input");
  if(cfg.grid_step<map.getResolution()||cfg.grid_step>.2||cfg.turn_cost<0||cfg.cost_weight<0||cfg.max_seconds<=0)
    throw nav2_core::PlannerException("Invalid maze planning settings");
  if(cancelled())throw nav2_core::PlannerCancelled("Maze planning cancelled");
  unsigned mx,my;
  if(!map.worldToMap(start.x,start.y,mx,my))throw nav2_core::StartOutsideMapBounds("Maze start outside map");
  if(!map.worldToMap(goal.x,goal.y,mx,my))throw nav2_core::GoalOutsideMapBounds("Maze goal outside map");
  if(!footprintSafe(map,footprint,start))throw nav2_core::StartOccupied("Maze start footprint blocked");
  if(!footprintSafe(map,footprint,goal))throw nav2_core::GoalOccupied("Maze goal footprint blocked");
  const auto xs=axis(map.getOriginX(),map.getOriginX()+map.getSizeInCellsX()*map.getResolution(),cfg.grid_step,start.x,goal.x);
  const auto ys=axis(map.getOriginY(),map.getOriginY()+map.getSizeInCellsY()*map.getResolution(),cfg.grid_step,start.y,goal.y);
  const int nx=xs.size(),ny=ys.size();
  if(std::size_t(nx)*ny>500000)throw nav2_core::PlannerException("Maze lattice exceeds resource bound");
  auto index=[](const auto & values,double q){return int(std::min_element(values.begin(),values.end(),[q](double a,double b){return std::abs(a-q)<std::abs(b-q);})-values.begin());};
  const int begin=index(ys,start.y)*nx+index(xs,start.x),finish=index(ys,goal.y)*nx+index(xs,goal.x);
  auto pose_at=[&](int cell,int heading){return Pose{xs[cell%nx],ys[cell/nx],heading*pi/2};};
  auto heuristic=[&](int cell){return std::abs(xs[cell%nx]-goal.x)+std::abs(ys[cell/nx]-goal.y);};
  double radius=0;for(auto p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
  Footprint circle;for(int i=0;i<24;++i){geometry_msgs::msg::Point p;double a=i*2*pi/24;p.x=radius/std::cos(pi/24)*std::cos(a);p.y=radius/std::cos(pi/24)*std::sin(a);circle.push_back(p);}
  std::vector<signed char> rotation(nx*ny,-1),edges(4*nx*ny,-1);
  auto can_turn=[&](int cell){if(rotation[cell]<0)rotation[cell]=footprintSafe(map,circle,pose_at(cell,0));return rotation[cell]!=0;};
  const double inf=std::numeric_limits<double>::infinity();
  std::vector<double> distance(4*nx*ny,inf);std::vector<int> parent(4*nx*ny,-1);
  using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  for(int h=0;h<4;++h){const double a=std::abs(wrap(h*pi/2-start.yaw));if(a>1e-6&&!can_turn(begin)&&!turnSafe(map,footprint,start,h*pi/2))continue;
    distance[4*begin+h]=cfg.turn_cost*a/(pi/2);open.push({distance[4*begin+h]+heuristic(begin),4*begin+h});}
  const auto started=std::chrono::steady_clock::now();int best=-1;double best_cost=inf;int expanded=0;
  while(!open.empty()) {
    auto [score,id]=open.top();open.pop();const int cell=id/4,h=id%4;
    if(score>=best_cost)break;
    if(score>distance[id]+heuristic(cell)+1e-9)continue;
    if(++expanded%128==0){if(cancelled())throw nav2_core::PlannerCancelled("Maze planning cancelled");
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>cfg.max_seconds)throw nav2_core::PlannerTimedOut("Maze planning time bound");}
    if(cell==finish){const double a=std::abs(wrap(goal.yaw-h*pi/2));if(a<1e-6||can_turn(cell)||turnSafe(map,footprint,pose_at(cell,h),goal.yaw)){
      const double total=distance[id]+cfg.turn_cost*a/(pi/2);if(total<best_cost){best=id;best_cost=total;}continue;}}
    auto relax=[&](int next,double cost){if(distance[id]+cost+1e-10<distance[next]){distance[next]=distance[id]+cost;parent[next]=id;open.push({distance[next]+heuristic(next/4),next});}};
    const int x=cell%nx,y=cell/nx;
    const int tx=x+(h==0)-(h==2),ty=y+(h==1)-(h==3);
    if(tx>=0&&tx<nx&&ty>=0&&ty<ny){const int next=ty*nx+tx;auto a=pose_at(cell,h),b=pose_at(next,h);
      if(edges[id]<0){unsigned u,v;edges[id]=map.worldToMap(b.x,b.y,u,v)&&map.getCost(u,v)<254&&footprintSafe(map,swept(footprint,a,b),{});}
      if(edges[id]){unsigned u,v;map.worldToMap((a.x+b.x)/2,(a.y+b.y)/2,u,v);
        relax(4*next+h,std::hypot(a.x-b.x,a.y-b.y)*(1+cfg.cost_weight*map.getCost(u,v)/252.0+cfg.unknown_penalty*unknown_fraction(swept(footprint,a,b),{})));}}
    if(can_turn(cell)){const double turn=cfg.turn_cost*(1+cfg.unknown_penalty*unknown_fraction(circle,pose_at(cell,0)));
      relax(4*cell+(h+1)%4,turn);relax(4*cell+(h+3)%4,turn);}
  }
  if(best<0)throw nav2_core::NoValidPathCouldBeFound("No footprint-safe orthogonal maze route");
  std::vector<Pose> reversed;for(int id=best;id>=0;id=parent[id])reversed.push_back(pose_at(id/4,id%4));
  std::reverse(reversed.begin(),reversed.end());std::vector<Pose> corners;
  for(auto p:reversed){if(!corners.empty()&&std::hypot(p.x-corners.back().x,p.y-corners.back().y)<1e-7)continue;
    if(corners.size()>1){const auto a=corners[corners.size()-2],b=corners.back();if(std::abs((b.x-a.x)*(p.y-b.y)-(b.y-a.y)*(p.x-b.x))<1e-10&&
        (b.x-a.x)*(p.x-b.x)+(b.y-a.y)*(p.y-b.y)>0)corners.pop_back();}corners.push_back(p);}
  std::vector<Pose> out;
  for(std::size_t i=0;i+1<corners.size();++i){auto a=corners[i],b=corners[i+1];double d=std::hypot(b.x-a.x,b.y-a.y),yaw=std::atan2(b.y-a.y,b.x-a.x);
    int n=std::max(1,int(std::ceil(d/.02)));for(int j=0;j<n;++j){double t=double(j)/n;out.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),yaw});}}
  out.push_back(goal);return out;
}

class MazePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(const nav2::LifecycleNode::WeakPtr & parent,std::string name,
    nav2::TransformBuffer::SharedPtr,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    node_=parent;map_=map;auto n=parent.lock();if(!n||!map)throw nav2_core::PlannerException("Missing maze planner dependencies");
    cfg_.unknown_policy=unknownPolicy(n->declare_or_get_parameter(name+".unknown_policy",std::string("blocked")));
    cfg_.unknown_penalty=n->declare_or_get_parameter(name+".unknown_penalty",2.0);
    if(cfg_.unknown_policy==UnknownPolicy::ObserveBeforeEnter)layer_=observationLayer(map);
    cfg_.grid_step=n->declare_or_get_parameter(name+".grid_step",cfg_.grid_step);
    cfg_.turn_cost=n->declare_or_get_parameter(name+".turn_cost",cfg_.turn_cost);
    cfg_.cost_weight=n->declare_or_get_parameter(name+".cost_weight",cfg_.cost_weight);
    cfg_.max_seconds=n->declare_or_get_parameter(name+".max_seconds",cfg_.max_seconds);
  }
  void cleanup() override {map_.reset();} void activate() override {} void deactivate() override {}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & via,std::function<bool()> cancelled) override {
    auto n=node_.lock();if(!n||!map_)throw nav2_core::PlannerException("Maze planner not configured");
    if(!via.empty())throw nav2_core::PlannerException("Submit maze viapoints as sequential goals");
    if(start.header.frame_id!=map_->getGlobalFrameID()||goal.header.frame_id!=map_->getGlobalFrameID())throw nav2_core::PlannerException("Maze poses must use costmap frame");
    auto *cost=map_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cost->getMutex());
    auto convert=[](const auto & p){return Pose{p.position.x,p.position.y,tf2::getYaw(p.orientation)};};
    nav2_costmap_2d::Costmap2D raw,observed;if(layer_){raw=layer_->identityMap();observed=layer_->observedMap();}
    auto route=planMaze(*cost,map_->getRobotFootprint(),convert(start.pose),convert(goal.pose),cfg_,cancelled,layer_?&raw:nullptr,layer_?&observed:nullptr);
    nav_msgs::msg::Path path;path.header.frame_id=map_->getGlobalFrameID();path.header.stamp=n->now();
    for(auto p:route){geometry_msgs::msg::PoseStamped q;q.header=path.header;q.pose.position.x=p.x;q.pose.position.y=p.y;q.pose.orientation.z=std::sin(p.yaw/2);q.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(q);}return path;
  }
private:
  std::shared_ptr<ObservationLayer> layer_;
  nav2::LifecycleNode::WeakPtr node_;std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;MazePlannerSettings cfg_;
};
}
PLUGINLIB_EXPORT_CLASS(body_route_controller::MazePlanner,nav2_core::GlobalPlanner)
