#include "common.hpp"
#include <nav2_map_server/map_io.hpp>
#include <random>
#include <iostream>
#include <queue>
#include <map>
using namespace maze_sim;
int main(int argc,char **argv) {
  try {
    if(argc!=8)throw std::runtime_error("prepare CONFIG NEW_OUTPUT SHIFT_M SEED inner|all matched|stale|partial ROUTE");
    const fs::path config=fs::absolute(argv[1]),out=fs::absolute(argv[2]);
    const double bound=std::stod(argv[3]);const unsigned seed=std::stoul(argv[4]);
    const std::string scope=argv[5],knowledge=argv[6],route=argv[7];
    if(!std::isfinite(bound)||bound<0||bound>.050000001||(scope!="inner"&&scope!="all")||(knowledge!="matched"&&knowledge!="stale"&&knowledge!="partial"))throw std::runtime_error("invalid shift, wall scope, or map mode");
    if(!fs::create_directory(out))throw std::runtime_error("output directory must be new");
    auto cfg=YAML::LoadFile(config.string());fs::path source=cfg["map"].as<std::string>();if(source.is_relative())source=config.parent_path()/source;
    Grid nominal=Grid::read(source),truth=nominal.clone();
    const Pose start=pose(cfg["start"]);std::vector<Pose> goals;
    if(cfg["routes"][route])for(const auto &n:cfg["routes"][route])goals.push_back(pose(n));
    else {std::ifstream f(route);std::string line;while(std::getline(f,line)){line=line.substr(0,line.find('#'));if(line.find_first_not_of(" \t\r")==std::string::npos)continue;std::istringstream row(line);Pose p;std::string extra;if(!(row>>p.x>>p.y>>p.yaw)||(row>>extra)||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw))throw std::runtime_error("invalid goal file");goals.push_back(p);}}
    if(goals.empty()||goals.size()>100)throw std::runtime_error("unknown route or empty/oversized goal file");
    if(!nominal.contains(start.x,start.y))throw std::runtime_error("start outside map");
    for(auto p:goals)if(!nominal.contains(p.x,p.y))throw std::runtime_error("goal outside map");
    struct Assembly {std::string id,kind;std::vector<cv::Point> cells;int dx=0,dy=0;Pose center;};
    std::vector<Assembly> groups;
    const int n=static_cast<int>(std::floor(bound/nominal.resolution+1e-8));std::vector<cv::Point> choices;
    for(int y=-n;y<=n;++y)for(int x=-n;x<=n;++x)if((x||y)&&std::hypot(x,y)*nominal.resolution<=bound+1e-9)choices.push_back({x,y});
    if(bound>0&&choices.empty())throw std::runtime_error("shift must be zero or at least one source-map cell");
    std::mt19937 rng(seed);cv::Mat owner(nominal.pixels.size(),CV_16S,cv::Scalar(-1));
    for(const auto &entry:cfg["assemblies"]) {
      Assembly a;a.id=entry["id"].as<std::string>();a.kind=entry["kind"].as<std::string>();
      for(int r=0;r<nominal.pixels.rows;++r)for(int c=0;c<nominal.pixels.cols;++c) {
        if(nominal.pixels.at<unsigned char>(r,c)!=0)continue;
        const auto p=nominal.center(r,c);bool selected=false;
        for(const auto &box:entry["boxes"])if(p.x>=box[0].as<double>()&&p.y>=box[1].as<double>()&&p.x<box[2].as<double>()&&p.y<box[3].as<double>())selected=true;
        if(!selected)continue;
        if(owner.at<short>(r,c)>=0)throw std::runtime_error("assembly masks overlap on occupied cells");
        owner.at<short>(r,c)=groups.size();a.cells.push_back({c,r});a.center.x+=p.x;a.center.y+=p.y;
      }
      if(a.cells.empty())throw std::runtime_error("empty assembly mask: "+a.id);
      a.center.x/=a.cells.size();a.center.y/=a.cells.size();
      if(bound>0&&(scope=="all"||a.kind=="inner")){auto d=choices[rng()%choices.size()];a.dx=d.x;a.dy=d.y;}
      groups.push_back(a);
    }
    for(const auto &g:groups)if(g.dx||g.dy)for(auto p:g.cells)truth.pixels.at<unsigned char>(p)=254;
    cv::Mat touched(nominal.pixels.size(),CV_8U,cv::Scalar(0));
    for(const auto &g:groups)for(auto p:g.cells) {
      const cv::Point q{p.x+g.dx,p.y-g.dy};if(!truth.inside(q))throw std::runtime_error("shift leaves map");
      truth.pixels.at<unsigned char>(q)=0;if(g.dx||g.dy){touched.at<unsigned char>(p)=1;touched.at<unsigned char>(q)=1;}
    }
    int changed=0,unknown_lost=0;for(int r=0;r<truth.pixels.rows;++r)for(int c=0;c<truth.pixels.cols;++c) {
      auto a=nominal.pixels.at<unsigned char>(r,c),b=truth.pixels.at<unsigned char>(r,c);
      if(a!=b){++changed;if(!touched.at<unsigned char>(r,c))throw std::runtime_error("unrelated map cell changed");}
      if(a==205&&b==254)++unknown_lost;
    }
    if(unknown_lost)throw std::runtime_error("unknown cells were cleared");
    const bool complete_world=knowledge=="partial" || cfg["complete_world"].as<bool>(false);
    std::size_t completed_unknown=0;
    if(complete_world)for(int r=0;r<truth.pixels.rows;++r)for(int c=0;c<truth.pixels.cols;++c)
      if(truth.pixels.at<unsigned char>(r,c)==205){truth.pixels.at<unsigned char>(r,c)=0;++completed_unknown;}
    auto navigation=(knowledge=="stale"?nominal:truth).clone();
    if(knowledge=="partial"){
      // Synthetic truth completion is explicit: unscanned exterior is solid.
      // Hide a band of the starting corridor and the destination footprints,
      // including any obstacles in those regions. Planner sees navigation only.
      auto hide=[&](double x0,double y0,double x1,double y1){cv::rectangle(navigation.pixels,navigation.pixel(x0,y1),navigation.pixel(x1,y0),cv::Scalar(205),-1);};
      hide(-1.86,1.2,-1.32,1.6);
      for(auto goal:goals)if(std::hypot(goal.x-start.x,goal.y-start.y)>.5)hide(goal.x-.3,goal.y-.3,goal.x+.3,goal.y+.3);
    }
    nominal.save(out,"nominal");truth.save(out,"truth");navigation.save(out,"navigation");
    nav_msgs::msg::OccupancyGrid loaded;
    if(nav2_map_server::loadMapFromYaml((out/"truth.yaml").string(),loaded)!=nav2_map_server::LOAD_MAP_SUCCESS)throw std::runtime_error("Nav2 map reload failed");
    std::size_t unknown=0;for(int r=0;r<truth.pixels.rows;++r)for(int c=0;c<truth.pixels.cols;++c) {
      const auto v=truth.pixels.at<unsigned char>(r,c);int expected=v==0?100:v==254?0:-1;
      if(loaded.data[(truth.pixels.rows-1-r)*truth.pixels.cols+c]!=expected)throw std::runtime_error("Nav2 occupancy encoding mismatch");
      if(expected<0)++unknown;
    }
    nav_msgs::msg::OccupancyGrid original;std::size_t original_unknown_as_free=0;
    if(nav2_map_server::loadMapFromYaml(source.string(),original)==nav2_map_server::LOAD_MAP_SUCCESS)for(int r=0;r<nominal.pixels.rows;++r)for(int c=0;c<nominal.pixels.cols;++c)
      if(nominal.pixels.at<unsigned char>(r,c)==205&&original.data[(nominal.pixels.rows-1-r)*nominal.pixels.cols+c]==0)++original_unknown_as_free;
    auto cost=truth.costmap();const auto f=footprint();
    if(!body_route_controller::footprintSafe(cost,f,start))throw std::runtime_error("start footprint blocked in truth map");
    for(const auto &g:goals)if(!body_route_controller::footprintSafe(cost,f,g))throw std::runtime_error("goal footprint blocked in truth map: "+std::to_string(g.x)+","+std::to_string(g.y));
    // A conservative circular center-connectivity check; actual Nav2 still plans and checks turns.
    cv::Mat free,dist;cv::compare(truth.pixels,254,free,cv::CMP_EQ);cv::distanceTransform(free,dist,cv::DIST_L2,cv::DIST_MASK_PRECISE);
    cv::Mat allowed=dist>(std::hypot(.18,.18)/truth.resolution+1.0);cv::Mat labels;cv::connectedComponents(allowed,labels,8,CV_32S);
    int component=labels.at<int>(truth.pixel(start.x,start.y));bool connected=component!=0;
    for(const auto &g:goals)connected=connected&&labels.at<int>(truth.pixel(g.x,g.y))==component;
    YAML::Node report;report["schema_version"]=1;report["source_map"]=source.string();report["start"]=poseNode(start);report["route"]=route;
    report["view_bounds"]=cfg["view_bounds"];report["map_mode"]=knowledge;report["wall_scope"]=scope;report["shift_bound_m"]=bound;report["seed"]=seed;
    report["matched_map_is_oracle"]=knowledge=="matched";
    report["ground_truth_complete"]=complete_world;report["synthetic_unknown_filled_occupied"]=completed_unknown;
    if(complete_world){report["world_completion"]="unscanned cells filled occupied; synthetic conservative completion";report["hidden_truth_usage"]="sensor simulation and independent collision/arrival evaluation only";}
    report["wall_adaptation_executed"]=false;report["changed_pixels"]=changed;
    report["unknown_cleared"]=unknown_lost;report["unknown_cells"]=unknown;report["source_unknown_loaded_as_free"]=original_unknown_as_free;
    report["simulation_free_thresh"]=.196;report["all_goal_footprints_clear"]=true;report["full_turn_circle_connectivity"]=connected;
    for(const auto &g:groups){YAML::Node a;a["id"]=g.id;a["kind"]=g.kind;a["occupied_pixels"]=g.cells.size();a["translation_m"]=poseNode({g.dx*nominal.resolution,g.dy*nominal.resolution,0});a["center"]=poseNode(g.center);report["assemblies"].push_back(a);}
    std::ofstream file(out/"goals.path");file<<std::setprecision(17);for(auto p:goals){file<<p.x<<' '<<p.y<<' '<<p.yaw<<'\n';report["goals"].push_back(poseNode(p));}
    saveYaml(out/"scenario.yaml",report);
    Canvas a(nominal,cfg["view_bounds"]),b(truth,cfg["view_bounds"]);
    a.text("Scanned map: original occupied pixels",22);b.text("Synthetic displaced walls: same scan raster",22);
    a.text("Simulation start and goals; map frame (metres)",45);b.text(complete_world?"Unscanned cells filled solid; fixed shelves retained":"Unknown retained; shelves outside wall masks stay fixed",45);
    for(auto *c:{&a,&b}){cv::circle(c->image,c->point(start),5,{180,80,10},-1);c->body(start,{180,80,10});int i=0;for(auto p:goals){cv::circle(c->image,c->point(p),4,{30,120,220},-1);cv::putText(c->image,std::to_string(++i),c->point(p)+cv::Point(6,-6),cv::FONT_HERSHEY_SIMPLEX,.45,{30,100,200},1,cv::LINE_AA);}}
    for(const auto &g:groups)if(g.dx||g.dy){auto p=b.point(g.center);auto q=b.point({g.center.x+g.dx*nominal.resolution,g.center.y+g.dy*nominal.resolution,0});cv::circle(b.image,p,5,{150,40,150},1);cv::arrowedLine(b.image,p,q,{150,40,150},2,cv::LINE_AA,0,.4);}
    Canvas nav(navigation,cfg["view_bounds"]);nav.text("Navigation input: purple unknown, black occupied",22);
    nav.body(start,{180,80,10});for(auto g:goals)nav.body(g,{30,120,220});nav.save(out/"navigation.png");
    a.save(out/"nominal.png");b.save(out/"truth.png");cv::Mat comparison;cv::hconcat(a.image,b.image,comparison);cv::imwrite((out/"layouts.png").string(),comparison);
    std::cout<<"Prepared "<<out<<"; changed_pixels="<<changed<<"; unknown_cleared=0; full_turn_circle_connectivity="<<connected<<"; source_unknown_loaded_as_free="<<original_unknown_as_free<<'\n';
    return 0;
  }catch(const std::exception &e){std::cerr<<"PREPARE FAILED: "<<e.what()<<'\n';return 2;}
}
