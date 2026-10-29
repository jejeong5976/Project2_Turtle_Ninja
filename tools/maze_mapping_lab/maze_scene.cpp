#include "common.hpp"
#include "validation.hpp"
#include <tinyxml2.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <png.h>
using namespace maze;
using namespace tinyxml2;
XMLElement* add(XMLDocument&d,XMLNode*p,const char*n){auto e=d.NewElement(n);p->InsertEndChild(e);return e;}
void attr(XMLElement*e,const char*k,const std::string&v){e->SetAttribute(k,v.c_str());}
void png(const fs::path&p,int w,int h,const std::vector<unsigned char>&rgb){
  FILE*f=fopen(p.c_str(),"wb");check(f,"Cannot open PNG");auto q=png_create_write_struct(PNG_LIBPNG_VER_STRING,nullptr,nullptr,nullptr);auto i=png_create_info_struct(q);
  if(setjmp(png_jmpbuf(q)))throw std::runtime_error("PNG encoding failed");
  png_init_io(q,f);png_set_IHDR(q,i,w,h,8,PNG_COLOR_TYPE_RGB,PNG_INTERLACE_NONE,PNG_COMPRESSION_TYPE_DEFAULT,PNG_FILTER_TYPE_DEFAULT);png_write_info(q,i);
  for(int y=h-1;y>=0;--y)png_write_row(q,const_cast<unsigned char*>(rgb.data()+y*w*3));
  png_write_end(q,i);png_destroy_write_struct(&q,&i);fclose(f);
}
void render(mjModel*m,mjData*d,const fs::path&out){
  auto get_display=reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
  EGLDisplay display=get_display?get_display(EGL_PLATFORM_SURFACELESS_MESA,EGL_DEFAULT_DISPLAY,nullptr):eglGetDisplay(EGL_DEFAULT_DISPLAY);
  EGLint major,minor;check(eglInitialize(display,&major,&minor),"Cannot initialize EGL (GPU sandbox access may require approval)");
  check(eglBindAPI(EGL_OPENGL_API),"Cannot bind OpenGL");EGLint config_attrs[]={EGL_SURFACE_TYPE,EGL_PBUFFER_BIT,EGL_RENDERABLE_TYPE,EGL_OPENGL_BIT,EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,EGL_DEPTH_SIZE,24,EGL_NONE};EGLConfig config;EGLint n;check(eglChooseConfig(display,config_attrs,&config,1,&n)&&n,"No EGL config");
  EGLint surface_attrs[]={EGL_WIDTH,1600,EGL_HEIGHT,1200,EGL_NONE};auto surface=eglCreatePbufferSurface(display,config,surface_attrs);auto context=eglCreateContext(display,config,EGL_NO_CONTEXT,nullptr);check(eglMakeCurrent(display,surface,surface,context),"Cannot make EGL context current");
  mjvScene scene; mjv_defaultScene(&scene);mjv_makeScene(m,&scene,30000);mjrContext ctx;mjr_defaultContext(&ctx);mjr_makeContext(m,&ctx,mjFONTSCALE_150);mjr_setBuffer(mjFB_OFFSCREEN,&ctx);mjvOption opt;mjv_defaultOption(&opt);opt.geomgroup[2]=0;opt.geomgroup[3]=1;
  mjvCamera camera;mjv_defaultCamera(&camera);camera.type=mjCAMERA_FREE;camera.lookat[0]=2.2;camera.lookat[1]=3.15;camera.lookat[2]=0;camera.distance=11.5;camera.azimuth=125;camera.elevation=-62;
  mjv_updateScene(m,d,&opt,nullptr,&camera,mjCAT_ALL,&scene);mjrRect rect{0,0,1600,1200};mjr_render(rect,&scene,&ctx);std::vector<unsigned char>rgb(1600*1200*3);mjr_readPixels(rgb.data(),nullptr,rect,&ctx);png(out/"maze_3d.png",1600,1200,rgb);
  camera.lookat[0]=d->qpos[0];camera.lookat[1]=d->qpos[1];camera.lookat[2]=.30;camera.distance=1.35;camera.azimuth=135;camera.elevation=-28;
  mjv_updateScene(m,d,&opt,nullptr,&camera,mjCAT_ALL,&scene);mjr_render(rect,&scene,&ctx);mjr_readPixels(rgb.data(),nullptr,rect,&ctx);png(out/"robot_C1_P0.png",1600,1200,rgb);
  mjr_freeContext(&ctx);mjv_freeScene(&scene);eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);eglDestroyContext(display,context);eglDestroySurface(display,surface);eglTerminate(display);
}
void pose_robot(mjModel*m,mjData*d,Pose p,const Json&pose){
  int root=mj_name2id(m,mjOBJ_JOINT,"benchmark_free");check(root>=0,"Missing free root");int q=m->jnt_qposadr[root];d->qpos[q]=p.x;d->qpos[q+1]=p.y;d->qpos[q+2]=.072;d->qpos[q+3]=cos(p.yaw/2);d->qpos[q+4]=0;d->qpos[q+5]=0;d->qpos[q+6]=sin(p.yaw/2);
  for(auto it=pose.begin();it!=pose.end();++it){int j=mj_name2id(m,mjOBJ_JOINT,it.key().c_str());check(j>=0,"Missing posture joint "+it.key());d->qpos[m->jnt_qposadr[j]]=it.value();}
  mj_forward(m,d);
}
void build_scene(const std::map<std::string,std::string>&a){
  fs::path out=option(a,"--output"),urdf=option(a,"--urdf"),arena=option(a,"--arena"),root=urdf.parent_path().parent_path();check(!out.empty()&&!fs::exists(out),"Scene output must be a new directory");fs::create_directories(out);
  Json j=read_json(arena);fs::copy_file(arena,out/"arena.json");fs::copy_file(urdf,out/"source_robot.urdf");fs::copy_file(root/"config/initial_pose.json",out/"initial_pose.json");
  XMLDocument u;check(u.LoadFile(urdf.c_str())==XML_SUCCESS,"Cannot parse URDF");auto robot=u.RootElement();
  auto walk=[&](auto&&self,XMLElement*e)->void{if(std::string(e->Name())=="mesh"){std::string f=e->Attribute("filename");const std::string prefix="package://nt_release_description/";check(f.rfind(prefix,0)==0,"Unexpected mesh package "+f);fs::path rel=f.substr(prefix.size());fs::create_directories((out/"assets"/rel).parent_path());if(!fs::exists(out/"assets"/rel))fs::copy_file(root/rel,out/"assets"/rel);attr(e,"filename",fs::absolute(out/"assets"/rel).string());}for(auto c=e->FirstChildElement();c;c=c->NextSiblingElement())self(self,c);};walk(walk,robot);
  auto compiler=add(u,add(u,robot,"mujoco"),"compiler");compiler->SetAttribute("discardvisual","false");compiler->SetAttribute("fusestatic","false");compiler->SetAttribute("strippath","false");compiler->SetAttribute("inertiafromgeom","false");
  u.SaveFile((out/"simulation_robot.urdf").c_str());auto robot_model=load_model(out/"simulation_robot.urdf");char error[2048]{};check(mj_saveLastXML((out/"robot_converted.xml").c_str(),robot_model.get(),error,sizeof(error)),error);
  XMLDocument x;check(x.LoadFile((out/"robot_converted.xml").c_str())==XML_SUCCESS,"Cannot parse MJCF");auto xml=x.RootElement();auto wb=xml->FirstChildElement("worldbody");
  auto root_body=wb->FirstChildElement("body");check(root_body&&std::string(root_body->Attribute("name"))=="base_link","Unexpected imported root body");add(x,root_body,"freejoint")->SetAttribute("name","benchmark_free");
  auto groups=[&](auto&&self,XMLElement*e)->void{if(std::string(e->Name())=="geom"){int ct=1;e->QueryIntAttribute("contype",&ct);e->SetAttribute("group",ct==0?1:2);}for(auto c=e->FirstChildElement();c;c=c->NextSiblingElement())self(self,c);};groups(groups,wb);
  auto visual=add(x,xml,"visual");auto global=add(x,visual,"global");global->SetAttribute("offwidth",1600);global->SetAttribute("offheight",1200);auto head=add(x,visual,"headlight");head->SetAttribute("ambient","0.4 0.4 0.4");
  auto floor=add(x,wb,"geom");floor->SetAttribute("name","floor");floor->SetAttribute("type","plane");floor->SetAttribute("size","5 6 .02");floor->SetAttribute("pos","2.2 3.15 -.002");floor->SetAttribute("rgba",".78 .79 .80 1");floor->SetAttribute("group",3);
  for(const auto&b:obstacles(j)){auto e=add(x,wb,"geom");attr(e,"name",b.name);e->SetAttribute("type",b.cylinder?"cylinder":"box");attr(e,"pos",numbers({b.x,b.y,b.z}));attr(e,"size",b.cylinder?numbers({b.hx,b.hz}):numbers({b.hx,b.hy,b.hz}));e->SetAttribute("group",0);e->SetAttribute("rgba",b.cylinder?".10 .25 .65 1":(b.name.rfind("wall",0)==0?".30 .40 .55 1":".72 .57 .31 1"));}
  // Inventory is decorative only: rays at 54.5 mm pass below these pucks.
  int puck=0;for(const auto&b:j["boxes"])if(std::string(b["name"]).rfind("source",0)==0)for(double dx:{-.04,.04}){auto e=add(x,wb,"geom");attr(e,"name","placeholder_puck_"+std::to_string(puck++));e->SetAttribute("type","cylinder");e->SetAttribute("size",".015 .030");attr(e,"pos",numbers({double(b["center"][0])+dx,b["center"][1],.10}));e->SetAttribute("rgba",dx<0?".8 .12 .1 1":".1 .6 .2 1");e->SetAttribute("group",0);}
  auto light=add(x,wb,"light");light->SetAttribute("pos","2 3 8");light->SetAttribute("dir","0 0 -1");light->SetAttribute("diffuse",".8 .8 .8");
  x.SaveFile((out/"maze.xml").c_str());auto m=load_model(out/"maze.xml");Data d(mj_makeData(m.get()));auto p=read_json(out/"initial_pose.json")["positions"];pose_robot(m.get(),d.get(),{0,0,0},p);
  double radius=0,minz=10,maxz=-10;std::array<double,4>bounds{10,10,-10,-10};
  for(int g=0;g<m->ngeom;++g)if(m->geom_group[g]==1||m->geom_group[g]==2){
    auto vertex=[&](double x0,double y0,double z0){const auto*R=d->geom_xmat+g*9;const auto*T=d->geom_xpos+g*3;double x1=T[0]+R[0]*x0+R[1]*y0+R[2]*z0,y1=T[1]+R[3]*x0+R[4]*y0+R[5]*z0,z1=T[2]+R[6]*x0+R[7]*y0+R[8]*z0;radius=std::max(radius,std::hypot(x1,y1));bounds[0]=std::min(bounds[0],x1);bounds[1]=std::min(bounds[1],y1);bounds[2]=std::max(bounds[2],x1);bounds[3]=std::max(bounds[3],y1);minz=std::min(minz,z1);maxz=std::max(maxz,z1);};
    if(m->geom_type[g]==mjGEOM_MESH){int id=m->geom_dataid[g];for(int k=0;k<m->mesh_vertnum[id];++k){const auto*v=m->mesh_vert+3*(m->mesh_vertadr[id]+k);vertex(v[0],v[1],v[2]);}}
    else{const auto*c=m->geom_aabb+6*g;for(int ix:{-1,1})for(int iy:{-1,1})for(int iz:{-1,1})vertex(c[0]+ix*c[3],c[1]+iy*c[4],c[2]+iz*c[5]);}
  }
  Json audit={{"engine",std::string("MuJoCo ")+mj_versionString()},{"source_urdf",urdf.string()},{"pose","C1_P0"},{"arm_positions",p},{"robot_radius_bound_m",radius},{"xy_bounds_m",bounds},{"z_bounds_m",{minz,maxz}},{"geometry_count",m->ngeom},{"bodies",m->nbody},{"joints",m->njnt},{"simulation_method","Kinematic route replay with 3D ray intersections; not wheel traction, motor torque or arm dynamics validation"},{"optical_overlay","Working mounts in base_footprint, z=.0545, yaw front=pi/rear=0; CAD base_link lifted .072 per its own footprint joint; no CAD optical TF published"},{"robot_ray_occlusion","Environment rays plus preserved baseline body mask. CAD self-occlusion is audited separately; CAD housings are not trusted optical calibrations."}};
  audit["urdf_radius_bound_m"]=radius;
  audit["robot_radius_bound_m"]=std::max(radius,std::hypot(.1376,.1625));
  audit["planning_envelope_basis"]="Larger of full C1_P0 CAD radial bound and preserved body-mask rectangle diagonal; then CLI clearance margin. Conservative in-plane, not verified physical swept envelope.";
  audit["robot_ray_occlusion"]="Fixed C1_P0 visual triangles intersected at preserved optical height; both CAD LiDAR housings excluded because their transforms differ; per-sensor blocked counts saved with each dataset.";
  write_json(out/"model_audit.json",audit);std::cout<<audit.dump(2)<<'\n';pose_robot(m.get(),d.get(),{.734,.950,pi},p);if(number(a,"--render",0)!=0)render(m.get(),d.get(),out);
}
std::vector<Pose> connect(Pose start,Pose goal,const std::vector<Obstacle>&o,double radius,Json&snaps){
  constexpr double step=.04;constexpr int nx=111,ny=159;auto idx=[](int x,int y){return y*nx+x;};
  auto free=[&](int x,int y){return x>=1&&x<nx-1&&y>=1&&y<ny-1&&clearance(x*step,y*step,o)>=radius;};
  auto nearest=[&](Pose p){double best=1e9;std::pair<int,int>r{-1,-1};for(int y=1;y<ny-1;++y)for(int x=1;x<nx-1;++x)if(free(x,y)){double ds=hypot(p.x-x*step,p.y-y*step);if(ds<best){best=ds;r={x,y};}}check(best<.45,"Waypoint has no nearby collision-free grid cell");if(best>.04)snaps.push_back({{"requested",{p.x,p.y}},{"used",{r.first*step,r.second*step}},{"distance_m",best}});return r;};
  auto s=nearest(start),g=nearest(goal);std::vector<double>dist(nx*ny,1e9);std::vector<int>parent(nx*ny,-1);using Q=std::pair<double,int>;std::priority_queue<Q,std::vector<Q>,std::greater<Q>>q;dist[idx(s.first,s.second)]=0;q.push({0,idx(s.first,s.second)});
  while(!q.empty()){auto [cost,k]=q.top();q.pop();(void)cost;if(k==idx(g.first,g.second))break;int x=k%nx,y=k/nx;for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)if((dx||dy)&&free(x+dx,y+dy)&&free(x+dx,y)&&free(x,y+dy)){int n=idx(x+dx,y+dy);double nd=dist[k]+step*hypot(dx,dy);if(nd<dist[n]){dist[n]=nd;parent[n]=k;q.push({nd+step*hypot(g.first-x-dx,g.second-y-dy),n});}}}
  int last=idx(g.first,g.second);check(dist[last]<1e8,"Route disconnected at required robot clearance: "+numbers({start.x,start.y,goal.x,goal.y}));std::vector<Pose>path;while(last>=0){path.push_back({last%nx*step,last/nx*step,0});last=parent[last];}std::reverse(path.begin(),path.end());
  auto safe=[&](Pose a,Pose b){int n=std::max(1,int(ceil(hypot(b.x-a.x,b.y-a.y)/.01)));for(int k=0;k<=n;++k){auto p=interp(a,b,double(k)/n);if(clearance(p.x,p.y,o)<radius-1e-6)return false;}return true;};
  if(safe(start,path.front()))path.insert(path.begin(),start);
  if(safe(path.back(),goal))path.push_back(goal);
  std::vector<Pose>smooth;for(size_t k=0;k<path.size();){smooth.push_back(path[k]);if(k+1==path.size())break;size_t n=path.size()-1;while(n>k+1&&!safe(path[k],path[n]))--n;k=n;}return smooth;
}
void generate(const std::map<std::string,std::string>&a){
  fs::path scene=option(a,"--scene"),out=option(a,"--output");check(!out.empty()&&!fs::exists(out),"Dataset output must be new");fs::create_directories(out);Json j=read_json(scene/"arena.json"),audit=read_json(scene/"model_audit.json");auto o=obstacles(j);
  std::string route=option(a,"--route","coverage_closed"),heading=option(a,"--heading","tangent");check(heading=="tangent"||heading=="fixed","Invalid heading policy");double speed=number(a,"--speed",.04),turn=number(a,"--turn-rate",.1),noise=number(a,"--range-noise",.005),scale=number(a,"--odom-scale",.01),drift=number(a,"--yaw-drift",.002),margin=number(a,"--margin",.04);int seed=int(number(a,"--seed",1));
  check(speed>0&&speed<=.10&&turn>0&&turn<=.5&&noise>=0&&noise<=.1&&std::abs(scale)<=.1&&std::abs(drift)<=.05&&margin>=0&&margin<=.2,"Invalid generation limits");double radius=double(audit["robot_radius_bound_m"])+margin;auto points=j.at("routes").at(route);Json snaps=Json::array();std::vector<Pose>path;Pose initial{points[0][0],points[0][1],pi};
  for(size_t i=1;i<points.size();++i){Pose start=path.empty()?initial:path.back(),goal{points[i][0],points[i][1],0};auto p=connect(start,goal,o,radius,snaps);if(!path.empty())p.erase(p.begin());path.insert(path.end(),p.begin(),p.end());}
  check(!path.empty(),"Empty route");path.front().yaw=pi;initial=path.front();std::vector<Sample>samples{{0,initial,{0,0,0}}};double t=0,distance=0;Pose truth=initial,odom{};
  auto append=[&](Pose next){Pose delta=relative(truth,next);double ds=hypot(delta.x,delta.y);Pose increment{delta.x*(1+scale),delta.y*(1-scale*.5),delta.yaw+drift*ds};odom=combine(odom,increment);t+=.02;truth=next;samples.push_back({t,truth,odom});distance+=ds;};
  auto hold=[&](double sec){for(int k=0;k<int(std::ceil(sec/.02));++k)append(truth);};
  auto move=[&](Pose goal,bool rotating){double amount=rotating?std::abs(wrap(goal.yaw-truth.yaw)):hypot(goal.x-truth.x,goal.y-truth.y);if(amount<1e-8)return;double v=rotating?turn:speed,acc=rotating?.8:.2,ta=std::min(v/acc,sqrt(amount/acc)),peak=acc*ta,tc=std::max(0.,(amount-acc*ta*ta)/peak),duration=2*ta+tc;int n=std::max(1,int(ceil(duration/.02)));Pose from=truth;for(int k=1;k<=n;++k){double elapsed=std::min(duration,k*.02),s;if(elapsed<ta)s=.5*acc*elapsed*elapsed;else if(elapsed<ta+tc)s=.5*acc*ta*ta+peak*(elapsed-ta);else s=amount-.5*acc*std::pow(duration-elapsed,2);append(interp(from,goal,std::clamp(s/amount,0.,1.)));}};
  hold(3);for(size_t i=1;i<path.size();++i){double yaw=heading=="fixed"?pi:atan2(path[i].y-truth.y,path[i].x-truth.x);if(hypot(path[i].x-truth.x,path[i].y-truth.y)<1e-8)continue;move({truth.x,truth.y,yaw},true);move({path[i].x,path[i].y,yaw},false);hold(.5);}move({truth.x,truth.y,pi},true);hold(5);
  auto m=load_model(scene/"maze.xml");Data d(mj_makeData(m.get()));pose_robot(m.get(),d.get(),{0,0,0},read_json(scene/"initial_pose.json")["positions"]);
  // The two CAD housings have a different optical convention. Exclude those
  // housings; use all other visual triangles for fixed-posture self-occlusion.
  for(int g=0;g<m->ngeom;++g){const char*n=mj_id2name(m.get(),mjOBJ_BODY,m->geom_bodyid[g]);if(n&&std::string(n).find("lidar_")==0)m->geom_group[g]=4;}
  mjtByte robot_groups[6]={0,1,0,0,0,0};std::array<double,800>self_range{};std::array<int,2>self_hits{};
  for(int side=0;side<2;++side)for(int k=0;k<400;++k){auto mount=j["lidars"][side];double angle=double(mount[3])-2*pi*k/400;mjtNum origin[3]={mount[0],mount[1],mount[2]},dir[3]={cos(angle),sin(angle),0};double r=mj_ray(m.get(),d.get(),origin,dir,robot_groups,1,-1,nullptr,nullptr);self_range[side*400+k]=r;if(r>=0)++self_hits[side];}
  write_json(out/"self_occlusion.json",{{"blocked_of_400",self_hits},{"status","CAD sensitivity, not measured optical coverage; LiDAR housings excluded because CAD and working extrinsics differ"}});
  mjtByte groups[6]={1,0,0,0,0,0};std::mt19937 rng(seed);std::normal_distribution<double>normal(0,noise);std::vector<ScanRecord>scans;size_t nscans=0;double minclear=100;
  for(const auto&s:samples)minclear=std::min(minclear,clearance(s.truth.x,s.truth.y,o)-double(audit["robot_radius_bound_m"]));
  for(double reference=.1;reference<=t-.001;reference+=.1){ScanRecord frame{};frame.t=reference;for(int side=0;side<2;++side)for(int k=0;k<400;++k){double bt=reference-(399-k)*.00025;auto p=sample_at(samples,bt);auto mount=j["lidars"][side];auto sensor=combine(p,{mount[0],mount[1],mount[3]});double angle=sensor.yaw-2*pi*k/400;mjtNum origin[3]={sensor.x,sensor.y,mount[2]},dir[3]={cos(angle),sin(angle),0};int hit=-1;double r=mj_ray(m.get(),d.get(),origin,dir,groups,1,-1,&hit,nullptr);double sr=self_range[side*400+k];if(sr>=0&&(r<0||sr<r))r=0;frame.ranges[side*400+k]=(r>=.05&&r<=12)?float(std::clamp(r+normal(rng),.05,12.)):0.f;}scans.push_back(frame);if(++nscans%1000==0)std::cout<<"generated "<<nscans<<" scans / "<<int(t*10)<<std::endl;}
  write_binary(out/"trajectory.bin",samples);write_binary(out/"scans.bin",scans);std::ofstream csv(out/"trajectory.csv");csv<<"time_s,truth_x,truth_y,truth_yaw,odom_x,odom_y,odom_yaw\n"<<std::setprecision(10);for(const auto&s:samples)csv<<s.t<<','<<s.truth.x<<','<<s.truth.y<<','<<s.truth.yaw<<','<<s.odom.x<<','<<s.odom.y<<','<<s.odom.yaw<<'\n';
  Json manifest={{"schema",1},{"scene",fs::absolute(scene).string()},{"route",route},{"heading",heading},{"speed_m_s",speed},{"turn_rate_rad_s",turn},{"duration_s",t},{"distance_m",distance},{"seed",seed},{"scan_count",scans.size()},{"sample_count",samples.size()},{"sample_bytes",sizeof(Sample)},{"scan_bytes",sizeof(ScanRecord)},{"initial_pose",{initial.x,initial.y,initial.yaw}},{"end_pose",{truth.x,truth.y,truth.yaw}},{"minimum_conservative_clearance_m",minclear},{"radius_bound_m",audit["robot_radius_bound_m"]},{"planning_margin_m",margin},{"waypoint_adjustments",snaps},{"noise_model",{{"status","Proposed sensitivity model, not calibrated to hardware"},{"range_sigma_m",noise},{"odom_forward_scale_error",scale},{"odom_lateral_scale_error",-scale*.5},{"yaw_drift_rad_per_m",drift}}},{"timing_model","10 Hz, 400 selected rays, 250 us nominal spacing, assumed timing offset zero; perfect simulated acquisition times are not hardware calibration"},{"limitations","Kinematic motion, stationary world, no wheel slip dynamics, no optical material/multipath/dropout model. Raw rays intersect 3D environment; preserved body mask applied by replay."}};write_json(out/"manifest.json",manifest);std::cout<<manifest.dump(2)<<'\n';
  std::ofstream svg(out/"route.svg");svg<<"<svg xmlns='http://www.w3.org/2000/svg' width='760' height='1000' viewBox='-.2 -.2 4.9 6.9'><rect x='-.2' y='-.2' width='4.9' height='6.9' fill='#f8fafc'/><g transform='translate(0 6.304) scale(1 -1)'>";for(const auto&b:o)svg<<"<rect x='"<<b.x-b.hx<<"' y='"<<b.y-b.hy<<"' width='"<<2*b.hx<<"' height='"<<2*b.hy<<"' fill='#5d7186'/>";svg<<"<polyline fill='none' stroke='#d36527' stroke-width='.025' points='";for(size_t i=0;i<samples.size();i+=20)svg<<samples[i].truth.x<<','<<samples[i].truth.y<<' ';svg<<"'/><circle cx='"<<initial.x<<"' cy='"<<initial.y<<"' r='.08' fill='#16a170'/><circle cx='"<<truth.x<<"' cy='"<<truth.y<<"' r='.05' fill='#e33355'/></g></svg>\n";
}
int main(int argc,char**argv){try{check(argc>=2,"maze_scene build|generate|render|validate|summarize [--key value]");auto a=args(argc,argv);std::string cmd=argv[1];if(cmd=="build")build_scene(a);else if(cmd=="generate")generate(a);else if(cmd=="validate")validate_scene(a);else if(cmd=="summarize")summarize(a);else if(cmd=="render"){fs::path scene=option(a,"--scene");auto m=load_model(scene/"maze.xml");Data d(mj_makeData(m.get()));pose_robot(m.get(),d.get(),{.734,.950,pi},read_json(scene/"initial_pose.json")["positions"]);render(m.get(),d.get(),scene);}else throw std::runtime_error("Unknown command");return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
