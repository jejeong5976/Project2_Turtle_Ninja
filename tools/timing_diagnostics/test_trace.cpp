#include "nt_timing_trace.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>
using namespace nt_timing;
void check(bool v){if(!v)throw std::runtime_error("Trace assertion failed");}
std::vector<Record> read(const std::filesystem::path&p){std::ifstream f(p,std::ios::binary);std::vector<Record> r;Record v;while(f.read(reinterpret_cast<char*>(&v),sizeof v))r.push_back(v);check(f.gcount()==0);return r;}
int main(){
 try {
  Queue<4> q;Record r{};for(int i=0;i<4;++i){r.id=i;check(q.push(r));}check(!q.push(r));for(int i=0;i<4;++i){check(q.pop(r));check(r.id==uint64_t(i));}check(!q.pop(r));
  char path[]="/tmp/nt-trace-test-XXXXXX";check(mkdtemp(path));setenv("NT_TIMING_DIR",path,1);
  const auto name=[&](const char*label){return std::filesystem::path(path)/(std::string(label)+"_"+std::to_string(getpid())+".nttrace");};
  {
    Logger log("concurrent");std::vector<std::thread> threads;
    for(int t=0;t<4;++t)threads.emplace_back([&,t]{for(int i=0;i<1000;++i)log.event(Raw,t*1000+i,123,1,2,3,4,1.5,2.5,3.5);});
    for(auto &t:threads)t.join();
    log.close();
  }
  auto rows=read(name("concurrent"));check(rows.front().kind==Header&&rows.back().kind==End);std::set<uint64_t> ids;
  for(size_t i=1;i+1<rows.size();++i){check(rows[i].kind==Raw&&rows[i].stamp==123&&rows[i].d==4&&rows[i].z==3.5);ids.insert(rows[i].id);}
  check(ids.size()==rows.size()-2);check(rows.back().id==ids.size());check(rows.back().a==int64_t(ids.size()));check(ids.size()+rows.back().b==4000);
  bool refused=false;try{Logger again("concurrent");}catch(...){refused=true;}check(refused);
  {Logger limited("limited",3);for(int i=0;i<10;++i)limited.event(Read,i);}
  auto limited=read(name("limited"));check(limited.back().b==7&&limited.back().id==3);
  unsetenv("NT_TIMING_DIR");{Logger disabled("disabled");check(!disabled.enabled());disabled.event(Read);}
  std::cout<<"PASS queue full/wrap, concurrent identity/counts, loss footer, no overwrite, disabled mode: "<<path<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
