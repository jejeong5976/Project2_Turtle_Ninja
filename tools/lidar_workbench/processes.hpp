#pragma once
#include "options.hpp"
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <fstream>

namespace workbench {
inline double steady_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct ChildSpec { std::string name; std::vector<std::string> argv; bool required=true; };
class Processes {
  struct Child {pid_t pid;std::string name;bool required;bool reaped=false;};
  std::vector<Child> children_;
public:
  void start(const ChildSpec &spec,const std::filesystem::path &directory) {
    const int log=open((directory/(spec.name+".log")).c_str(),O_CREAT|O_WRONLY|O_EXCL,0600);
    require(log>=0,"Cannot create child log");
    const pid_t pid=fork();
    if(pid<0){close(log);throw std::runtime_error("fork failed");}
    if(pid==0) {
      setpgid(0,0);signal(SIGINT,SIG_DFL);signal(SIGTERM,SIG_DFL);signal(SIGHUP,SIG_DFL);
      const int null=open("/dev/null",O_RDONLY);dup2(null,STDIN_FILENO);close(null);
      dup2(log,STDOUT_FILENO);dup2(log,STDERR_FILENO);close(log);
      std::vector<char *> args;for(const auto &s:spec.argv)args.push_back(const_cast<char *>(s.c_str()));args.push_back(nullptr);
      execvp(args[0],args.data());_exit(127);
    }
    close(log);setpgid(pid,pid);children_.push_back({pid,spec.name,spec.required,false});
    std::ofstream pids(directory/"child_pids.csv",std::ios::app);
    pids<<spec.name<<','<<pid<<'\n';require(bool(pids),"Cannot record owned child PID");
  }
  void check() {
    for(auto &c:children_) if(!c.reaped) {
      int status=0;const auto result=waitpid(c.pid,&status,WNOHANG);
      if(result==c.pid){c.reaped=true;require(!c.required,"Required process exited: "+c.name+"; inspect its session log");}
    }
  }
  void stop() noexcept {
    // Only our child process groups; never pkill, device-owner killing, or external services.
    for(auto it=children_.rbegin();it!=children_.rend();++it) {
      kill(-it->pid,SIGCONT);
      kill(-it->pid,SIGINT);
      int status=0;double deadline=steady_now()+5;
      while(!it->reaped&&steady_now()<deadline) {
        const auto result=waitpid(it->pid,&status,WNOHANG);
        if(result==it->pid||(result<0&&errno==ECHILD))it->reaped=true;
        else std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      // A launcher can exit before its children. Bound cleanup for the whole owned group.
      if(kill(-it->pid,0)==0) {
        kill(-it->pid,SIGTERM);std::this_thread::sleep_for(std::chrono::milliseconds(200));
        kill(-it->pid,SIGKILL);
      }
      if(!it->reaped)waitpid(it->pid,&status,0);
      it->reaped=true;
    }
    children_.clear();
  }
  ~Processes(){stop();}
};
}  // namespace workbench
