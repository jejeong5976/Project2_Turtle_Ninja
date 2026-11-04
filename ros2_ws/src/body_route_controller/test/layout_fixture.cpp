#include "layout_fixture_io.hpp"
#include <iostream>
int main(int argc,char** argv) {
  try {if(argc!=2) {throw std::runtime_error("layout_fixture NEW_DIRECTORY (synthetic only)");}layout_test::fixture(argv[1]);return 0;}
  catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
