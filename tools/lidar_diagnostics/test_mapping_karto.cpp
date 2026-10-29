// Installed mapper-core check: NaN rays must not add clearing evidence.
#include <karto_sdk/Karto.h>
#include <memory>
#include <iostream>
void check(bool v){if(!v)throw std::runtime_error("Karto mapping assertion failed");}
class GridProbe:public karto::OccupancyGrid {
public:
  GridProbe():OccupancyGrid(300,300,{-7.5,-7.5},.05) {
    for(auto *g:{GetCellPassCounts(),GetCellHitsCounts()}){
      g->Resize(300,300);g->GetCoordinateConverter()->SetOffset({-7.5,-7.5});
    }
  }
  void add(karto::LocalizedRangeScan &scan){AddScan(&scan);}
  uint64_t passes(){uint64_t n=0;auto *g=GetCellPassCounts();for(int i=0;i<g->GetDataSize();++i)n+=g->GetDataPointer()[i];return n;}
};
int main(){
  std::unique_ptr<karto::LaserRangeFinder> laser(karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("mapping_unknown_test")));
  laser->SetMinimumRange(.05);laser->SetMaximumRange(12);laser->SetRangeThreshold(12);
  // Same [0,2pi) metadata and 1600-count convention as the planned ROS adapter.
  const float increment=2*std::acos(-1.)/1600;
  laser->SetMinimumAngle(0);laser->SetMaximumAngle(1599*increment);laser->SetAngularResolution(increment);laser->SetIs360Laser(false);
  check(laser->GetNumberOfRangeReadings()==1600);
  karto::SensorManager::GetInstance()->RegisterSensor(laser.get());
  karto::RangeReadingsVector ranges(1600,NAN);ranges[0]=2.;
  karto::LocalizedRangeScan hit(laser->GetName(),ranges);hit.SetCorrectedPose({0,0,0});check(laser->Validate(&hit));
  GridProbe grid;grid.add(hit);const auto before=grid.passes();check(before>0);
  ranges.assign(1600,NAN);karto::LocalizedRangeScan unknown(laser->GetName(),ranges);unknown.SetCorrectedPose({0,0,0});check(laser->Validate(&unknown));
  grid.add(unknown);check(grid.passes()==before);
  check(hit.GetPointReadings(true).size()==1&&unknown.GetPointReadings(true).empty());
  karto::SensorManager::GetInstance()->UnregisterSensor(laser.get());
  std::cout<<"PASS installed Karto: 1600 readings accepted; NaNs add neither hit nor clearing evidence\n";
}
