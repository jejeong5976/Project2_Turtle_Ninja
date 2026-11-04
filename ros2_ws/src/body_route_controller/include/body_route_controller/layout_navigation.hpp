#pragma once
#include "body_route_controller/layout.hpp"
#include "body_route_controller/collision.hpp"

namespace body_route_controller::layout
{
// Uses the same polygon collision implementation as the actual custom controller.
// Checks all translation samples and complete turns, including the current pose.
Crossing checkedCrossing(const Portal &,const Estimate &,const Grid &,const Footprint &);
}
