#ifndef LANELET2_PARSER_HPP
#define LANELET2_PARSER_HPP

#include "lanelet2.hpp"

class LaneletParser
{
public:
  bool loadOSM(const std::string& osm_file);
  bool getLanePointVec(const unsigned int& lanelet_id, std::vector<LL2Point>& right_lane, std::vector<LL2Point>& left_lane);
  std::size_t size()
  {
    return ll2_lanelets_.size();
  }
  bool isInside(const double& x, const double& y, double& elev);

private:
  LL2Points ll2_points_;
  LL2LineStrings ll2_linestrings_;
  LL2LaneLets ll2_lanelets_;

  bool isCross(const double& x, const double& y, const LL2Point& p, const LL2Point& q);
};

#endif
