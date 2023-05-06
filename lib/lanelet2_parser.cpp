#include <iostream>

#include "lanelet2_parser/lanelet2_parser.hpp"
#include "lanelet2_parser/lanelet2.hpp"
#include "tinyxml2/tinyxml2.h"


bool LaneletParser::loadOSM(const std::string& osm_file)
{
  tinyxml2::XMLDocument doc;
  doc.LoadFile(osm_file.c_str());

  tinyxml2::XMLElement* root = doc.FirstChildElement("osm");
  if (root == NULL)
  {
    std::cerr << "\033[31;1mError: OSM NULL\033[m" << std::endl;
    exit(1);
  }

  for (tinyxml2::XMLElement* element = root->FirstChildElement("node"); element != NULL; element = element->NextSiblingElement("node"))
  {
    LL2Point p;
    double x, y;
    for (tinyxml2::XMLElement* tag = element->FirstChildElement("tag"); tag != NULL; tag = tag->NextSiblingElement("tag"))
    {
      if (std::string(tag->Attribute("k")) == std::string("local_x")) p.local_x = tag->DoubleAttribute("v");
      if (std::string(tag->Attribute("k")) == std::string("local_y")) p.local_y = tag->DoubleAttribute("v");
      if (std::string(tag->Attribute("k")) == std::string("ele")) p.elevation = tag->DoubleAttribute("v");
    }

    p.id = element->UnsignedAttribute("id");
    p.lat = element->DoubleAttribute("lat");
    p.lon = element->DoubleAttribute("lon");

    p.valid = true;

    ll2_points_.addNewPoint(p);
  }

  ll2_points_.makeVec();

  for (tinyxml2::XMLElement* element = root->FirstChildElement("way"); element != NULL; element = element->NextSiblingElement("way"))
  {
    LL2LineString ls;
    ls.id = element->UnsignedAttribute("id");

    for (tinyxml2::XMLElement* tag = element->FirstChildElement("nd"); tag != NULL; tag = tag->NextSiblingElement("nd"))
    {
      unsigned int id = tag->UnsignedAttribute("ref");
      ls.id_vec.push_back(id);
    }
    ls.valid = true;

    ll2_linestrings_.addNewLineString(ls);
  }

  for (tinyxml2::XMLElement* element = root->FirstChildElement("relation"); element != NULL; element = element->NextSiblingElement("relation"))
  {
    LL2LaneLet ll;
    ll.id = element->UnsignedAttribute("id");

    for (tinyxml2::XMLElement* tag = element->FirstChildElement("member"); tag != NULL; tag = tag->NextSiblingElement("member"))
    {
      if (std::string(tag->Attribute("role")) == std::string("right")) ll.right_id = tag->UnsignedAttribute("ref");
      if (std::string(tag->Attribute("role")) == std::string("left")) ll.left_id = tag->UnsignedAttribute("ref");
    }

    ll2_lanelets_.addNewLaneLet(ll);
  }

  return true;
}

bool LaneletParser::getLanePointVec(const unsigned int& lanelet_seq, std::vector<LL2Point>& right_lane, std::vector<LL2Point>& left_lane)
{
  LL2LaneLet ll = ll2_lanelets_.getLaneLetBySeq(lanelet_seq);

  LL2LineString line_string = ll2_linestrings_.getLineString(ll.right_id);
  right_lane.clear();
  right_lane.reserve(line_string.id_vec.size());
  for (int i = 0; i < line_string.id_vec.size(); i++)
  {
    LL2Point p = ll2_points_.getPoint(line_string.id_vec[i]);
    right_lane.push_back(p);
  }

  line_string = ll2_linestrings_.getLineString(ll.left_id);
  left_lane.clear();
  left_lane.reserve(line_string.id_vec.size());
  for (int i = 0; i < line_string.id_vec.size(); i++)
  {
    LL2Point p = ll2_points_.getPoint(line_string.id_vec[i]);
    left_lane.push_back(p);
  }

  return true;
}

bool LaneletParser::isInside(const double& x, const double& y, double& elev)
{
  for (int lanelet_id = 0; lanelet_id < ll2_lanelets_.size(); lanelet_id++)
  {
    std::vector<LL2Point> right_line, left_line;
    getLanePointVec(lanelet_id, right_line, left_line);

    int cross_num = 0;    
    LL2Point last_p = right_line[0];
    double min_range = (right_line[0].local_x - x) * (right_line[0].local_x - x) + (right_line[0].local_y - y) * (right_line[0].local_y);
    elev = right_line[0].elevation;
    for (int i = 1; i < right_line.size(); i++)
    {
      LL2Point q = right_line[i];
      if (isCross(x, y, last_p, q)) cross_num++;
      last_p = q;

      double range = (right_line[i].local_x - x) * (right_line[i].local_y - y) + (right_line[i].local_y - y) * (right_line[i].local_y - y); 
      if (range < min_range)
      {
        min_range = range;
        elev = right_line[i].elevation;
      }
    }

    for (int i = left_line.size() - 1; i >= 0; i--)
    {
      LL2Point q = left_line[i];
      if (isCross(x, y, last_p, q)) cross_num++;
      last_p = q;

      double range = (left_line[i].local_x - x) * (left_line[i].local_y - y) + (left_line[i].local_y - y) * (left_line[i].local_y - y); 
      if (range < min_range)
      {
        min_range = range;
        elev = left_line[i].elevation;
      }
    }

    if (isCross(x, y, last_p, right_line[0])) cross_num++;

    if (cross_num % 2 == 1) return true;
  }

  return false;
}

bool LaneletParser::isInside(const double& x, const double& y, double& elev, int& lane_id)
{
  for (int lanelet_id = 0; lanelet_id < ll2_lanelets_.size(); lanelet_id++)
  {
    std::vector<LL2Point> right_line, left_line;
    getLanePointVec(lanelet_id, right_line, left_line);

    int cross_num = 0;    
    LL2Point last_p = right_line[0];
    double min_range = (right_line[0].local_x - x) * (right_line[0].local_x - x) + (right_line[0].local_y - y) * (right_line[0].local_y);
    elev = right_line[0].elevation;
    for (int i = 1; i < right_line.size(); i++)
    {
      LL2Point q = right_line[i];
      if (isCross(x, y, last_p, q)) cross_num++;
      last_p = q;

      double range = (right_line[i].local_x - x) * (right_line[i].local_y - y) + (right_line[i].local_y - y) * (right_line[i].local_y - y); 
      if (range < min_range)
      {
        min_range = range;
        elev = right_line[i].elevation;
      }
    }

    for (int i = left_line.size() - 1; i >= 0; i--)
    {
      LL2Point q = left_line[i];
      if (isCross(x, y, last_p, q)) cross_num++;
      last_p = q;

      double range = (left_line[i].local_x - x) * (left_line[i].local_y - y) + (left_line[i].local_y - y) * (left_line[i].local_y - y); 
      if (range < min_range)
      {
        min_range = range;
        elev = left_line[i].elevation;
      }
    }

    if (isCross(x, y, last_p, right_line[0])) cross_num++;

    if (cross_num % 2 == 1) {
      lane_id = lanelet_id;
      return true;
    }
  }

  return false;
}

bool LaneletParser::isCross(const double& x, const double& y, const LL2Point& p, const LL2Point& q)
{
  if ((p.local_y < y && q.local_y < y) || (y < p.local_y && y < q.local_y))
    return false;

  if (p.local_y == q.local_y) return false;

  double test_x = p.local_x + (q.local_x - p.local_x) * (y - p.local_y) / (q.local_y - p.local_y);
  return test_x >= x;
}
