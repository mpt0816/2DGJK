#include "interface.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <iostream>

using namespace std;

namespace {

struct Point1D {
  double x;
};

struct Point2D : public Point1D {
  double y;
};

struct PathPoint : public Point2D {
  double z;
  double theta;
  double s;
};

typedef std::vector<PathPoint> Path;
typedef std::vector<gjk::Polygon> Obstacles;
} // namespace


constexpr int map_max = 200;

Path GeneratePath();

Obstacles GenerateObstacles(int num);

toolbox::TruckParam GenerateTruckParam();

void SetTruckParam(
    const toolbox::TruckParam& params,
    toolbox::TruckModel& truck);

void TransformPathToMarker(
    std::string marker_name,
    const Path& path,
    int index_collision,
    visualization_msgs::Marker& outputs);

void TransformObstaclesToMarkerArray(
    std::string marker_name,
    const Obstacles& obstacles,
    int index_collision,
    visualization_msgs::MarkerArray& outputs);

void TransformHeadToMarkerArray(
    std::string marker_name,
    const Path& path,
    const toolbox::TruckParam& trunk_param,
    int index_collision,
    visualization_msgs::MarkerArray& outputs);

void TransformTrailerToMarkerArray(
    std::string marker_name,
    const Path& path,
    const toolbox::TruckParam& trunk_param,
    int index_collision,
    visualization_msgs::MarkerArray& outputs);

void TransformSingleObstacleToMarker(
    std::string marker_name,
    const gjk::Polygon& obstacle,
    visualization_msgs::MarkerArray& outputs);

void TransformSingleHeadToMarker(
    std::string marker_name,
    const PathPoint& point,
    const toolbox::TruckParam& trunk_param,
    visualization_msgs::MarkerArray& outputs);

void TransformSingleTrailerToMarker(
    std::string marker_name,
    const PathPoint& point,
    const toolbox::TruckParam& trunk_param,
    visualization_msgs::MarkerArray& outputs);

void TransformSinglePathToMarker(
    std::string marker_name,
    const PathPoint& point,
    visualization_msgs::Marker& outputs);

int main(int argc, char* argv[]) {
  Path path = GeneratePath();
  Obstacles obstacles = GenerateObstacles(100);
  toolbox::TruckParam truck_param = GenerateTruckParam();
  double trailer_yaw_init = path.front().theta;
  double check_s = path.back().s;
  double interval = std::hypot(path[1].x - path[0].x, path[1].y - path[0].y);
  bool check_trailer = true;
  int test_interface = 1; // 1, 2, 3, 4

  toolbox::Collision<PathPoint> collision;

  cout << "<<----- test interface 01 ----->>" << endl;
  int idx_path_1, idx_obj_1;
  bool collision_1 = collision.DetectObstacle(
      path, obstacles, truck_param, check_trailer, trailer_yaw_init, check_s, interval, idx_path_1, idx_obj_1);
  if (collision_1) {
    cout << "path num : " << idx_path_1 << endl;
    cout << "obstacle num : " << idx_obj_1 << endl;
  } else {
    cout << "No collision!" << endl;
  }

  cout << "<<----- test interface 02 ----->>" << endl;
  int idx_path_2, idx_obj_2;
  bool collision_2 = false;
  for (int i = 0; i < path.size(); ++i) {
    collision_2 = collision.DetectObstacle(path[i], obstacles, truck_param, check_trailer, trailer_yaw_init, idx_obj_2);
    if (collision_2) {
      idx_path_2 = i;
      break;
    }
  }
  if (collision_2) {
    cout << "path num : " << idx_path_2 << endl;
    cout << "obstacle num : " << idx_obj_2 << endl;
  } else {
    cout << "No collision!" << endl;
  }

  cout << "<<----- test interface 03 ----->>" << endl;
  double distance_3 = collision.DetectObstacle(path.front(), obstacles.front(), truck_param, check_trailer, trailer_yaw_init);
  cout << "distance_3 = " << distance_3 << endl;

  cout << "<<----- test interface 04 ----->>" << endl;
  PathPoint wp_truck, wp_obstacle;
  double distance_4 = collision.DetectObstacle(path.front(), obstacles.front(), truck_param, check_trailer, trailer_yaw_init, wp_truck, wp_obstacle);
  cout << "distance_4 = " << distance_4 << endl;
  cout << "witness point of truck is (" << wp_truck.x << ", " << wp_truck.y << ")" << endl;
  cout << "witness point of obstacle is (" << wp_obstacle.x << ", " << wp_obstacle.y << ")" << endl;

  // ros rviz show the result
  ros::init(argc, argv, "gjk");
  ros::NodeHandle nh;
  ros::Publisher pub_path = nh.advertise<visualization_msgs::Marker>("path", 1, true);
  ros::Publisher pub_obstacle = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);
  ros::Publisher pub_head = nh.advertise<visualization_msgs::MarkerArray>("head", 1, true);
  ros::Publisher pub_trailer = nh.advertise<visualization_msgs::MarkerArray>("trailer", 1, true);

  ros::Publisher pub_wp_t = nh.advertise<visualization_msgs::Marker>("witness_point_t", 1, true);
  ros::Publisher pub_wp_o = nh.advertise<visualization_msgs::Marker>("witness_point_o", 1, true);
  ros::Publisher pub_s_o = nh.advertise<visualization_msgs::MarkerArray>("single_obstacles", 1, true);
  ros::Publisher pub_s_head = nh.advertise<visualization_msgs::MarkerArray>("signle_head", 1, true);
  ros::Publisher pub_s_trailer = nh.advertise<visualization_msgs::MarkerArray>("signle_trailer", 1, true);

  ros::Rate rate(1);
  while (ros::ok()) {
    visualization_msgs::Marker path_maker;
    visualization_msgs::MarkerArray obstacles_marker;
    visualization_msgs::MarkerArray head_marker;
    visualization_msgs::MarkerArray trailer_marker;

    visualization_msgs::Marker witness_point_t_maker;
    visualization_msgs::Marker witness_point_o_maker;

    visualization_msgs::MarkerArray single_obstacles_marker;
    visualization_msgs::MarkerArray single_head_marker;
    visualization_msgs::MarkerArray single_trailer_marker;

    switch (test_interface) {
      case 1:
        TransformPathToMarker("path", path, idx_path_1, path_maker);
        TransformObstaclesToMarkerArray("obstacles", obstacles, idx_obj_1, obstacles_marker);
        TransformHeadToMarkerArray("head", path, truck_param, idx_path_1, head_marker);
        TransformTrailerToMarkerArray("trailer", path, truck_param, idx_path_1, trailer_marker);
        break;
      case 2:
        TransformPathToMarker("path", path, idx_path_2, path_maker);
        TransformObstaclesToMarkerArray("obstacles", obstacles, idx_obj_2, obstacles_marker);
        TransformHeadToMarkerArray("head", path, truck_param, idx_path_2, head_marker);
        TransformTrailerToMarkerArray("trailer", path, truck_param, idx_path_2, trailer_marker);
        break;
      case 3:
        TransformSingleHeadToMarker("signle_head", path.front(), truck_param, single_head_marker);
        TransformSingleTrailerToMarker("signle_trailer", path.front(), truck_param, single_trailer_marker);
        TransformSingleObstacleToMarker("single_obstacles", obstacles.front(), single_obstacles_marker);
      case 4:
        TransformSingleHeadToMarker("signle_head", path.front(), truck_param, single_head_marker);
        TransformSingleTrailerToMarker("signle_trailer", path.front(), truck_param, single_trailer_marker);
        TransformSingleObstacleToMarker("single_obstacles", obstacles.front(), single_obstacles_marker);
        TransformSinglePathToMarker("witness_point_t", wp_truck, witness_point_t_maker);
        TransformSinglePathToMarker("witness_point_o", wp_obstacle, witness_point_o_maker);
      default:
        break;
    }
    pub_path.publish(path_maker);
    pub_obstacle.publish(obstacles_marker);
    pub_head.publish(head_marker);
    pub_trailer.publish(trailer_marker);

    pub_wp_t.publish(witness_point_t_maker);
    pub_wp_o.publish(witness_point_o_maker);
    pub_s_o.publish(single_obstacles_marker);
    pub_s_head.publish(single_head_marker);
    pub_s_trailer.publish(single_trailer_marker);
    rate.sleep();
  }
}

//Path GeneratePath() {
//  Path path;
//  PathPoint path_point;
//  constexpr int path_point_num = map_max * 2 + 1;
//  auto FuncDistance = [](PathPoint& pt0, PathPoint& pt1) {
//    return std::hypot(pt0.x - pt1.x, pt0.y - pt1.y);
//  };
//  for (int i = 0; i < path_point_num; ++i) {
//    path_point.x     = 0.5 * i;
//    path_point.y     = 0.5 * i;
//    path_point.theta = M_PI_2 / 2.0;
//
//    if (i == 0) {
//      path_point.s = 0.;
//    } else {
//      path_point.s = path[i - 1].s + FuncDistance(path[i - 1], path[i]);
//    }
//    path.push_back(path_point);
//  }
//  return path;
//}

Path GeneratePath() {
  Path path;
  PathPoint path_point;
  constexpr double step = 2.0;
  constexpr double T    = 0.1;
  constexpr double K    = 20.0;
  int path_point_num = std::floor(map_max / step);
  auto FuncDistance = [](PathPoint& pt0, PathPoint& pt1) {
    return std::hypot(pt0.x - pt1.x, pt0.y - pt1.y);
  };
  for (int i = 0; i < path_point_num; ++i) {
    path_point.x     = step * i;
    path_point.y     = K * std::sin(path_point.x * T) + map_max / 2.0;
    path_point.theta = std::atan2(K * T * std::cos(path_point.x * T), 1.0);
    if (i == 0) {
      path_point.s = 0.;
    } else {
      path_point.s = path[i - 1].s + FuncDistance(path[i - 1], path[i]);
    }
    path.push_back(path_point);
  }
  return path;
}

Obstacles GenerateObstacles(int num) {
  std::vector<gjk::Polygon> obstacles;
  srand((int)time(0)); // 产生随机种子
  // 生成的obstacle为矩形
  double length = 1.0;
  double width  = 1.0;
  for (int i = 0; i < num; ++i) {
    gjk::Polygon  obstacle;
    obstacle.num_vertex = 4;
    obstacle.vertices = new(double* [4]);
    for (int j = 0; j < 4; ++j) {
      obstacle.vertices[j] = new (double [2]);
    }

    double center_x = rand() / double(RAND_MAX) * map_max;
    double center_y = rand() / double(RAND_MAX) * map_max;
    double theta    = rand() / double(RAND_MAX) * M_PI * 2;

    obstacle.vertices[0][0] = center_x + length / 2.0 * std::cos(theta) - width / 2.0 * std::sin(theta);
    obstacle.vertices[0][1] = center_y + length / 2.0 * std::sin(theta) + width / 2.0 * std::cos(theta);

    obstacle.vertices[1][0] = center_x + length / 2.0 * std::cos(theta) + width / 2.0 * std::sin(theta);
    obstacle.vertices[1][1] = center_y + length / 2.0 * std::sin(theta) - width / 2.0 * std::cos(theta);

    obstacle.vertices[2][0] = center_x - length / 2.0 * std::cos(theta) + width / 2.0 * std::sin(theta);
    obstacle.vertices[2][1] = center_y - length / 2.0 * std::sin(theta) - width / 2.0 * std::cos(theta);

    obstacle.vertices[3][0] = center_x - length / 2.0 * std::cos(theta) - width / 2.0 * std::sin(theta);
    obstacle.vertices[3][1] = center_y - length / 2.0 * std::sin(theta) + width / 2.0 * std::cos(theta);

    obstacles.emplace_back(obstacle);
  }
//  for (int i = 0; i < 4; ++i) {
//    delete[] obstacle.vertices[i];
//  }
//  delete[] obstacle.vertices;
  return obstacles;
}

toolbox::TruckParam GenerateTruckParam() {
  toolbox::TruckParam truck_param;
  truck_param.head_base2front = 5.38;
  truck_param.head_base2tail  = 1.45;
  truck_param.head_width      = 2.54;
  truck_param.head_wheelbase  = 3.90;

  truck_param.trailer_base2front = 0.85;
  truck_param.trailer_base2tail  = 12.07;
  truck_param.trailer_width      = 2.54;
  truck_param.trailer_wheelbase  = 9.0;

  truck_param.trailer_base2head_base = 0.32;

  return truck_param;
}

void SetTruckParam(
    const toolbox::TruckParam& params,
    toolbox::TruckModel& truck) {
  truck.SetParam(params.trailer_base2head_base,
                 params.head_base2front,
                 params.head_base2tail,
                 params.head_width,
                 params.head_wheelbase,
                 params.trailer_base2front,
                 params.trailer_base2tail,
                 params.trailer_width,
                 params.trailer_wheelbase);
}

void TransformPathToMarker(
    std::string marker_name,
    const Path& path,
    int index_collision,
    visualization_msgs::Marker& outputs) {
  if (path.empty()) {
    return;
  }
  outputs.header.frame_id = "\gjk";
  outputs.header.stamp    = ros::Time::now();
  outputs.lifetime        = ros::Duration();
  outputs.ns              = marker_name;
  outputs.id              = 0;
  outputs.type            = visualization_msgs::Marker::POINTS;
  outputs.action          = visualization_msgs::Marker::ADD;

  outputs.scale.x         = 0.2;
  outputs.scale.y         = 0.2;
  outputs.scale.z         = 0.0;

  for (int i = 0; i < path.size(); ++i) {
    geometry_msgs::Point pt;
    std_msgs::ColorRGBA color;
    pt.x = path.at(i).x;
    pt.y = path.at(i).y;
    pt.z = 0.0;
    outputs.points.push_back(pt);

    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;
    if (i == index_collision) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
    }
    outputs.colors.push_back(color);
  }
}

void TransformObstaclesToMarkerArray(
    std::string marker_name,
    const std::vector<gjk::Polygon>& obstacles,
    int index_collision,
    visualization_msgs::MarkerArray& outputs) {
  if (obstacles.empty()) return;
  int id = 0;
  for (int i = 0; i < obstacles.size(); ++i) {
    gjk::Polygon polygon = obstacles.at(i);
    visualization_msgs::Marker output;
    output.header.frame_id = "\gjk";
    output.header.stamp    = ros::Time::now();
    output.lifetime        = ros::Duration();
    output.ns              = marker_name;
    output.id              = id;
    output.type            = visualization_msgs::Marker::LINE_STRIP;
    output.action          = visualization_msgs::Marker::ADD;

    output.color.r         = 1.0;
    output.color.g         = 1.0;
    output.color.b         = 0.0;
    output.color.a         = 1.0;

    if (i == index_collision) {
      output.color.r         = 1.0;
      output.color.g         = 0.0;
      output.color.b         = 1.0;
      output.color.a         = 1.0;
    }

    output.scale.x = 0.1;
    output.scale.y = 0.1;
    output.scale.z = 0.0;

    for (int j = 0; j < 4; ++j) {
      geometry_msgs::Point point;
      point.x = polygon.vertices[j][0];
      point.y = polygon.vertices[j][1];
      point.z = 0.0;
      output.points.push_back(point);
    }
    geometry_msgs::Point point;
    point.x = polygon.vertices[0][0];
    point.y = polygon.vertices[0][1];
    point.z = 0.0;
    output.points.push_back(point);
    outputs.markers.push_back(output);
    id += 1;
  }
}

void TransformHeadToMarkerArray(
    std::string marker_name,
    const Path& path,
    const toolbox::TruckParam& trunk_param,
    int index_collision,
    visualization_msgs::MarkerArray& outputs) {
  if (path.empty()) return;
  toolbox::VehicleModel vehicle;
  vehicle.SetParam(trunk_param.head_base2front,
                   trunk_param.head_base2tail,
                   trunk_param.head_width,
                   trunk_param.head_wheelbase);
  int id = 0;
  for (int i = 0; i < path.size(); ++i) {
    vehicle.RectangleRep<PathPoint>(path[i]);
    visualization_msgs::Marker output;
    output.header.frame_id = "\gjk";
    output.header.stamp    = ros::Time::now();
    output.lifetime        = ros::Duration();
    output.ns              = marker_name;
    output.id              = id;
    output.type            = visualization_msgs::Marker::LINE_STRIP;
    output.action          = visualization_msgs::Marker::ADD;

    if (i == index_collision) {
      output.color.r = 1.0;
      output.color.g = 0.0;
      output.color.b = 0.0;
      output.color.a = 1.0;

      output.scale.x = 0.25;
      output.scale.y = 0.25;
      output.scale.z = 0.0;
    } else {
      output.color.r = 1.0;
      output.color.g = 1.0;
      output.color.b = 1.0;
      output.color.a = 0.5;

      output.scale.x = 0.05;
      output.scale.y = 0.05;
      output.scale.z = 0.0;
    }

    for (int j = 0; j < 4; ++j) {
      geometry_msgs::Point point;
      point.x = vehicle.contour.vertices[j][0];
      point.y = vehicle.contour.vertices[j][1];
      point.z = 0.0;
      output.points.push_back(point);
    }
    geometry_msgs::Point point;
    point.x = vehicle.contour.vertices[0][0];
    point.y = vehicle.contour.vertices[0][1];
    point.z = 0.0;
    output.points.push_back(point);
    outputs.markers.push_back(output);
    id += 1;
  }
}

void TransformTrailerToMarkerArray(
    std::string marker_name,
    const Path& path,
    const toolbox::TruckParam& trunk_param,
    int index_collision,
    visualization_msgs::MarkerArray& outputs) {
  if (path.empty()) return;
  toolbox::TruckModel truck;
  SetTruckParam(trunk_param, truck);
  int id = 0;
  PathPoint trailer_pose;
  for (int i = 0; i < path.size(); ++i) {
    if (i == 0) {
      trailer_pose = truck.CalTrailerPose<PathPoint>(path.front(), path.front().theta);
    } else {
      PathPoint header_pose_last = path[i - 1];
      PathPoint header_pose_now  = path[i];
      trailer_pose = truck.CalTrailerPose<PathPoint>(header_pose_last, header_pose_now, trailer_pose.theta);
    }
    truck.trailer.RectangleRep<PathPoint>(trailer_pose);
    visualization_msgs::Marker output;
    output.header.frame_id = "\gjk";
    output.header.stamp    = ros::Time::now();
    output.lifetime        = ros::Duration();
    output.ns              = marker_name;
    output.id              = id;
    output.type            = visualization_msgs::Marker::LINE_STRIP;
    output.action          = visualization_msgs::Marker::ADD;

    if (i == index_collision) {
      output.color.r = 1.0;
      output.color.g = 0.0;
      output.color.b = 0.0;
      output.color.a = 1.0;

      output.scale.x = 0.25;
      output.scale.y = 0.25;
      output.scale.z = 0.0;
    } else {
      output.color.r = 0.0;
      output.color.g = 1.0;
      output.color.b = 0.0;
      output.color.a = 0.5;

      output.scale.x = 0.05;
      output.scale.y = 0.05;
      output.scale.z = 0.0;
    }

    for (int j = 0; j < 4; ++j) {
      geometry_msgs::Point point;
      point.x = truck.trailer.contour.vertices[j][0];
      point.y = truck.trailer.contour.vertices[j][1];
      point.z = 0.0;
      output.points.push_back(point);
    }
    geometry_msgs::Point point;
    point.x = truck.trailer.contour.vertices[0][0];
    point.y = truck.trailer.contour.vertices[0][1];
    point.z = 0.0;
    output.points.push_back(point);
    outputs.markers.push_back(output);
    id += 1;
  }
}

void TransformSingleObstacleToMarker(
    std::string marker_name,
    const gjk::Polygon& obstacle,
    visualization_msgs::MarkerArray& outputs) {
  int id = 0;
  visualization_msgs::Marker output;
  output.header.frame_id = "\gjk";
  output.header.stamp    = ros::Time::now();
  output.lifetime        = ros::Duration();
  output.ns              = marker_name;
  output.id              = id;
  output.type            = visualization_msgs::Marker::LINE_STRIP;
  output.action          = visualization_msgs::Marker::ADD;

  output.color.r         = 1.0;
  output.color.g         = 1.0;
  output.color.b         = 0.0;
  output.color.a         = 1.0;

  output.scale.x = 0.1;
  output.scale.y = 0.1;
  output.scale.z = 0.0;

  for (int j = 0; j < 4; ++j) {
    geometry_msgs::Point point;
    point.x = obstacle.vertices[j][0];
    point.y = obstacle.vertices[j][1];
    point.z = 0.0;
    output.points.push_back(point);
  }
  geometry_msgs::Point point;
  point.x = obstacle.vertices[0][0];
  point.y = obstacle.vertices[0][1];
  point.z = 0.0;
  output.points.push_back(point);
  outputs.markers.push_back(output);
}

void TransformSingleHeadToMarker(
    std::string marker_name,
    const PathPoint& pose,
    const toolbox::TruckParam& trunk_param,
    visualization_msgs::MarkerArray& outputs) {

  toolbox::VehicleModel vehicle;
  vehicle.SetParam(trunk_param.head_base2front,
                   trunk_param.head_base2tail,
                   trunk_param.head_width,
                   trunk_param.head_wheelbase);
  int id = 0;
  vehicle.RectangleRep<PathPoint>(pose);
  visualization_msgs::Marker output;
  output.header.frame_id = "\gjk";
  output.header.stamp    = ros::Time::now();
  output.lifetime        = ros::Duration();
  output.ns              = marker_name;
  output.id              = id;
  output.type            = visualization_msgs::Marker::LINE_STRIP;
  output.action          = visualization_msgs::Marker::ADD;

  output.color.r = 1.0;
  output.color.g = 1.0;
  output.color.b = 1.0;
  output.color.a = 1.0;

  output.scale.x = 0.25;
  output.scale.y = 0.25;
  output.scale.z = 0.0;

  for (int j = 0; j < 4; ++j) {
    geometry_msgs::Point point;
    point.x = vehicle.contour.vertices[j][0];
    point.y = vehicle.contour.vertices[j][1];
    point.z = 0.0;
    output.points.push_back(point);
  }
  geometry_msgs::Point point;
  point.x = vehicle.contour.vertices[0][0];
  point.y = vehicle.contour.vertices[0][1];
  point.z = 0.0;
  output.points.push_back(point);
  outputs.markers.push_back(output);
}

void TransformSingleTrailerToMarker(
    std::string marker_name,
    const PathPoint& pose,
    const toolbox::TruckParam& trunk_param,
    visualization_msgs::MarkerArray& outputs) {

  toolbox::TruckModel truck;
  SetTruckParam(trunk_param, truck);
  int id = 0;
  PathPoint trailer_pose;
  trailer_pose = truck.CalTrailerPose<PathPoint>(pose, pose.theta);
  truck.trailer.RectangleRep<PathPoint>(trailer_pose);
  visualization_msgs::Marker output;
  output.header.frame_id = "\gjk";
  output.header.stamp    = ros::Time::now();
  output.lifetime        = ros::Duration();
  output.ns              = marker_name;
  output.id              = id;
  output.type            = visualization_msgs::Marker::LINE_STRIP;
  output.action          = visualization_msgs::Marker::ADD;

  output.color.r = 1.0;
  output.color.g = 1.0;
  output.color.b = 1.0;
  output.color.a = 1.0;

  output.scale.x = 0.25;
  output.scale.y = 0.25;
  output.scale.z = 0.0;


  for (int j = 0; j < 4; ++j) {
    geometry_msgs::Point point;
    point.x = truck.trailer.contour.vertices[j][0];
    point.y = truck.trailer.contour.vertices[j][1];
    point.z = 0.0;
    output.points.push_back(point);
  }
  geometry_msgs::Point point;
  point.x = truck.trailer.contour.vertices[0][0];
  point.y = truck.trailer.contour.vertices[0][1];
  point.z = 0.0;
  output.points.push_back(point);
  outputs.markers.push_back(output);
}

void TransformSinglePathToMarker(
    std::string marker_name,
    const PathPoint& point,
    visualization_msgs::Marker& outputs) {
  outputs.header.frame_id = "\gjk";
  outputs.header.stamp    = ros::Time::now();
  outputs.lifetime        = ros::Duration();
  outputs.ns              = marker_name;
  outputs.id              = 0;
  outputs.type            = visualization_msgs::Marker::POINTS;
  outputs.action          = visualization_msgs::Marker::ADD;

  outputs.scale.x         = 1.0;
  outputs.scale.y         = 1.0;
  outputs.scale.z         = 0.0;

  geometry_msgs::Point pt;
  std_msgs::ColorRGBA color;
  pt.x = point.x;
  pt.y = point.y;
  pt.z = 0.0;
  outputs.points.push_back(pt);

  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;

  outputs.colors.push_back(color);
}