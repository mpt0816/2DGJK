#ifndef GJK_INTERFACE_INTERFACE_H_
#define GJK_INTERFACE_INTERFACE_H_

#include "../core/gjk.h"
#include "../interface/common/data.h"
#include <vector>

namespace toolbox {

class Collision {
public:
  // return false if no collision, index == MAX_INT
  // return true, the index of path and obstacle if collision
  template <class T>
  bool DetectObstacle(const std::vector<T>& path,
                      const std::vector<gjk::Polygon>& obstacles,
                      const TruckParam& truck_param,
                      const bool& check_trailer,
                      const double& trailer_yaw,
                      const double& check_s,
                      const double& interval,
                      int& idx_path,
                      int& idx_obj) {
    if (path.empty() || obstacles.empty()) {
      idx_path = idx_obj = std::numeric_limits<int>::max();
      return false;
    }

    if (!check_trailer) {
      return CheckVehicleModel<T>(path, obstacles, truck_param, check_s, interval, idx_path, idx_obj);
    } else {
      return CheckTruckModel<T>(path, obstacles, truck_param, trailer_yaw, check_s, interval, idx_path, idx_obj);
    }
  }

  // return false if no collision, , index == MAX_INT
  // return true, the index of path if collision
  template <class T>
  bool DetectObstacle(const T& head_pose,
                      const std::vector<gjk::Polygon>& obstacles,
                      const TruckParam& truck_param,
                      const bool& check_trailer,
                      const double& trailer_yaw,
                      int& idx_obj) {
    if (obstacles.empty()) {
      idx_obj = std::numeric_limits<int>::max();
      return false;
    }

    if (!check_trailer) {
      return CheckVehicleModel<T>(head_pose, obstacles, truck_param, idx_obj);
    } else {
      return CheckTruckModel<T>(head_pose, obstacles, truck_param, trailer_yaw, idx_obj);
    }
  }
  
  // return distance between truck and obstacle
  template <class T>
  double DetectObstacle(const T& head_pose,
                        const gjk::Polygon& obstacle,
                        const TruckParam& truck_param,
                        const bool& check_trailer,
                        const double& trailer_yaw) {
    if (!check_trailer) {
      GenerateVehicleModel(truck_param);
      vehicle_.RectangleRep<T>(head_pose);
      return gjk::Gjk::GjkLevel2(obstacle, vehicle_.contour);
    } else {
      GenerateTruckModel(truck_param);
      T trailer_pose = truck_.CalTrailerPose<T>(head_pose, trailer_yaw);
      truck_.head.RectangleRep<T>(head_pose);
      truck_.trailer.RectangleRep<T>(trailer_pose);
      double distance_h = gjk::Gjk::GjkLevel2(obstacle, truck_.head.contour);
      double distance_t = gjk::Gjk::GjkLevel2(obstacle, truck_.trailer.contour);
      return std::fmin(distance_h, distance_t);
    }
  }
  
  // return distance and witness points between truck and obstacle
  template <class T>
  double DetectObstacle(const T& head_pose,
                        const gjk::Polygon& obstacle,
                        const TruckParam& truck_param,
                        const bool& check_trailer,
                        const double& trailer_yaw,
                        T& wp_truck,
                        T& wp_obstacle) {
    if (!check_trailer) {
      double coord_t[2], coord_o[2];
      GenerateVehicleModel(truck_param);
      vehicle_.RectangleRep<T>(head_pose);
      double distance = gjk::Gjk::GjkLevel3(obstacle, vehicle_.contour, coord_o, coord_t);
      wp_truck.x = coord_t[0]; 
      wp_truck.y = coord_t[1];
      wp_obstacle.x = coord_o[0]; 
      wp_obstacle.y = coord_o[1];
      return distance;
    } else {
      double coord_t_h[2], coord_o_h[2], coord_t_t[2], coord_o_t[2];
      GenerateTruckModel(truck_param);
      T trailer_pose = truck_.CalTrailerPose<T>(head_pose, trailer_yaw);
      truck_.head.RectangleRep<T>(head_pose);
      truck_.trailer.RectangleRep<T>(trailer_pose);
      double distance_h = gjk::Gjk::GjkLevel3(obstacle, truck_.head.contour, coord_o_h, coord_t_h);
      double distance_t = gjk::Gjk::GjkLevel3(obstacle, truck_.trailer.contour, coord_o_t, coord_t_t);
      double distance = std::fmin(distance_h, distance_t);
      wp_truck.x = distance_h < distance_t ? coord_t_h[0] : coord_t_t[0]; 
      wp_truck.y = distance_h < distance_t ? coord_t_h[1] : coord_t_t[1]; 
      wp_obstacle.x = distance_h < distance_t ? coord_o_h[0] : coord_o_t[0];  
      wp_obstacle.y = distance_h < distance_t ? coord_o_h[1] : coord_o_t[1]; 
      return distance;
    }
  }

 private:
  template <class T>
  inline bool CheckVehicleModel(const std::vector<T>& path,
                                const std::vector<gjk::Polygon>& obstacles,
                                const TruckParam& truck_param,
                                const double& check_s,
                                const double& interval,
                                int& idx_path,
                                int& idx_obj) {
    idx_path = idx_obj = std::numeric_limits<int>::max();
    GenerateVehicleModel(truck_param);
    for (int i = 0; i < path.size();) {
      double foward = std::numeric_limits<double>::max();
      if (path[i].s > check_s) {
        return false;
      } else {
        vehicle_.RectangleRep<T>(path[i]);
        for (int j = 0; j < obstacles.size(); ++j) {
          double aabb_distance = AABBDistance(obstacles[j], vehicle_);
          if (aabb_distance > 0.01) {
            if (aabb_distance < foward) {
              foward = aabb_distance;
            }
          } else {
            double gjk_distance = gjk::Gjk::GjkLevel2(obstacles[j], vehicle_.contour);
            if (gjk_distance < 0.01) {
              idx_path = i; idx_obj = j;
              return true;
            } else if (gjk_distance < foward) {
              foward = gjk_distance;
            }
          }
        } 
      }
      int step = std::floor(foward / interval) ? std::floor(foward / interval) : 1;
      i += step;
    }
    return false;
  }

  template <class T>
  inline bool CheckVehicleModel(const T head_pose,
                                const std::vector<gjk::Polygon>& obstacles,
                                const TruckParam& truck_param,
                                int& idx_obj) {
    idx_obj = std::numeric_limits<int>::max();
    GenerateVehicleModel(truck_param);
    vehicle_.RectangleRep<T>(head_pose);
    for (int i = 0; i < obstacles.size(); ++i) {
      if (AABBCollide(obstacles[i], vehicle_)) { 
        if (gjk::Gjk::GjkLevel1(obstacles[i], vehicle_.contour)) {
          idx_obj = i;
          return true;
        }
      }    
    }
    return false;
  }
  
  template <class T>
  inline bool CheckTruckModel(const std::vector<T>& path,
                              const std::vector<gjk::Polygon>& obstacles,
                              const TruckParam& truck_param,
                              const double& trailer_yaw,
                              const double& check_s,
                              const double& interval,
                              int& idx_path,
                              int& idx_obj) {
    idx_path = idx_obj = std::numeric_limits<int>::max();
    GenerateTruckModel(truck_param);
    T head_pose, head_pose_last, trailer_pose;
    for (int i = 0; i < path.size();) {
      double foward = std::numeric_limits<double>::max();
      head_pose = path[i];
      if (path[i].s > check_s) {
        return false;
      } else {
        if (i == 0) {
          trailer_pose = truck_.CalTrailerPose<T>(head_pose, trailer_yaw);
        } else {
          head_pose_last = path.at(i - 1);
          trailer_pose = truck_.CalTrailerPose<T>(head_pose_last, head_pose, trailer_pose.theta);
        }
        truck_.head.RectangleRep<T>(head_pose);
        truck_.trailer.RectangleRep<T>(trailer_pose);
        for (int j = 0; j < obstacles.size(); ++j) {
          double aabb_distance_h = AABBDistance(obstacles[j], truck_.head);
          double aabb_distance_t = AABBDistance(obstacles[j], truck_.trailer);
          if (aabb_distance_h > 0.01 && aabb_distance_t > 0.01) {
            double min_distance = std::fmin(aabb_distance_h, aabb_distance_t);
            if (min_distance < foward) {
              foward = min_distance;
            }
          } else if (aabb_distance_h < 0.01 && aabb_distance_t < 0.01) {
            double gjk_distance = gjk::Gjk::GjkLevel2(obstacles[j], truck_.head.contour);
            if (gjk_distance < 0.01) {
              idx_path = i; idx_obj = j;
              return true;
            } else if (gjk_distance < foward) {
              foward = gjk_distance;
            }
            gjk_distance = gjk::Gjk::GjkLevel2(obstacles[j], truck_.trailer.contour);
            if (gjk_distance < 0.01) {
              idx_path = i; idx_obj = j;
              return true;
            } else if (gjk_distance < foward) {
              foward = gjk_distance;
            }
          } else if (aabb_distance_h < 0.01) {
            double gjk_distance = gjk::Gjk::GjkLevel2(obstacles[j], truck_.head.contour);
            if (gjk_distance < 0.01) {
              idx_path = i; idx_obj = j;
              return true;
            } else if (gjk_distance < foward) {
              foward = gjk_distance;
            }
          } else {
            double gjk_distance = gjk::Gjk::GjkLevel2(obstacles[j], truck_.trailer.contour);
            if (gjk_distance < 0.01) {
              idx_path = i; idx_obj = j;
              return true;
            } else if (gjk_distance < foward) {
              foward = gjk_distance;
            }
          }
        } 
      }
      int step = std::floor(foward / interval) ? std::floor(foward / interval) : 1;
      i += step;
    }
    return false;
  }

  template <class T>
  inline bool CheckTruckModel(const T head_pose,
                              const std::vector<gjk::Polygon>& obstacles,
                              const TruckParam& truck_param,
                              const double& trailer_yaw,
                              int& idx_obj) {
    idx_obj = std::numeric_limits<int>::max();
    GenerateTruckModel(truck_param);
    T trailer_pose = truck_.CalTrailerPose<T>(head_pose, trailer_yaw);
    truck_.head.RectangleRep<T>(head_pose);
    truck_.trailer.RectangleRep<T>(trailer_pose);
    for (int i = 0; i < obstacles.size(); ++i) {
      if (AABBCollide(obstacles[i], truck_.head)) {
        if (gjk::Gjk::GjkLevel1(obstacles[i], truck_.head.contour)) {
          idx_obj = i;
          return true;
        }
      }
      if (AABBCollide(obstacles[i], truck_.trailer)) {
        if (gjk::Gjk::GjkLevel1(obstacles[i], truck_.trailer.contour)) {
          idx_obj = i;
          return true;
        }
      }
    }
    return false;
  }

  inline void GenerateVehicleModel(const TruckParam& truck_param) {
    vehicle_.SetParam(truck_param.head_base2front,
                      truck_param.head_base2tail,
                      truck_param.head_width,
                      truck_param.head_wheelbase);
  }
  
  inline void GenerateTruckModel(const TruckParam& truck_param) {
    truck_.SetParam(truck_param.trailer_base2head_base,
                    truck_param.head_base2front,
                    truck_param.head_base2tail,
                    truck_param.head_width,
                    truck_param.head_wheelbase,
                    truck_param.trailer_base2front,
                    truck_param.trailer_base2tail,
                    truck_param.trailer_width,
                    truck_param.trailer_wheelbase);
  }

  inline bool AABBCollide(const gjk::Polygon& obstacle,
                          const VehicleModel& vehicle) {
    double p_min_x = std::numeric_limits<double>::max();
    double p_max_x = std::numeric_limits<double>::lowest();
    double p_min_y = std::numeric_limits<double>::max();
    double p_max_y = std::numeric_limits<double>::lowest();
    for (int i = 0; i < obstacle.num_vertex; ++i) {
      p_min_x = std::fmin(obstacle.vertices[i][0], p_min_x);
      p_max_x = std::fmax(obstacle.vertices[i][0], p_max_x);
      p_min_y = std::fmin(obstacle.vertices[i][1], p_min_y);
      p_max_y = std::fmax(obstacle.vertices[i][1], p_max_y);
    }
    return !(p_min_x > vehicle.max_x || p_max_x < vehicle.min_x ||
             p_min_y > vehicle.max_y || p_max_y < vehicle.min_y);
  }

  inline double AABBDistance(const gjk::Polygon& obstacle, 
                             const VehicleModel& vehicle) {
    double p_min_x = std::numeric_limits<double>::max();
    double p_max_x = std::numeric_limits<double>::lowest();
    double p_min_y = std::numeric_limits<double>::max();
    double p_max_y = std::numeric_limits<double>::lowest();
    for (int i = 0; i < obstacle.num_vertex; ++i) {
      double p_min_x = std::fmin(obstacle.vertices[i][0], p_min_x);
      double p_max_x = std::fmax(obstacle.vertices[i][0], p_max_x);
      double p_min_y = std::fmin(obstacle.vertices[i][1], p_min_y);
      double p_max_y = std::fmax(obstacle.vertices[i][1], p_max_y);
    }

    double gap_x = 0.0, gap_y = 0.0, gap = 0.0;
    if (p_min_x > vehicle.max_x) {
      gap_x = p_min_x - vehicle.max_x;
    } else if (p_max_x < vehicle.min_x) {
      gap_x = vehicle.min_x - p_max_x;
    } else {
      gap_x = 0.0;
    } 

    if (p_min_y > vehicle.max_y) {
      gap_y = p_min_y - vehicle.max_y;
    } else if (p_max_y < vehicle.min_y) {
      gap_y = vehicle.min_y - p_max_y;
    } else {
      gap_y = 0.0;
    }
    
    if (gap_x < 1.0 && gap_y < 1.0) {
      return 0.0;
    } else if (gap_x < 1.0) {
      return gap_y;
    } else if (gap_y < 1.0) {
      return gap_x;
    } else {
      float gap = static_cast<float>(gap_x * gap_x, gap_y * gap_y);
      return FastSqrt(gap);
    } 
  }

 private:
  VehicleModel vehicle_;
  TruckModel   truck_;
};

} // namespace toolbox

#endif // GJK_INTERFACE_INTERFACE_H_