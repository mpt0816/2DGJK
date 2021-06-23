#ifndef GJK_INTERFACE_DATA_H_
#define GJK_INTERFACE_DATA_H_

#include "../../common/data.h"
#include "../../interface/common/util.h"
#include <cmath>
#include <limits>

namespace toolbox {

struct TruckParam {
  double trailer_base2head_base;
  double head_base2front;
  double head_base2tail;
  double head_width;
  double head_wheelbase;
  double trailer_base2front;
  double trailer_base2tail;
  double trailer_width;
  double trailer_wheelbase;

  explicit TruckParam(double trailer_base2head_base,
                      double head_base2front,
                      double head_base2tail,
                      double head_width,
                      double head_wheelbase,
                      double trailer_base2front,
                      double trailer_base2tail,
                      double trailer_width,
                      double trailer_wheelbase)
    : trailer_base2head_base (trailer_base2head_base)
    , head_base2front(head_base2front)
    , head_base2tail(head_base2tail)
    , head_width(head_width)
    , head_wheelbase(head_wheelbase)
    , trailer_base2front(trailer_base2front)
    , trailer_base2tail(trailer_base2tail)
    , trailer_width(trailer_width)
    , trailer_wheelbase(trailer_wheelbase) {}

  TruckParam() = default;
  ~TruckParam() = default;
};

class VehicleModel {
 public:
  double base2front;
  double base2tail;
  double width;
  double wheelbase;
  gjk::Polygon contour;

  double min_x;
  double max_x;
  double min_y;
  double max_y;

  VehicleModel()
      : base2front(0.)
      , base2tail(0.)
      , width(0.)
      , wheelbase(0.)
      , min_x(std::numeric_limits<double>::max())
      , max_x(std::numeric_limits<double>::lowest())
      , min_y(std::numeric_limits<double>::max())
      , max_y(std::numeric_limits<double>::lowest()) {
    contour.num_vertex = 4;
    contour.vertices = new(double* [4]);
    for (int i = 0; i < 4; ++i) {
      contour.vertices[i] = new (double [2]);
    }
  }
  
  void SetParam(const double front, const double tail, 
                const double wd, const double wb) {
    base2front = front;
    base2tail  = tail;
    width      = wd;
    wheelbase  = wb;
  }

  template <class T>
  void RectangleRep(const T& pose) {
    double base_x = pose.x, base_y = pose.y;
    double half_width = width / 2.0;

    contour.vertices[0][0] = base_x + base2front * std::cos(pose.theta) -
        half_width * std::sin(pose.theta);
    contour.vertices[0][1] = base_y + base2front * std::sin(pose.theta) +
        half_width * std::cos(pose.theta);

    contour.vertices[1][0] = base_x + base2front * std::cos(pose.theta) +
        half_width * std::sin(pose.theta);
    contour.vertices[1][1] = base_y + base2front * std::sin(pose.theta) -
        half_width * std::cos(pose.theta);

    contour.vertices[2][0] = base_x - base2tail * std::cos(pose.theta) +
        half_width * std::sin(pose.theta);
    contour.vertices[2][1] = base_y - base2tail * std::sin(pose.theta) -
        half_width * std::cos(pose.theta);

    contour.vertices[3][0] = base_x - base2tail * std::cos(pose.theta) -
        half_width * std::sin(pose.theta);
    contour.vertices[3][1] = base_y - base2tail * std::sin(pose.theta) +
        half_width * std::cos(pose.theta);

    min_x = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    min_y = std::numeric_limits<double>::max();
    max_y = std::numeric_limits<double>::lowest();
    for (int i = 0; i < 4; ++i) {
      min_x = std::fmin(contour.vertices[i][0], min_x);
      max_x = std::fmax(contour.vertices[i][0], max_x);
      min_y = std::fmin(contour.vertices[i][1], min_y);
      max_y = std::fmax(contour.vertices[i][1], max_y);
    }
  }

  ~VehicleModel() {
    for (int i = 0; i < 4; ++i) {
      delete[] contour.vertices[i];
    }
    delete[] contour.vertices;
  }
};

class TruckModel {
 public:
  double trailer_base2head_base; 
  VehicleModel head;
  VehicleModel trailer;

  TruckModel()
      : trailer_base2head_base(0.), 
        head(), 
        trailer() {}

  void SetParam(const double base_diff, 
                const double h_front, 
                const double h_tail, 
                const double h_wd, 
                const double h_wb,
                const double t_front, 
                const double t_tail, 
                const double t_wd, 
                const double t_wb) {
    trailer_base2head_base = base_diff;
    head.SetParam(h_front, h_tail, h_wd, h_wb);
    trailer.SetParam(t_front, t_tail, t_wd, t_wb);
  }

  template <class T>
  T CalTrailerPose(const T& head_pose_now,
                   const T& head_pose_next,
                   const double trailer_yaw_now) {
    T trailer_pose;
    trailer_pose.theta = CalTrailerNextYaw<T>(
        head_pose_now, head_pose_next, trailer_yaw_now);
    trailer_pose.x = head_pose_next.x + trailer_base2head_base *
        std::cos(head_pose_next.theta);
    trailer_pose.y = head_pose_next.y + trailer_base2head_base *
        std::sin(head_pose_next.theta);
    return trailer_pose;
  }

  template <class T>
  T CalTrailerPose(const T& header_pose,
                   const double trailer_yaw) {
    T trailer_pose;
    trailer_pose.theta = trailer_yaw;
    trailer_pose.x = header_pose.x + trailer_base2head_base *
        std::cos(header_pose.theta);
    trailer_pose.y = header_pose.y + trailer_base2head_base *
        std::sin(header_pose.theta);
    return trailer_pose;
  }

 private:
  template <class T>
  double CalTrailerNextYaw(const T& head_pose_now,
                           const T& head_pose_next,
                           const double trailer_yaw_now) {
    // Bug from Dor. Chen
    // double L2 = trailer.wheelbase;
    // double d = trailer_base2head_base;
    // double trailer_new_yaw = trailer_yaw_now;
    // double x_now = head_pose_now.x;
    // double y_now = head_pose_now.y;
    // double yaw_now = head_pose_now.theta;
    // double x_next = head_pose_next.x;
    // double y_next = head_pose_next.y;
    // double yaw_next = head_pose_next.theta;
    // double delta_yaw = DifferentAngle(yaw_next, yaw_now);
    // double delta_length = Distance(x_now, y_now, x_next, y_next);
    // if (delta_yaw <= 0.01 || delta_yaw >= -0.01) {
    //   // 间隔yaw角太大，则需要分段进行计算, 假定车头按圆弧运动
    //   // arc_length: 圆弧总长度
    //   // step_arc_length: 每段圆弧长
    //   // min_num: 设定的最小分段数
    //   int min_num = 5;
    //   double arc_length = delta_length / 2.0 / std::sin(std::fabs(delta_yaw / 2.0)) * delta_yaw;
    //   double step_arc_length = 0.1;
    //   int step_num = std::floor(arc_length / step_arc_length) > min_num ?
    //                  std::floor(arc_length / step_arc_length) : min_num;
    //   step_arc_length = arc_length / step_num;
    //   double step_delta_yaw = delta_yaw / step_num;
    //   for (int i = 0; i < step_num; ++i) {
    //     double current_step_yaw = yaw_now + step_delta_yaw * i;
    //     double trailer_delta_yaw = step_arc_length / L2 * sin(current_step_yaw - trailer_new_yaw) -
    //         d * step_delta_yaw / L2 * cos(current_step_yaw - trailer_new_yaw);
    //     trailer_new_yaw += trailer_delta_yaw;
    //     trailer_new_yaw = NormalizeAngle(trailer_new_yaw);
    //   }
    // } else {
    //   double trailer_delta_yaw = delta_length / L2 * sin(yaw_now - trailer_new_yaw) -
    //       d * delta_yaw / L2 * cos(yaw_now - trailer_new_yaw);
    //   trailer_new_yaw += trailer_delta_yaw;
    //   trailer_new_yaw = NormalizeAngle(trailer_new_yaw);
    // }
    // return trailer_new_yaw;

    double trailer_yaw = trailer_yaw_now;
    double header_delta_yaw    = DifferentAngle(head_pose_now.theta, head_pose_next.theta);
    double header_delta_length = Distance(head_pose_now.x, head_pose_now.y,
                                          head_pose_next.x, head_pose_next.y);
    if (header_delta_yaw >= 0.01) {
        double header_radius = header_delta_length / 2.0 / std::sin(std::abs(header_delta_yaw / 2.0));
        double header_arc_length = header_radius * header_delta_yaw;
        double step_header_arc_length = 0.1;
        int step_num = static_cast<int>(std::ceil(header_arc_length / step_header_arc_length));
        step_header_arc_length = header_arc_length / step_num;
        double step_header_yaw = header_delta_yaw / step_num;
        for (int i = 0; i < step_num; ++i) {
            double cur_step_move_yaw = head_pose_now.theta + step_header_yaw * i;
            double step_header_move_vx = std::cos(cur_step_move_yaw);
            double step_header_move_vy = std::sin(cur_step_move_yaw);
            double trailer_vx          = std::sin(trailer_yaw);
            double trailer_vy          = -std::cos(trailer_yaw);
            double trailer_arc_length  = step_header_arc_length *
                                         std::abs(step_header_move_vx * trailer_vx + step_header_move_vy * trailer_vy) -
                                         trailer_base2head_base * step_header_yaw *
                                         std::abs(step_header_move_vy * trailer_vx - step_header_move_vx * trailer_vy);
            double trailer_delta_yaw = trailer_arc_length / trailer.wheelbase;
            if (DifferentAngle(cur_step_move_yaw, trailer_yaw) <= 0.0) {
                trailer_yaw += trailer_delta_yaw;
                trailer_yaw  = NormalizeAngle(trailer_yaw);
            } else {
                trailer_yaw -= trailer_delta_yaw;
                trailer_yaw  = NormalizeAngle(trailer_yaw);
            }
        }
    } else {
        double header_move_vx = std::cos(head_pose_now.theta);
        double header_move_vy = std::sin(head_pose_now.theta);
        double trailer_vx     = std::sin(trailer_yaw);
        double trailer_vy     = -std::cos(trailer_yaw);
        double trailer_arc_length = header_delta_length *
                                    std::abs(header_move_vx * trailer_vx + header_move_vy * trailer_vy) -
                                    trailer_base2head_base * header_delta_yaw *
                                    std::abs(header_move_vy * trailer_vx - header_move_vx * trailer_vy);
        double trailer_delta_yaw = trailer_arc_length / trailer.wheelbase;
        if (DifferentAngle(head_pose_now.theta, trailer_yaw) <= 0.0) {
            trailer_yaw += trailer_delta_yaw;
            trailer_yaw  = NormalizeAngle(trailer_yaw);
        } else {
            trailer_yaw -= trailer_delta_yaw;
            trailer_yaw  = NormalizeAngle(trailer_yaw);
        }
    }
    return trailer_yaw;
  }
};

} // namespace toolbox

#endif // GJK_INTERFACE_DATA_H_