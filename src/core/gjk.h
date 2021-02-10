#ifndef GJK_CORE_GJK_H_
#define GJK_CORE_GJK_H_

#include "../common/data.h"
#include "../common/util.h"
#include "../common/constant.h"
#include "stdint.h"
#include <iostream>
#include <cmath>

namespace gjk {

class Gjk {
 public:
  // Level 1: binary result, collide or not
  static bool GjkLevel1(const Polygon& polygon_p, const Polygon& polygon_q) {
    int i = 0;                       // counter, avoid constructe and destructe
    double tesnorm = 0.;             // norm2 of simplex's vertices, avoid constructe and destructe
    double norm2Wmax = 0.;
    int iteration = 0;               // iteration counter
    double direction[2];             // search direction
    double negetive_direction[2];    // search direction * -1
    double s_p[2], s_q[2];           // sopport mapping computed last
    double w[2];                     // vertex on CSO boundary by the difference of support functions on both polygon
    
    double eps_rel2 = constant::kEpsRel * constant::kEpsRel;
    double eps_abs2 = constant::kEpsAbs * constant::kEpsAbs;

    // Initialise search direction and simplex
    Simplex simplex;
    simplex.num_vertex = 1;
    for (i = 0; i < 2; ++i) {
      direction[i] = polygon_p.vertices[0][i] - polygon_q.vertices[0][i];
      // polygon_p.s[i] = polygon_p.vertices[0][i];
      // polygon_q.s[i] = polygon_q.vertices[0][i];
      s_p[i] = polygon_p.vertices[0][i];
      s_q[i] = polygon_q.vertices[0][i];

      simplex.vertices[0][i] = direction[i];
      simplex.pts_p[0][i] = s_p[i];
      simplex.pts_q[0][i] = s_q[i];
    }

    if (NORM2(direction) <= eps_abs2) {
      return true;
    }

    // begin GJK interation
    do {
      iteration++;

      // update negetive search direction
      negetive_direction[0] = -direction[0];
      negetive_direction[1] = -direction[1];

      // support function
      Support(polygon_p, negetive_direction, s_p);
      Support(polygon_q, direction, s_q);
      // CSO point
      w[0] = s_p[0] - s_q[0];
      w[1] = s_p[1] - s_q[1];

      // 1st exit condition
      if (DOTPRODUCT(direction, w) > 0.0) {
        return false;
      } else {
        if (simplex.num_vertex == 2) {
          if (CROSSPRODUCT(simplex.vertices[0], w) * CROSSPRODUCT(simplex.vertices[1], w) <= 0.0) {
            return true;
          }
        }
      }

      // 2nd exit condition
      if (NORM2(direction) < eps_rel2) {
        break;
      }

      // add new vertex to simplex
      i = simplex.num_vertex;
      ASSIGN(simplex.vertices[i], w);
      // simplex.vertices[i][0] = w[0];
      // simplex.vertices[i][1] = w[1];
      simplex.num_vertex++;

      // record point of CSO from polygon P and Q
      for (i = 0; i < 2; ++i) {
        simplex.pts_p[simplex.num_vertex - 1][i] = s_p[i];
        simplex.pts_q[simplex.num_vertex - 1][i] = s_q[i];
      }

      // invoke distance sub-algorithm
      SubAlgorithm(simplex, direction);

      // 3rd exit condition
      for (i = 0; i < simplex.num_vertex; ++i) {
        tesnorm = NORM2(simplex.vertices[i]);
        if (tesnorm > norm2Wmax) {
          norm2Wmax = tesnorm;
        }
      }
      if (NORM2(direction) <= eps_abs2 * norm2Wmax) {
        break;
      }

      // 4th and 5th exit condition
    } while (simplex.num_vertex != 3 && iteration != constant::kMaxInter);

    // compute and return distance
    return NORM2(direction) < eps_abs2 ? true : false;
  }

  // Level 2: distance scalar
  static double GjkLevel2(const Polygon& polygon_p, const Polygon& polygon_q) {

    int i = 0;                       // counter, avoid constructe and destructe
    double tesnorm = 0.;             // norm2 of simplex's vertices, avoid constructe and destructe
    double norm2Wmax = 0.;
    int iteration = 0;               // iteration counter
    double direction[2];             // search direction
    double negetive_direction[2];    // search direction * -1
    double s_p[2], s_q[2];           // sopport mapping computed last
    double w[2];                     // vertex on CSO boundary by the difference of support functions on both polygon
    
    double eps_rel2 = constant::kEpsRel * constant::kEpsRel;
    double eps_abs2 = constant::kEpsAbs * constant::kEpsAbs;

    // Initialise search direction and simplex
    Simplex simplex;
    simplex.num_vertex = 1;
    for (i = 0; i < 2; ++i) {
      direction[i] = polygon_p.vertices[0][i] - polygon_q.vertices[0][i];
      s_p[i] = polygon_p.vertices[0][i];
      s_q[i] = polygon_q.vertices[0][i];

      simplex.vertices[0][i] = direction[i];
      simplex.pts_p[0][i] = s_p[i];
      simplex.pts_q[0][i] = s_q[i];
    }

    if (NORM2(direction) <= eps_abs2) {
      return 0.0;
    }

    // begin GJK interation
    do {
      iteration++;

      // update negetive search direction
      negetive_direction[0] = -direction[0];
      negetive_direction[1] = -direction[1];

      // support function
      Support(polygon_p, negetive_direction, s_p);
      Support(polygon_q, direction, s_q);
      // CSO point
      w[0] = s_p[0] - s_q[0];
      w[1] = s_p[1] - s_q[1];

      // 1st exit condition
      if (NORM2(direction) - DOTPRODUCT(direction, w) <= eps_rel2 * NORM2(direction)) {
        break;
      }

      // 2nd exit condition
      if (NORM2(direction) < eps_rel2) {
        break;
      }

      // add new vertex to simplex
      i = simplex.num_vertex;
      ASSIGN(simplex.vertices[i], w);
      // simplex.vertices[i][0] = w[0];
      // simplex.vertices[i][1] = w[1];
      simplex.num_vertex++;

      // record point of CSO from polygon P and Q
      for (i = 0; i < 2; ++i) {
        simplex.pts_p[simplex.num_vertex - 1][i] = s_p[i];
        simplex.pts_q[simplex.num_vertex - 1][i] = s_q[i];
      }

      // invoke distance sub-algorithm
      SubAlgorithm(simplex, direction);

      // 3rd exit condition
      for (i = 0; i < simplex.num_vertex; ++i) {
        tesnorm = NORM2(simplex.vertices[i]);
        if (tesnorm > norm2Wmax) {
          norm2Wmax = tesnorm;
        }
      }
      if (NORM2(direction) <= eps_abs2 * norm2Wmax) {
        break;
      }

      // 4th and 5th exit condition
    } while (simplex.num_vertex != 3 && iteration != constant::kMaxInter);

    // compute and return distance
    return std::sqrt(NORM2(direction));
  }

  // Level 3: distance, complete witness points
  static double GjkLevel3(const Polygon& polygon_p,
                          const Polygon& polygon_q,
                          double* coord_p,
                          double* coord_q) {
    int i = 0;                       // counter, avoid constructe and destructe
    double tesnorm = 0.;             // norm2 of simplex's vertices, avoid constructe and destructe
    double norm2Wmax = 0.;
    int iteration = 0;               // iteration counter
    double direction[2];             // search direction
    double negetive_direction[2];    // search direction * -1
    double s_p[2], s_q[2];           // sopport mapping computed last
    double w[2];                     // vertex on CSO boundary by the difference of support functions on both polygon

    double eps_rel2 = constant::kEpsRel * constant::kEpsRel;
    double eps_abs2 = constant::kEpsAbs * constant::kEpsAbs;

    // Initialise search direction and simplex
    Simplex simplex;
    simplex.num_vertex = 1;
    for (i = 0; i < 2; ++i) {
      direction[i] = polygon_p.vertices[0][i] - polygon_q.vertices[0][i];
      s_p[i] = polygon_p.vertices[0][i];
      s_q[i] = polygon_q.vertices[0][i];

      simplex.vertices[0][i] = direction[i];
      simplex.pts_p[0][i] = s_p[i];
      simplex.pts_q[0][i] = s_q[i];
    }

    if (NORM2(direction) <= eps_abs2) {
      return 0.0;
    }

    // begin GJK interation
    do {
      iteration++;

      // update negetive search direction
      negetive_direction[0] = -direction[0];
      negetive_direction[1] = -direction[1];

      // support function
      Support(polygon_p, negetive_direction, s_p);
      Support(polygon_q, direction, s_q);
      // CSO point
      w[0] = s_p[0] - s_q[0];
      w[1] = s_p[1] - s_q[1];

      // 1st exit condition
      if (NORM2(direction) - DOTPRODUCT(direction, w) <= eps_rel2 * NORM2(direction)) {
        break;
      }

      // 2nd exit condition
      if (NORM2(direction) < eps_rel2) {
        break;
      }

      // add new vertex to simplex
      i = simplex.num_vertex;
      ASSIGN(simplex.vertices[i], w);
      // simplex.vertices[i][0] = w[0];
      // simplex.vertices[i][1] = w[1];
      simplex.num_vertex++;

      // record point of CSO from polygon P and Q
      for (i = 0; i < 2; ++i) {
        simplex.pts_p[simplex.num_vertex - 1][i] = s_p[i];
        simplex.pts_q[simplex.num_vertex - 1][i] = s_q[i];
      }

      // invoke distance sub-algorithm
      SubAlgorithm(simplex, direction);

      // 3rd exit condition
      for (i = 0; i < simplex.num_vertex; ++i) {
        tesnorm = NORM2(simplex.vertices[i]);
        if (tesnorm > norm2Wmax) {
          norm2Wmax = tesnorm;
        }
      }
      if (NORM2(direction) <= eps_abs2 * norm2Wmax) {
        break;
      }

      // 4th and 5th exit condition
    } while (simplex.num_vertex != 3 && iteration != constant::kMaxInter);

    for (i = 0; i < simplex.num_vertex; ++i) {
      coord_p[0] += simplex.lambdas[i] * simplex.pts_p[i][0];
      coord_p[1] += simplex.lambdas[i] * simplex.pts_p[i][1];
      coord_q[0] += simplex.lambdas[i] * simplex.pts_q[i][0];
      coord_q[1] += simplex.lambdas[i] * simplex.pts_q[i][1];
    }

    // compute and return distance
    return std::sqrt(NORM2(direction));
  }

 private:
  // maybe use hill-climbing algorithm
  inline static void Support(const Polygon& polygon, const double* direction, double* s) {
    int index = -1;
    double max_projection = DOTPRODUCT(s, direction);
    for (int i = 1; i < polygon.num_vertex; ++i) {
      double projection = DOTPRODUCT(polygon.vertices[i], direction);
      if (projection > max_projection) {
        max_projection = projection;
        index = i;
      }
    }
    if (index != -1) {
      ASSIGN(s, polygon.vertices[index]);
      // polygon.s[0] = polygon.vertices[index][0];
      // polygon.s[1] = polygon.vertices[index][1];
    }
  }

  inline static void RemovePointFrom3P(Simplex& simplex, int index) {
    switch (index) {
      case 0:
        ASSIGN(simplex.vertices[0], simplex.vertices[1]);
        ASSIGN(simplex.vertices[1], simplex.vertices[2]);
        ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
        ASSIGN(simplex.pts_p[1], simplex.pts_p[2]);
        ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
        ASSIGN(simplex.pts_q[1], simplex.pts_q[2]);
        // for (int i = 0; i < 2; ++i) {
        //   simplex.vertices[0][i] = simplex.vertices[1][i];
        //   simplex.vertices[1][i] = simplex.vertices[2][i];
        //   simplex.pts_p[0][i] = simplex.pts_p[1][i];
        //   simplex.pts_p[1][i] = simplex.pts_p[2][i];
        //   simplex.pts_q[0][i] = simplex.pts_q[1][i];
        //   simplex.pts_q[1][i] = simplex.pts_q[2][i];
        // }
        simplex.num_vertex--;
        break;
      case 1:
        ASSIGN(simplex.vertices[1], simplex.vertices[2]);
        ASSIGN(simplex.pts_p[1], simplex.pts_p[2]);
        ASSIGN(simplex.pts_q[1], simplex.pts_q[2]);
        // for (int i = 0; i < 2; ++i) {
        //   simplex.vertices[1][i] = simplex.vertices[2][i];
        //   simplex.pts_p[1][i] = simplex.pts_p[2][i];
        //   simplex.pts_q[1][i] = simplex.pts_q[2][i];
        // }
        simplex.num_vertex--;
        break;
      case 2:
        simplex.num_vertex--;
        break;
      default:
        break;
    }
  }

  inline static void RemovePointFrom2P(Simplex& simplex, int index) {
    switch (index) {
      case 0:
        ASSIGN(simplex.vertices[0], simplex.vertices[1]);
        ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
        ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
        // for (int i = 0; i < 2; ++i) {
        //   simplex.vertices[0][i] = simplex.vertices[1][i];
        //   simplex.pts_p[0][i] = simplex.pts_p[1][i];
        //   simplex.pts_q[0][i] = simplex.pts_q[1][i];
        // }
        simplex.num_vertex--;
        break;
      case 1:
        simplex.num_vertex--;
        break;
      default:
        break;
    }
  }

  inline static void ClaculateLambda(Simplex& simplex, double u, double v, double sum) {
    simplex.lambdas[0] = u / sum;
    simplex.lambdas[1] = v / sum;
    simplex.lambdas[2] = 1 - simplex.lambdas[0] - simplex.lambdas[1];
  }

  inline static void ClaculateDirection(Simplex& simplex, double* direction) {
    direction[0] = simplex.lambdas[0] * simplex.vertices[0][0] + 
                   simplex.lambdas[1] * simplex.vertices[1][0] +
                   simplex.lambdas[2] * simplex.vertices[2][0];
    direction[1] = simplex.lambdas[0] * simplex.vertices[0][1] + 
                   simplex.lambdas[1] * simplex.vertices[1][1] +
                   simplex.lambdas[2] * simplex.vertices[2][1];
  }

  inline static void S1D(Simplex& simplex, double* direction) {
    // assume 1st point is A
    //        2nd point is B
    double ab[2]{simplex.vertices[1][0] - simplex.vertices[0][0],
                 simplex.vertices[1][1] - simplex.vertices[0][1]};
    double con1 = DOTPRODUCT(simplex.vertices[0], ab);
    double con2 = DOTPRODUCT(simplex.vertices[1], ab);
    if (con1 > 0. || con1 == 0.) {
      simplex.num_vertex = 1;
      simplex.lambdas[0] = 1.0;
      ASSIGN(direction, simplex.vertices[0]);
      // direction[0] = simplex.vertices[0][0]; 
      // direction[1] = simplex.vertices[0][1];
      return;
    }
    if (con2 < 0. || con2 == 0.) {
      simplex.num_vertex = 1;
      simplex.lambdas[0] = 1.0;
      ASSIGN(simplex.vertices[0], simplex.vertices[1]);
      ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
      ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
      ASSIGN(direction, simplex.vertices[1]);
      // simplex.vertices[0][0] = simplex.vertices[1][0];
      // simplex.vertices[0][1] = simplex.vertices[1][1];
      // simplex.pts_p[0][0] = simplex.pts_p[1][0];
      // simplex.pts_p[0][1] = simplex.pts_p[1][1];
      // simplex.pts_q[0][0] = simplex.pts_q[1][0];
      // simplex.pts_q[0][1] = simplex.pts_q[1][1];
      // direction[0] = simplex.vertices[1][0]; 
      // direction[1] = simplex.vertices[1][1];
      return;
    }
    simplex.lambdas[0] = - con2 / (con1 - con2);
    simplex.lambdas[1] = 1- simplex.lambdas[0];
    direction[0] = simplex.lambdas[0] * simplex.vertices[0][0] +
                   simplex.lambdas[1] * simplex.vertices[1][0];
    direction[1] = simplex.lambdas[0] * simplex.vertices[0][1] +
                   simplex.lambdas[1] * simplex.vertices[1][1];
  }

  inline static void ConeRegion0(Simplex& simplex, double* direction, int index) {
    double mv[2], nv[2];
    for (int i = 0; i < 2; ++i) {
      mv[i] = simplex.vertices[0][i] - simplex.vertices[1][i];
      nv[i] = simplex.vertices[0][i] - simplex.vertices[2][i];
    }
    if (DOTPRODUCT(mv, nv) < 0.) {
      if (DOTPRODUCT(simplex.vertices[0], mv) > 0.) {
        RemovePointFrom3P(simplex, 2);
        S1D(simplex, direction);
        return;
      }
      if (DOTPRODUCT(simplex.vertices[0], nv) > 0.) {
        RemovePointFrom3P(simplex, 1);
        S1D(simplex, direction);
        return;
      }
    } else {
      simplex.num_vertex = 1;
      simplex.lambdas[0] = 1.0;
      direction = simplex.vertices[0];
    }
  }

  inline static void ConeRegion1(Simplex& simplex, double* direction, int index) {
    double mv[2], nv[2];
    for (int i = 0; i < 2; ++i) {
      mv[i] = simplex.vertices[1][i] - simplex.vertices[0][i];
      nv[i] = simplex.vertices[1][i] - simplex.vertices[2][i];
    }
    if (DOTPRODUCT(mv, nv) < 0.) {
      if (DOTPRODUCT(simplex.vertices[1], mv) > 0.) {
        RemovePointFrom3P(simplex, 2);
        S1D(simplex, direction);
        return;
      }
      if (DOTPRODUCT(simplex.vertices[1], nv) > 0.) {
        RemovePointFrom3P(simplex, 0);
        S1D(simplex, direction);
        return;
      }
    } else {
      simplex.num_vertex = 1;
      simplex.lambdas[0] = 1.0;
      ASSIGN(simplex.vertices[0], simplex.vertices[1]);
      ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
      ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
      ASSIGN(direction, simplex.vertices[0]);
      // for (int i = 0; i < 2; ++i) {
      //   simplex.vertices[0][i] = simplex.vertices[1][i];
      //   simplex.pts_p[0][i] = simplex.pts_p[1][i];
      //   simplex.pts_q[0][i] = simplex.pts_q[1][i];
      // }
      // direction = simplex.vertices[0];
    }
  }

  inline static void ConeRegion2(Simplex& simplex, double* direction, int index) {
    double mv[2], nv[2];
    for (int i = 0; i < 2; ++i) {
      mv[i] = simplex.vertices[2][i] - simplex.vertices[0][i];
      nv[i] = simplex.vertices[2][i] - simplex.vertices[1][i];
    }
    if (DOTPRODUCT(mv, nv) < 0.) {
      if (DOTPRODUCT(simplex.vertices[2], mv) > 0.) {
        RemovePointFrom3P(simplex, 1);
        S1D(simplex, direction);
        return;
      }
      if (DOTPRODUCT(simplex.vertices[2], nv) > 0.) {
        RemovePointFrom3P(simplex, 0);
        S1D(simplex, direction);
        return;
      }
    } else {
      simplex.num_vertex = 1;
      simplex.lambdas[0] = 1.0;
      ASSIGN(simplex.vertices[0], simplex.vertices[2]);
      ASSIGN(simplex.pts_p[0], simplex.pts_p[2]);
      ASSIGN(simplex.pts_q[0], simplex.pts_q[2]);
      ASSIGN(direction, simplex.vertices[0]);
      // for (int i = 0; i < 2; ++i) {
      //   simplex.vertices[0][i] = simplex.vertices[2][i];
      //   simplex.pts_p[0][i] = simplex.pts_p[2][i];
      //   simplex.pts_q[0][i] = simplex.pts_q[2][i];
      // }
      // direction = simplex.vertices[0];
    }
  }

  inline static void S2D(Simplex& simplex, double* direction) {
    // assume 1st point is A
    //        2nd point is B
    //        3rd point is C
    double u = CROSSPRODUCT(simplex.vertices[1], simplex.vertices[2]);
    double v = CROSSPRODUCT(simplex.vertices[2], simplex.vertices[0]);
    double w = CROSSPRODUCT(simplex.vertices[0], simplex.vertices[1]);
    double sum = u + v + w;
    int barycode = 
        (SAMESIGN(sum, w)) ^ (SAMESIGN(sum, v) << 1) ^ (SAMESIGN(sum, u) << 2);
    switch (barycode) {
      case 1: 
        ConeRegion2(simplex, direction, 2);
        break;
      case 2:
        ConeRegion1(simplex, direction, 1);
        break;
      case 3:
        RemovePointFrom3P(simplex, 0);
        S1D(simplex, direction);
        break;
      case 4:
        ConeRegion0(simplex, direction, 0);
        break;
      case 5:
        RemovePointFrom3P(simplex, 1);
        S1D(simplex, direction);
        break;
      case 6:
        RemovePointFrom3P(simplex, 2);
        S1D(simplex, direction);
        break;
      case 7:
        ClaculateLambda(simplex, u, v, sum);
        ClaculateDirection(simplex, direction);
        break;
      default:
        break;
    }  
  }

  // barycode-based distance sub-algorithm
  inline static void SubAlgorithm(Simplex& s, double* dir) {
    switch (s.num_vertex) {
      case 3:
        S2D(s, dir);
        break;
      case 2:
        S1D(s, dir);
        break;
      case 1:
        s.lambdas[0] = 1.0;
        s.lambdas[1] = 0.0;
        s.lambdas[2] = 0.0;
        break;
      default:
        break;
    }
  }

};

} // namespace gjk

#endif // GJK_CORE_GJK_H_