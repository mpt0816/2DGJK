#ifndef GJK_COMMON_DATA_H_
#define GJK_COMMON_DATA_H_

namespace gjk {

// structure for a polygon
struct Polygon {
  int num_vertex;                   // number of polygon's vertices
  double **vertices;                // pointer to pointer to the vertices's coordinate
  // double s[2];                      // sopport mapping computed last
};

// structure for a simplex
struct Simplex {
  int num_vertex;                   // number of simplex's vertices
  double vertices[3][2];            // simplex's vertices
  double lambdas[3];                // barycentric coordinates for each vertex
  double pts_p[3][2];               // points of polygon P that form the simplex
  double pts_q[3][2];               // points of polygon Q that form the simplex
};

} // namespace gjk

#endif // GJK_COMMON_DATA_H_