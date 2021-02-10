#include "../core/gjk.h"
#include <iostream>


int main(int argc, char* argv[]) {

  gjk::Polygon p, q;
  p.vertices = new(double* [4]);
  q.vertices = new(double* [4]);
  p.num_vertex = 4;
  q.num_vertex = 4;
  for (int i = 0; i < 4; ++i) {
    p.vertices[i] = new(double [2]);
    q.vertices[i] = new(double [2]);
  }

  p.vertices[0][0] = 0.0; p.vertices[0][1] = 1.0;
  p.vertices[1][0] = 0.0; p.vertices[1][1] = 0.0;
  p.vertices[2][0] = 1.0; p.vertices[2][1] = 0.0;
  p.vertices[3][0] = 1.0; p.vertices[3][1] = 1.0;

  q.vertices[0][0] = 3.0; q.vertices[0][1] = 1.0;
  q.vertices[1][0] = 3.0; q.vertices[1][1] = 0.0;
  q.vertices[2][0] = 4.0; q.vertices[2][1] = 0.0;
  q.vertices[3][0] = 4.0; q.vertices[3][1] = 1.0;
  double pt_p[2], pt_q[2];

  bool level_1 = gjk::Gjk::GjkLevel1(p, q);
  double level_2 = gjk::Gjk::GjkLevel2(p, q);
  double level_3 = gjk::Gjk::GjkLevel3(p, q, pt_p, pt_q);

  std::cout << "Level 1 : overlap : " << (level_1 ? "true" : "false") << std::endl;
  std::cout << "Level 2 : gjk distance between polygons = " << level_2 << std::endl;
  std::cout << "Level 3 : gjk distance between polygons = " << level_3 << std::endl;
  std::cout << "Level 3 : witness point, polygon P is (" << pt_p[0] << ", " << pt_p[1] << ")" << std::endl;
  std::cout << "Level 3 : witness point, polygon Q is (" << pt_q[0] << ", " << pt_q[1]<< ")" << std::endl;

  for (int i = 0; i < 4; ++i) {
    delete p.vertices[i];
    delete q.vertices[i];
  }
  delete p.vertices;
  delete q.vertices;
}