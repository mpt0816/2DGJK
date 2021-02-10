#ifndef GJK_COMMON_UTIL_H_
#define GJK_COMMON_UTIL_H_

namespace gjk {

// macro that implements the CompareSign function
#define SAMESIGN(m, n) ((m > 0.) == (n > 0.)) 

// macro that implements squared vector norm 
#define NORM2(m) (m[0] * m[0] + m[1] * m[1])

// macro that implements dot product 
#define DOTPRODUCT(m, n) (m[0] * n[0] + m[1] * n[1])

// macro that implements cross product
#define CROSSPRODUCT(m, n) (m[0] * n[1] - m[1] * n[0])

// macro that assign m == n
#define ASSIGN(m, n) m[0] = n[0];\
                     m[1] = n[1]

} // namespace gjk

#endif // GJK_COMMON_UTIL_H_