# ifndef GJK_COMMON_CONSTANT_H_
# define GJK_COMMON_CONSTANT_H_

namespace gjk {
namespace constant {

// Tolerance on relative distance
static const double kEpsRel = 1e-5;

// Tolerance on absolute distance
static const double kEpsAbs = 1e-5;

// Maximum number of iterations of the GJK algorithm
static const int kMaxInter = 50;

} // namespace constant
} // namespace gjk

# endif // GJK_COMMON_CONSTANT_H_