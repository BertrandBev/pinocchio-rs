#include <Eigen/Dense>
#include <iostream>

inline bool check_length_eq(const char *name, size_t length, size_t expected) {
  if (length != expected) {
    std::cerr << name << " length: " << length << " expected: " << expected
              << std::endl;
    return false;
  }
  return true;
}

#define MAP_TY(ty, slice, expected_length)                                     \
  __extension__({                                                              \
    if (!check_length_eq(#slice, (slice).length(), (expected_length)))         \
      return false;                                                            \
    Eigen::Map<ty>((slice).data(),                                             \
                   static_cast<Eigen::Index>((slice).length()));               \
  })

#define MAP(slice, expected_length)                                            \
  MAP_TY(const Eigen::VectorXd, slice, expected_length)

#define MAP_MUT(slice, expected_length)                                        \
  MAP_TY(Eigen::VectorXd, slice, expected_length)
