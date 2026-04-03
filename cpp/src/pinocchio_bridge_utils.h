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

#define CHECK_LEN(el, len, expected)                                           \
  __extension__({                                                              \
    if (!check_length_eq(#el, (len), (expected)))                              \
      return false;                                                            \
  })

#define MAP_TY(ty, el, len, expected_length)                                   \
  __extension__({                                                              \
    CHECK_LEN(el, len, expected_length);                                       \
    Eigen::Map<ty>((el).data(), (len));                                        \
  })

#define MAP(slice, expected_length)                                            \
  MAP_TY(const Eigen::VectorXd, slice, slice.length(), expected_length)

#define MAP_MUT(slice, expected_length)                                        \
  MAP_TY(Eigen::VectorXd, slice, slice.length(), expected_length)

#define MAP_MAT_MUT(mat, expected_length)                                      \
  MAP_TY(Eigen::VectorXd, mat, mat.rows() * mat.cols(), expected_length)

#define INDEX_CHECK_LEN(idx, len)                                              \
  __extension__({                                                              \
    if ((idx) >= (len)) {                                                      \
      std::cerr << #idx << " " << (idx) << " >= " << len << std::endl;         \
      return false;                                                            \
    }                                                                          \
  })
