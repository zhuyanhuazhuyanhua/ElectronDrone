#include <Eigen/Eigen>
#include <iostream>

class uwb_solver {
public:
  /**
   * @brief 构造函数
   *
   * @param anchors 4个基站坐标[x, y, z]
   * @note 4个基站坐标必须是非共面的
   */
  uwb_solver(const std::vector<Eigen::Vector3d> &anchors,
             const double &coplanar_threshold = 1e-6) {
    if (anchors.size() != 4) {
      std::cout << "Error: The number of anchors must be 4." << std::endl;
      throw std::invalid_argument("Invalid number of anchors");
    }

    // 检查基站坐标是否在同一平面上
    Eigen::Vector3d v1 = anchors[1] - anchors[0];
    Eigen::Vector3d v2 = anchors[2] - anchors[0];
    Eigen::Vector3d v3 = anchors[3] - anchors[0];
    Eigen::Vector3d normal = v1.cross(v2);
    if (normal.dot(v3) == 0) {
      std::cout << "Error: The anchors are coplanar." << std::endl;
      throw std::invalid_argument("Anchors are coplanar");
    }

    Eigen::Matrix3d A;
    A << anchors[1][0] - anchors[0][0], anchors[1][1] - anchors[0][1],
        anchors[1][2] - anchors[0][2], anchors[2][0] - anchors[0][0],
        anchors[2][1] - anchors[0][1], anchors[2][2] - anchors[0][2],
        anchors[3][0] - anchors[0][0], anchors[3][1] - anchors[0][1],
        anchors[3][2] - anchors[0][2];

    // 验证行列式是否可逆
    double det = A.determinant();
    if (std::abs(det) < 1e-8) {
      std::cerr << "Error: Matrix A is near-singular or singular. Cannot "
                   "compute inverse."
                << std::endl;
      throw std::runtime_error("Matrix A is not invertible");
    }
    Eigen::Matrix3d A_inverse = A.inverse();

    // 计算b_tmp
    b_tmp_ << (anchors[1][0] * anchors[1][0] - anchors[0][0] * anchors[0][0] +
               anchors[1][1] * anchors[1][1] - anchors[0][1] * anchors[0][1] +
               anchors[1][2] * anchors[1][2] - anchors[0][2] * anchors[0][2]) /
                  2.0,
        (anchors[2][0] * anchors[2][0] - anchors[0][0] * anchors[0][0] +
         anchors[2][1] * anchors[2][1] - anchors[0][1] * anchors[0][1] +
         anchors[2][2] * anchors[2][2] - anchors[0][2] * anchors[0][2]) /
            2.0,
        (anchors[3][0] * anchors[3][0] - anchors[0][0] * anchors[0][0] +
         anchors[3][1] * anchors[3][1] - anchors[0][1] * anchors[0][1] +
         anchors[3][2] * anchors[3][2] - anchors[0][2] * anchors[0][2]) /
            2.0;
    std::cout << "uwb_solver init success" << std::endl;
  }

  /**
   * @brief 求解位置
   *
   * @param distances 距离1-4号基站的位置
   * @return Eigen::Vector3d [x, y, z] 位置
   */
  Eigen::Vector3d solve(const Eigen::Vector4d &distances) {
    // 计算b
    Eigen::Vector3d b = b_tmp_;
    b(0) -= (distances[1] * distances[1] - distances[0] * distances[0]) / 2.0;
    b(1) -= (distances[2] * distances[2] - distances[0] * distances[0]) / 2.0;
    b(2) -= (distances[3] * distances[3] - distances[0] * distances[0]) / 2.0;

    // 计算位置
    Eigen::Vector3d position = b.transpose() * A_inverse_;
    return position;
  }

private:
  // 基站坐标
  Eigen::Matrix3d A_inverse_;
  // 暂存b的常量
  Eigen::Vector3d b_tmp_;
};
