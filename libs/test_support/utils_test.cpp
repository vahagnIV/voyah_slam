#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include "utils_test.h"

namespace voyahslam::test {

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> CreateRandomMatrix(unsigned int a_rows, unsigned int a_cols) {
	assert(a_rows > 0 && a_cols > 0);
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<T> dis(0, 1);
	return Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(a_rows, a_cols).unaryExpr([&](float dummy) {
		return dis(gen);
	});
}
template
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> CreateRandomMatrix<float>(unsigned int, unsigned int);

template
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> CreateRandomMatrix<double>(unsigned int , unsigned int);

template<>
Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> CreateRandomMatrix<uint8_t>(unsigned int a_rows, unsigned int a_cols) {
	assert(a_rows > 0 && a_cols > 0);
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dis(0, 255);
	return Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(a_rows, a_cols).unaryExpr([&](float dummy) {
		return static_cast<uint8_t>(dis(gen));
	});
}

geometry::TPoints2DF CreateRandomKeyPoints(int a_n, unsigned int a_width, unsigned int a_height)
{
	assert(a_n > 0 && a_width > 0 && a_height > 0);
	auto kps = CreateRandomMatrix(2, a_n);
	kps.row(0).array() *= a_width;
	kps.row(1).array() *= a_height;
	return kps;
}

geometry::TMatrix33F CreateRotationMatrix(precision_t alpha, precision_t beta, precision_t gamma) {
	geometry::TMatrix33F rotation_z, rotation_y, rotation_x;

	rotation_z << std::cos(alpha), -std::sin(alpha), 0, std::sin(alpha),
		std::cos(alpha), 0, 0, 0, 1;
	rotation_y << std::cos(beta), 0, -std::sin(beta), 0, 1, 0, std::sin(beta),
		0, std::cos(beta);
	rotation_x << 1, 0, 0, 0, std::cos(gamma), -std::sin(gamma), 0,
		std::sin(gamma), std::cos(gamma);

	return rotation_z * rotation_y * rotation_x;
}

geometry::TMatrix33F CreateRotationMatrixD(precision_t a_alpha, precision_t a_beta, precision_t a_gamma) {
	precision_t alpha = a_alpha * M_PI / 180;
	precision_t beta = a_beta * M_PI / 180;
	precision_t gamma = a_gamma * M_PI / 180;
	return CreateRotationMatrix(alpha, beta, gamma);
}

unsigned int GetMatrixRank(const Eigen::MatrixXd &a_mat) {
	Eigen::BDCSVD<Eigen::MatrixXd>
		svd(a_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	return svd.rank();
}

geometry::TMatrix33F CrossProductMatrix(const geometry::TRowVectorF& a_v) {
	geometry::TMatrix33F matrix;
	matrix << 0, -a_v(2), a_v(1), a_v(2), 0, -a_v(0), -a_v(1), a_v(0), 0;
	return matrix;
}
} // namespace voyahslam::test
