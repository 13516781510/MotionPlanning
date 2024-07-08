//
// Created by 29451 on 2024/7/8.
//
#include <Eigen/Dense>
#include <cstdint>

using namespace Eigen;
int8_t P, M, target;
float a0[15] = {0.042, 0.1125, 0.175, 0.234, 0.2865, 0.3335,
				0.3765, 0.415, 0.45, 0.482, 0.5106, 0.5365, 0.56, 0.5812,
                0.6};
float alpha;
float u = 0;
float Y_now;
int32_t angle_now;
float t_now;

void Usr_Main() {
	//MPC
	//initial
	int8_t P = 15;
	int8_t M = 2;
	float alpha = 0.1;

	static MatrixXd Y_ref = MatrixXd::Zero(P, 1);
	static MatrixXd Y_cor = MatrixXd::Zero(P, 1);
	static MatrixXd Y0 = MatrixXd::Zero(P, 1);
	static MatrixXd A = MatrixXd::Zero(P, M);
	static MatrixXd du = MatrixXd::Zero(M, 1);
	static MatrixXd s = MatrixXd::Zero(P, P);
	static MatrixXd h = MatrixXd::Zero(P, 1);
	static MatrixXd Q = MatrixXd::Zero(P, P);
	static MatrixXd R = MatrixXd::Zero(M, M);
	static MatrixXd H = MatrixXd::Zero(M, M);

	//A Matrix
	for (int i = 0; i < P; i++) {
		A(i, 0) = a0[i];
	}

	for (int i = 1; i < P; i++) {
		for (int j = 1; j < M; j++) {
			if (i >= j)
				A(i, j) = A(i - 1, j - 1);
		}
	}
	//S Matrix
	for (int i = 0; i < (P - 1); i++) {
		s(i, i + 1) = 1;
	}
	s(P - 1, P - 1) = 1;
	//H Matrix
	for (int i = 0; i < P; i++) {
		h(i, 0) = 0.5;
	}
	//Q Matrix
	for (int i = 0; i < P; i++) {
		Q(i, i) = 1;
	}
	//R Matrix
	for (int i = 0; i < M; i++) {
		R(i, i) = 1;
	}

	Y_now = EulerAngle[0]->data;
	target = EulerAngle[1]->data;
	if (Y_now == 0);
	else {
		//Reference
		Y_ref(0, 0) = alpha * Y_now + (1 - alpha) * target;
		for (int i = 1; i < P; i++) {
			Y_ref(i, 0) = alpha * Y_ref(i - 1, 0) + (1 - alpha) * target;
		}
		//Correct the reference
		Y_cor = Y0 + h * (Y_now - Y0(0, 0));
		//Shift
		Y0 = s * Y_cor;
		//Predict
		Y0 = A * du + Y0;
		//Optimization
		H = A.transpose() * Q * A + R;
		du = H.inverse() * A.transpose() * Q * (Y_ref - Y0);

		u += du(0, 0);
	}
}

