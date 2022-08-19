#pragma once

#include "../header/MathTool.h"

using namespace GMT;


double MathTool::degree2Radian(double degree)
{
	return degree * MathPI / 180.f;
}

double MathTool::radian2Degree(double radian)
{
	return radian * 180.f / MathPI;
}

Mat MathTool::ZYZ2SO3(Vector3d P)
{
	double alpha = P(0);
	double beta = P(1);
	double gamma = P(2);


	//%--------Z(alpha)-Y(beta)-Z(gamma)-------------------------
	//r11 = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
	//r21 = cos(alpha)*sin(gamma) + cos(beta)*cos(gamma)*sin(alpha);
	//r31 = -cos(gamma)*sin(beta);
	//r12 = -cos(gamma)*sin(alpha) - cos(alpha)*cos(beta)*sin(gamma);
	//r22 = cos(alpha)*cos(gamma) - cos(beta)*sin(alpha)*sin(gamma);
	//r32 = sin(beta)*sin(gamma);
	//r13 = cos(alpha)*sin(beta);
	//r23 = sin(alpha)*sin(beta);
	//r33 = cos(beta);


	double r11 = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
	double r21 = cos(alpha)*sin(gamma) + cos(beta)*cos(gamma)*sin(alpha);
	double r31 = -cos(gamma)*sin(beta);
	double r12 = -cos(gamma)*sin(alpha) - cos(alpha)*cos(beta)*sin(gamma);
	double r22 = cos(alpha)*cos(gamma) - cos(beta)*sin(alpha)*sin(gamma);
	double r32 = sin(beta)*sin(gamma);
	double r13 = cos(alpha)*sin(beta);
	double r23 = sin(alpha)*sin(beta);
	double r33 = cos(beta);

	//%----------------------------------------------------------
	//	R = [r11 r12 r13; r21 r22 r23; r31 r32 r33];

	Mat R(3, 3);
	R << r11, r12, r13,
		r21, r22, r23,
		r31, r32, r33;
	//R << r11, r21, r31,
	//	r12, r22, r32,
	//	r13, r23, r33;


	return R;
}

Mat MathTool::poseZYZ2SO3(VectorXd Pose)
{
	return ZYZ2SO3(Pose.segment(3, 3));
}

//只能處理1*6的矩陣
VectorXd MathTool::degree2Radian6(VectorXd Pose)
{
	Pose(3) = degree2Radian(Pose(3));
	Pose(4) = degree2Radian(Pose(4));
	Pose(5) = degree2Radian(Pose(5));

	return Pose;
}

VectorXd MathTool::radian2Degree12(VectorXd Pose)
{
	Pose(6) = radian2Degree(Pose(6));
	Pose(7) = radian2Degree(Pose(7));
	Pose(8) = radian2Degree(Pose(8));
	Pose(9) = radian2Degree(Pose(9));
	Pose(10) = radian2Degree(Pose(10));
	Pose(11) = radian2Degree(Pose(11));

	return Pose;
}

//vector<double> MathTool::randomFactory(double min, double max, int how_many)
//{
//	/* 隨機設備 */
//	std::random_device rd;
//
//	/* 梅森旋轉演算法 */
//	std::mt19937 generator(rd());
//
//	std::uniform_real_distribution<float> unif(min, max);
//
//	vector<double> radom_numbers;
//	for (int i = 0; i < how_many; i++)
//		radom_numbers.push_back(unif(generator));
//
//	/* 隨機設備的熵值 */
//	if (rd.entropy() < 10) {//隨意給的門檻，通常entropy是32
//		DEBUG_PRINT_MESSAGE("warning! random seed runing out");
//	}
//
//	return radom_numbers;
//}
//
//double MathTool::getRandomNumber(double min, double max)
//{
//	if (random_numbers.size() < 1)
//		random_numbers = randomFactory(0, 1, 100);
//
//	double first = random_numbers[0];
//	random_numbers.erase(random_numbers.begin());
//
//	double scale = max - min;
//
//	return first * scale + min;
//}

VectorXd MathTool::mat2Vector(Mat in)
{
	VectorXd out(Map<VectorXd>(in.data(), in.cols()*in.rows()));
	return out;
}

Matrix4d MathTool::poseZYZ2SE3(VectorXd pose)
{
	//%
	//T(1:3, 4) = P(1:3);
	//w = P(4:6);
	//T(1:3, 1 : 3) = ZYZToSO3(w);
	//T(4, :) = [0 0 0 1];
	Matrix3d so3 = ZYZ2SO3_2(pose.segment(3, 3));
	Matrix4d se3;
	se3 << so3(0, 0), so3(0, 1), so3(0, 2), pose(0),
		so3(1, 0), so3(1, 1), so3(1, 2), pose(1),
		so3(2, 0), so3(2, 1), so3(2, 2), pose(2),
		0, 0, 0, 1;



	return se3;
}

Matrix3d MathTool::ZYZ2SO3_2(Vector3d w)
{
	// so3mat = [0, -omg(3), omg(2);
	//			omg(3), 0, -omg(1);
	//			-omg(2), omg(1), 0];
	double phi = w(0);
	double theta = w(1);
	double psi = w(2);

	return rotZ(phi)*rotY(theta)*rotZ(psi);
}

Mat MathTool::RPY2SO3_2(Vector3d w)
{
	double phi = w(0);
	double theta = w(1);
	double psi = w(2);

	return rotZ(phi)*rotY(theta)*rotX(psi);
}

Mat MathTool::poseRPY2SO3_2(VectorXd Pose)
{
	return RPY2SO3_2(Pose.segment(3, 3));
}

Matrix4d MathTool::poseRPY2SE3(VectorXd pose)
{
	Matrix3d so3 = RPY2SO3_2(pose.segment(3, 3));
	Matrix4d se3;
	se3 << so3(0, 0), so3(0, 1), so3(0, 2), pose(0),
		so3(1, 0), so3(1, 1), so3(1, 2), pose(1),
		so3(2, 0), so3(2, 1), so3(2, 2), pose(2),
		0, 0, 0, 1;

	return se3;
}

VectorXd MathTool::sE3ToPoseAxisAngle(MatrixXd T)
{
	//%
	//p = T(1:3, 4);
	//R = T(1:3, 1 : 3);
	//w = SO3ToAxisAngle(R);
	//P = [p; w];
	Vector3d p = { T(0,3),T(1,3),T(2,3) };
	MatrixXd R = T.block(0, 0, 3, 3);
	Vector3d w = sO3ToPoseAxisAngle(R);

	VectorXd P(6);
	P << p, w;

	//TODO:以下matlab code可能要用超載實現
	//if nargout <= 1
	//	varargout{ 1 } = P;
	//elseif nargout == 2
	//	varargout{ 1 } = p;
	//varargout{ 2 } = w;
	//else
	//	error('Wrong number of output arguments')
	//	end

	return P;
}

VectorXd MathTool::sE3ToPoseRPY(MatrixXd T)
{

	Vector3d p = { T(0,3),T(1,3),T(2,3) };
	MatrixXd R = T.block(0, 0, 3, 3);

	Vector3d w = sO3ToPoseRPY(R);

	//DEBUG_PRINT_VALUE(R)
	//DEBUG_PRINT_VALUE(w)


	VectorXd P(6);
	P << p, w;


	return P;
}

Vector3d MathTool::sO3ToPoseAxisAngle(MatrixXd R)
{
	//以下註解部分為原matlab code，不知道哪來的妖魔鬼怪。。。

	/*epsilon = 0.01;
	epsilon2 = 0.1;

	if ((abs(R(1, 2) - R(2, 1)) < epsilon) && (abs(R(1, 3) - R(3, 1)) < epsilon)...
		&& (abs(R(2, 3) - R(3, 2)) < epsilon))
		if ((abs(R(1, 2) + R(2, 1)) < epsilon2)...
			&& (abs(R(1, 3) + R(3, 1)) < epsilon2)...
			&& (abs(R(2, 3) + R(3, 2)) < epsilon2)...
			&& (abs(R(1, 1) + R(2, 2) + R(3, 3) - 3) < epsilon2))
			omega = [0, 0, 0]';
			%         P = 0
		else

			angle = pi;
	xx = (R(1, 1) + 1) / 2;
	yy = (R(2, 2) + 1) / 2;
	zz = (R(3, 3) + 1) / 2;
	xy = (R(1, 2) + R(2, 1)) / 4;
	xz = (R(1, 3) + R(3, 1)) / 4;
	yz = (R(2, 3) + R(3, 2)) / 4;

	if ((xx > yy) && (xx > zz))
		% P = 1
		if (xx < epsilon)
			x = 0;
	y = 0.7071;
	z = 0.7071;
		else
			x = sqrt(xx);
	y = xy / x;
	z = xz / x;
	end
		elseif(yy > zz)
		% P = 2
		if (yy < epsilon)
			x = 0.7071;
	y = 0;
	z = 0.7071;
		else
			y = sqrt(yy);
	x = xy / y;
	z = yz / y;
	end
	else
		%             P = 3
		if (zz < epsilon)
			x = 0.7071;
	y = 0.7071;
	z = 0;
		else
			z = sqrt(zz);
	x = xz / z;
	y = yz / z;
	end
		end
		omega = angle * [x, y, z]';
		end
	else
		s = sqrt((R(3, 2) - R(2, 3))*(R(3, 2) - R(2, 3))...
			+ (R(1, 3) - R(3, 1))*(R(1, 3) - R(3, 1))...
			+ (R(2, 1) - R(1, 2))*(R(2, 1) - R(1, 2)));
	if (abs(s) < 0.001)
		% P = 4
		s = 1;
	end

		angle = acos((R(1, 1) + R(2, 2) + R(3, 3) - 1) / 2);
	x = (R(3, 2) - R(2, 3)) / s;
	y = (R(1, 3) - R(3, 1)) / s;
	z = (R(2, 1) - R(1, 2)) / s;
	omega = angle * [x, y, z]';
		end*/


	double epsilon = 1e-5;
	double epsilon2 = 1e-5;

	Vector3d omega;
	double angle, xx, yy, zz, xy, xz, yz;
	double x, y, z, s;

	if ((abs(R(0, 1) - R(1, 0)) < epsilon)
		&& (abs(R(0, 2) - R(2, 0)) < epsilon)
		&& (abs(R(1, 2) - R(2, 1)) < epsilon)) {

		if ((abs(R(0, 1) + R(1, 0)) < epsilon2)
			&& (abs(R(0, 2) + R(2, 0)) < epsilon2)
			&& (abs(R(1, 2) + R(2, 1)) < epsilon2)
			&& (abs(R(0, 0) + R(1, 1) + R(2, 2) - 3) < epsilon2)) {

			omega << 0, 0, 0;
		}
		else {
			angle = MathPI;
			xx = (R(0, 0) + 1) / 2;
			yy = (R(1, 1) + 1) / 2;
			zz = (R(2, 2) + 1) / 2;
			xy = (R(0, 1) + R(1, 0)) / 4;
			xz = (R(0, 2) + R(2, 0)) / 4;
			yz = (R(1, 2) + R(2, 1)) / 4;

			if ((xx > yy) && (xx > zz)) {

				if (xx < epsilon) {
					x = 0;
					y = 0.7071;
					z = 0.7071;
				}
				else {
					x = sqrt(xx);
					y = xy / x;
					z = xz / x;
				}
			}
			else if (yy > zz) {

				if (yy < epsilon) {
					x = 0.7071;
					y = 0;
					z = 0.7071;
				}
				else {
					y = sqrt(yy);
					x = xy / y;
					z = yz / y;
				}
			}
			else {

				if (zz < epsilon) {
					x = 0.7071;
					y = 0.7071;
					z = 0;
				}
				else {
					z = sqrt(zz);
					x = xz / z;
					y = yz / z;
				}
			}

			omega << x, y, z;
			omega *= angle;
		}
	}
	else {

		s = sqrt((R(2, 1) - R(1, 2))*(R(2, 1) - R(1, 2))
			+ (R(0, 2) - R(2, 0))*(R(0, 2) - R(2, 0))
			+ (R(1, 0) - R(0, 1))*(R(1, 0) - R(0, 1)));

		if (abs(s) < 0.001) {
			s = 1;
		}


		angle = acos((R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2);
		x = (R(2, 1) - R(1, 2)) / s;
		y = (R(0, 2) - R(2, 0)) / s;
		z = (R(1, 0) - R(0, 1)) / s;

		omega << x, y, z;
		omega *= angle;
	}



	//TODO:以下matlab code可能要用超載實現
	/*
		if nargout <= 1
			varargout{ 1 } = omega;
	elseif nargout == 2
		varargout{ 1 } = [x; y; z];
	varargout{ 2 } = angle;
		else
			error('Wrong number of output arguments');
	end*/

	return omega;
}


Vector3d MathTool::sO3ToPoseRPY(MatrixXd R)
{
	Vector3d s{ R(0,0),R(1,0), R(2,0) };
	Vector3d n{ R(0,1),R(1,1), R(2,1) };
	Vector3d a{ R(0,2),R(1,2), R(2,2) };

	double phi = atan2(s(1), s(0));
	double theta = atan2(-s(2), cos(phi)*s(0) + sin(phi)*s(1));
	double psi = atan2(sin(phi)*a(0) - cos(phi)*a(1), -sin(phi)*n(0) + cos(phi)*n(1));

	Vector3d rpy{ phi,theta,psi };

	return rpy;
}

 Matrix3d MathTool::rotZ(double theta)
{
	Matrix3d r;
	r << cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;
	return r;
}

 Matrix3d MathTool::rotY(double theta)
{
	Matrix3d r;
	r << cos(theta), 0, sin(theta),
		0, 1, 0,
		-sin(theta), 0, cos(theta);
	return r;
}

 Matrix3d MathTool::rotX(double theta)
{
	Matrix3d r;
	r << 1, 0, 0,
		0, cos(theta), -sin(theta),
		0, sin(theta), cos(theta);
	return r;
}

MatrixXd MathTool::reshape(MatrixXd mat_in, int rows, int cols)
{
	Map<MatrixXd> out(mat_in.data(), rows, cols);

	return out;
}

MatrixXd MathTool::pinv(MatrixXd mat_in)
{
	auto svd = mat_in.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

	const auto & singularValues = svd.singularValues();

	Matrix < double, Dynamic, Dynamic> singularValuesInv(mat_in.cols(), mat_in.rows());
	singularValuesInv.setZero();
	double pinvtolar = 1.e-6;
	for (unsigned int i = 0; i < singularValues.size(); i++) {
		if (singularValues(i) > pinvtolar) {
			singularValuesInv(i, i) = 1.0f / singularValues(i);
		}
		else {
			singularValuesInv(i, i) = 0.f;
		}
	}
	MatrixXd pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();

	return pinvmat;
}

Matrix3d MathTool::vect3tomatrix3(Vector3d v_in)
{
	Matrix3d mat_out;
	mat_out << 0, -v_in(2), v_in(1),
		v_in(2), 0, -v_in(0),
		-v_in(1), v_in(0), 0;
	return mat_out;
}

Mat MathTool::MatrixExp6(Mat se3mat)
{

	Mat se3mat_33(3, 3);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) se3mat_33(i, j) = se3mat(i, j);
	Mat omgtheta = so3ToVec(se3mat_33);

	if (NearZero(norm(omgtheta))) {
		// T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
		Mat T = Mat::Identity(4, 4);
		for (int i = 0; i < 3; i++) T(i, 3) = se3mat(i, 3);
		return T;
	}
	else
	{
		Mat omghat;
		double theta = AxisAng3(omgtheta, omghat);

		Mat omgmat = 1.0 / theta * se3mat_33;

		/*
		T = [MatrixExp3(se3mat(1: 3, 1: 3)), ...
		s(  eye(3) * theta + (1 - cos(theta)) * omgmat ...
		+ (theta - sin(theta)) * omgmat * omgmat )s ...
		* se3mat(1: 3, 4) / theta;
		0, 0, 0, 1];
		*/
		Mat T = Mat::Identity(4, 4);
		Mat tempA = MatrixExp3(se3mat_33);
		Mat tempB1 = theta * Mat::Identity(3, 3) + (1.0 - cos(theta)) * omgmat;
		Mat tempB2 = (theta - sin(theta)) * omgmat * omgmat;
		Mat se3mat_p(3, 1);
		for (int i = 0; i < 3; i++) se3mat_p(i, 0) = se3mat(i, 3);
		Mat tempB = (1.0 / theta) * (tempB1 + tempB2) * se3mat_p;
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) T(i, j) = tempA(i, j);
		for (int i = 0; i < 3; i++) T(i, 3) = tempB(i, 0);
		//DEBUG_PRINT_VALUE(se3mat);
		//DEBUG_PRINT_VALUE(tempB1);
		//DEBUG_PRINT_VALUE(tempB2);
		//DEBUG_PRINT_VALUE(se3mat_p);
		//DEBUG_PRINT_VALUE(tempB);
		//DEBUG_PRINT_VALUE(T);
		return T;
	}
}

Mat MathTool::so3ToVec(Mat so3Mat)
{
	//% Takes a 3x3 skew - symmetric matrix(an element of so(3)).
	//	% Returns the corresponding 3 - vector(angular velocity).
	//	% Example Input :
	//%
	//	% clear; clc;
	//% so3mat = [[0, -3, 2]; [3, 0, -1]; [-2, 1, 0]];
	//% omg = so3ToVec(so3mat)
	//	%
	//	% Output:
	//% omg =
	//	% 1
	//	% 2
	//	% 3

	//	omg = [so3mat(3, 2); so3mat(1, 3); so3mat(2, 1)];
	//end

	Mat omg(3, 1);
	omg << so3Mat(2, 1), so3Mat(0, 2), so3Mat(1, 0);
	return omg;
}

bool MathTool::NearZero(double val)
{
	return (abs(val) < 1e-6);
}

double MathTool::norm(Mat mat)
{
	return mat.norm();
}

 double MathTool::AxisAng3(Mat expc3, Mat out_omghat)
{
	double theta = norm(expc3);
	out_omghat = 1.0 / theta * expc3;
	return theta;
}

Mat MathTool::MatrixExp3(Mat so3mat)
{
	Mat omgtheta = so3ToVec(so3mat);
	double norm_v = norm(omgtheta);
	if (NearZero(norm_v) == true)
	{
		// Mat omg = new Mat(new double[3, 1] { { so3Mat[2,1] }, { so3Mat[0, 2] }, { so3Mat[1, 0] } }, true);
		Mat R(3, 3);
		R = Mat::Zero(3, 3);
		for (int i = 0; i < 3; i++) R(i, i) = 1;
		return R;
	}
	else
	{
		// function [omghat, theta] = AxisAng3(expc3)
		double theta = norm_v;
		Mat omgmat = (1.0 / theta) * so3mat;
		// R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;

		Mat R = Mat::Identity(3, 3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
		return R;
	}
}

double MathTool::CubicTimeScaling(double Tf, double t)
{
	// s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3;
	if (Tf == 0) return 0;
	double A = t / Tf;
	double s = 3 * A * A - 2 * A * A * A;
	return s;
}

double MathTool::QuinticTimeScaling(double Tf, double t)
{
	// s = 10 * (t / Tf) ^ 3 - 15 * (t / Tf) ^ 4 + 6 * (t / Tf) ^ 5;
	if (Tf == 0) return 0;
	double A = t / Tf;
	double s = 10 * A * A * A - 15 * A * A * A * A + 6 * A * A * A * A * A;
	return s;
}

Matrix4d MathTool::MatrixLog6(Mat T)
{
	// [R, p] = TransToRp(T);
	// omgmat = MatrixLog3(R);
	Mat R(3, 3);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R(i, j) = T(i, j);
	Mat p(3, 1);
	p << T(0, 3), T(1, 3), T(2, 3);
	Mat omgmat = MatrixLog3(R);

	//DEBUG_PRINT_VALUE(R);
	//DEBUG_PRINT_VALUE(p);
	//DEBUG_PRINT_VALUE(omgmat);

	bool b_all_zero = true;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (!NearZero(omgmat(i, j)))
			{
				b_all_zero = false;
				break;
			}
		}
		if (b_all_zero == false) break;
	}
	if (b_all_zero == true)
	{
		//DEBUG_PRINT_VALUE(omgmat)
		//DEBUG_PRINT_VALUE(T)

		Matrix4d expmat = Matrix4d::Zero();
		//DEBUG_PRINT_VALUE(expmat)
		for (int i = 0; i < 3; i++) expmat(i, 3) = T(i, 3);
		//DEBUG_PRINT_VALUE(expmat)
		return expmat;
	}
	else
	{
		//theta = acos((trace(R) - 1) / 2);
		//expmat = [omgmat, (eye(3) - omgmat / 2 ...
		//	+ (1 / theta - cot(theta / 2) / 2) ...
		//	* omgmat * omgmat / theta) * p;
		//0, 0, 0, 0];
		double theta = acos((Trace(R) - 1.0) / 2.0);
		Mat tempA1 = Mat::Identity(3, 3) - 0.5 * omgmat;
		Mat tempA2 = (1.0 / theta - cot(0.5 * theta)*0.5) / theta * omgmat * omgmat;
		Mat tempA = (tempA1 + tempA2) * p;
		Matrix4d expmat;
		expmat << omgmat, tempA, 0, 0, 0, 0;
		//DEBUG_PRINT_VALUE(cot(0.5 * theta));
		//DEBUG_PRINT_VALUE((1.0 / theta - cot(0.5 * theta)*0.5));
		//DEBUG_PRINT_VALUE(tempA2);
		//DEBUG_PRINT_VALUE(tempA1 + tempA2);
		//DEBUG_PRINT_VALUE(tempA);

		return expmat;
	}
}

Matrix4d MathTool::TransInv(Mat T)
{
	Mat R(3, 3);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R(i, j) = T(i, j);
	// Vector2d p = { T(0, 3) ,  T(1, 3) ,  T(2, 3) };
	Mat p(3, 1);
	p << T(0, 3), T(1, 3), T(2, 3);
	// invT = [R', -R' * p; 0, 0, 0, 1];
	Mat R_ = R.transpose();
	Mat R_p = -R_ * p;
	Matrix4d invT = Mat::Identity(4, 4);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) invT(i, j) = R_(i, j);
	for (int i = 0; i < 3; i++) invT(i, 3) = R_p(i, 0);
	return invT;
}

double MathTool::Trace(Mat m)
{
	return m.trace();
}

Matrix3d MathTool::MatrixLog3(Mat R)
{
	double trace_sum = R(0, 0) + R(1, 1) + R(2, 2);
	double acosinput = (trace_sum - 1) / 2.0;

	Matrix3d so3mat;
	so3mat = Mat::Zero(3, 3);
	if (acosinput >= 1)
	{
		return so3mat;
	}
	else if (acosinput <= -1)
	{
		Mat omg(3, 1);
		if (!NearZero(1 + R(2, 2)))
		{
			double coef = (1.0 / sqrt(2.0 * (1.0 + R(2, 2))));
			omg(0, 0) = coef * R(0, 2);
			omg(1, 0) = coef * R(1, 2);
			omg(2, 0) = coef * (1.0 + R(2, 2));
		}
		else if (!NearZero(1 + R(1, 1)))
		{
			double coef = (1.0 / sqrt(2.0 * (1.0 + R(1, 1))));
			omg(0, 0) = coef * R(0, 1);
			omg(1, 0) = coef * (1.0 + R(1, 1));
			omg(2, 0) = coef * R(2, 1);
		}
		else
		{
			double coef = (1.0 / sqrt(2.0 * (1.0 + R(0, 0))));
			omg(0, 0) = coef * (1.0 + R(0, 0));
			omg(1, 0) = coef * R(1, 0);
			omg(2, 0) = coef * R(2, 0);
		}
		so3mat = VecToso3(MathPI * omg);
	}
	else
	{
		// theta = acos(acosinput);
		// so3mat = theta * (1 / (2 * sin(theta))) * (R - R');

		double theta = acos(acosinput);
		so3mat = theta * (0.5 / sin(theta)) * (R - R.transpose());
	}

	return so3mat;
}

VectorXd MathTool::SE3ToPoseZYZ(Mat T)
{
	//x = T(1, 4);
	//y = T(2, 4);
	//z = T(3, 4);
	//s = T(1:3, 1);
	//n = T(1:3, 2);
	//a = T(1:3, 3);
	double x = T(0, 3);
	double y = T(1, 3);
	double z = T(2, 3);
	Vector3d s = { T(0,0),T(1,0),T(2,0) };
	Vector3d n = { T(0,1),T(1,1),T(2,1) };
	Vector3d a = { T(0,2),T(1,2),T(2,2) };

	double phi = atan2(a(1), a(0));
	double theta = atan2(a(0)*cos(phi) + a(1)*sin(phi), a(2));
	double psi = atan2(s(1)*cos(phi) - s(0)*sin(phi), -n(0)*sin(phi) + n(1)*cos(phi));

	VectorXd poseZYZ(6);
	poseZYZ << x, y, z, phi, theta, psi;
	return poseZYZ;
}

 vector<double> MathTool::vectorXDtoStdVector(VectorXd v)
{
	vector<double> vd(v.data(), v.data() + v.size());
	return vd;
}

 VectorXd MathTool::stdVectortoVectorXD(vector<double> v)
{
	return Eigen::Map<Eigen::VectorXd>(v.data(), v.size());
}

 Matrix3d MathTool::VecToso3(Mat omg)
{
	Matrix3d so3Mat;
	// so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
	so3Mat << 0, -omg(2, 0), omg(1, 0),
		omg(2, 0), 0, -omg(0, 0),
		-omg(1, 0), omg(0, 0), 0;

	return so3Mat;
}

 Matrix4d MathTool::rp2t(Matrix3d r, Vector3d p)
{
	Matrix4d t;
	t << r, p, 0, 0, 0, 1;
	return t;
}

 Matrix3d MathTool::t2r(Matrix4d t)
{
	MatrixXd R = t.block(0, 0, 3, 3);
	return R;
}

 Vector3d MathTool::t2p(Matrix4d t)
{
	Vector3d p = { t(0,3),t(1,3),t(2,3) };
	return p;
}

Matrix3d MathTool::AxisAngle2SO3(Vector3d w)
{
	if (NearZero(norm(w))) return Matrix3d::Identity(3, 3);

	double theta = norm(w);
	w = w / theta;
	Matrix3d wx = VecToso3(w);
	Matrix3d so3 = Matrix3d::Identity(3, 3) + sin(theta)*wx + (1 - cos(theta))*wx*wx;
	return so3;
}

//vector<double> MathTool::findAll(vector<double> source, double target)
//{
//	std::vector<double> matches;
//	std::vector<double>::iterator i = source.begin();
//	//MyPred my_pred;
//	while (true) {
//		i = std::find_if(i, source.end(),
//			[](double x, double target) { return (x > target); });
//		if (i == source.end())
//			break;
//		matches.push_back(*i);
//	}
//
//	return matches;
//}

template<typename _Matrix_Type_>
_Matrix_Type_ pinv2(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV()
		*(svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()
		* svd.matrixU().adjoint();
}

