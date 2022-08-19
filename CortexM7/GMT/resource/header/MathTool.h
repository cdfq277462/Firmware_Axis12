#pragma once

#pragma region include

#include <iostream>
#include <vector>

//#define H755DEMO

#if defined(H755DEMO)
#include "arm_math.h"
#include "math.h"
#include "../GNU/cminpack-1.3.8/minpack.h"
#include "../GNU/eigen-3.4.0/Eigen/Dense"
/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"
#else
#include "../3rd_party/eigen-3.3.9/Eigen/Dense"
#include "../3rd_party/cminpack-1.3.8/cminpack.h"  
#include "../3rd_party/cminpack-1.3.8/minpack.h" 
#endif

#include "messageTable.h"

#pragma endregion

#pragma region define


//#pragma comment(lib, " cminpack")

#define real __cminpack_real__

#define MathPI 3.14159265358979323846
#define cot(X) (tan(MathPI*0.5 -X))
#define SQUARE(X) pow(X,2)

#define Mat MatrixXd
#define MatList vector<MatrixXd>
#define pinv0(X) X.completeOrthogonalDecomposition().pseudoInverse()

//就是一種type define，只不過儲存/傳遞的是function
typedef double(*LightSourceFunc)(void);




//#if defined(__WIN32__) || defined(WIN32) || defined (_WIN32) || defined (__linux__) //__unix__
//#define ERROR_MESSAGE(MSG) throw invalid_argument(MSG);
#if defined(UNDER_RTSS) || defined(NO_MESSAGE)
//#define DEBUG_PRINT_MESSAGE(STR) printf("%s",STR.c_str());
//#define ERROR_MESSAGE(MSG) printf("ERROR : %s",MSG.c_str());
#define DEBUG_PRINT_ERROR_MESSAGE(MSG) 
#define DEBUG_PRINT_MESSAGE(STR) 
#define DEBUG_PRINT_VALUE(VAR) 
#define DEBUG_SET_COUT_BOOL_TO_STRING
#define DEBUG_PRINT_FUNCTION_NAME
#define DEBUG_PRINT_FUNCTION_LINE
#define DEBUG_PRINT_FUNCTION_LINE
#define DEBUG_PRINT_CODE_LOCATION
#else 
#define DEBUG_PRINT_ERROR_MESSAGE(MSG) cout << endl << "ERROR:" << MSG << endl;
#define DEBUG_PRINT_MESSAGE(STR) cout << endl << STR << endl;
#define DEBUG_PRINT_VALUE(VAR) cout << endl << #VAR << endl << VAR << endl;
#define DEBUG_SET_COUT_BOOL_TO_STRING cout << std::boolalpha;
#define DEBUG_PRINT_FUNCTION_NAME std::cout << __func__ << std::endl;
#define DEBUG_PRINT_FUNCTION_LINE std::cout << __LINE__  << std::endl;
#define DEBUG_PRINT_FUNCTION_FILE std::cout << __FILE__   << std::endl;
#define DEBUG_PRINT_CODE_LOCATION std::cout << "at " << __FILE__ << " " << __func__ << "() line " << __LINE__ << std::endl;
#endif

#if defined(UNDER_RTSS) 
#define RELEASE_PRINT_VALUE(VAR) 
#define RELEASE_PRINT_MESSAGE(STR) 
#else 
#define RELEASE_PRINT_VALUE(VAR) cout << endl << #VAR << endl << VAR << endl;
#define RELEASE_PRINT_MESSAGE(STR) cout << endl << STR << endl;
#endif


#pragma endregion

using namespace std;
using namespace Eigen;


namespace GMT
{

#pragma region enumeration

	typedef enum
	{
		Cubic,      // 3rd-order
		Quintic,    // 5th-order
		Trapezoid,      // Trapezoid
		Sigmoid       // Sigmoid
	}enumTimeScaling;


#pragma endregion
	

	class MathTool
	{
	public:

		//角度轉弧度
		static double degree2Radian(double degree);

		//弧度轉角度
		static double radian2Degree(double radian);

		//w(zyz) abg轉  so3
		//實作使用展開式
		static Mat ZYZ2SO3(Vector3d w);
		//pose xyzabg轉 zyz so3
		//實作使用展開式
		static Mat poseZYZ2SO3(VectorXd Pose);
		//wToZYZSO3
		static Matrix4d poseZYZ2SE3(VectorXd pose);

		//結果和ZYZ2SO3相同
		//差別只在於實現過程使用矩陣乘法，而前者用展開式
		static Matrix3d ZYZ2SO3_2(Vector3d pose);




		//w(rpy=zyx) 轉  so3
		static Mat RPY2SO3_2(Vector3d w);
		//pose (rpy=zyx)轉 so3
		static Mat poseRPY2SO3_2(VectorXd Pose);
		//pose (rpy=zyx)轉 se3
		static Matrix4d poseRPY2SE3(VectorXd pose);




		//只能處理1*6的矩陣
		//將pose中的4、5、6分量 從角度轉弧度表示
		static VectorXd degree2Radian6(VectorXd Pose_deg);
		//只能處理1*12的矩陣
		//將pose中的6~11分量 從弧度轉角度表示
		static VectorXd radian2Degree12(VectorXd Pose_rad);

		////使用C++標準函式庫<random>中的random_device，資源有限不適合大量呼叫?
		////參考 https://blog.gtwang.org/programming/cpp-random-number-generator-and-probability-distribution-tutorial/
		//static vector<double> randomFactory(double min, double max, int how_many);
		//static double getRandomNumber(double min, double max);


		static VectorXd mat2Vector(Mat in);


		static VectorXd sE3ToPoseAxisAngle(MatrixXd se3mat);


		static Vector3d sO3ToPoseAxisAngle(MatrixXd so3mat);


		static VectorXd sE3ToPoseRPY(MatrixXd se3mat);


		static Vector3d sO3ToPoseRPY(MatrixXd so3mat);


		static Matrix3d rotZ(double theta);
		static Matrix3d rotY(double theta);
		static Matrix3d rotX(double theta);


		static MatrixXd reshape(MatrixXd mat_in, int rows, int cols);


		static MatrixXd pinv(MatrixXd m);

		static Matrix3d vect3tomatrix3(Vector3d vin);

		static Mat MatrixExp6(Mat se3mat);

		static Mat so3ToVec(Mat so3Mat);

		static bool NearZero(double val);

		static double norm(Mat mat_in);

		static double AxisAng3(Mat expc3, Mat omghat);

		static Mat MatrixExp3(Mat so3mat);

		static double CubicTimeScaling(double Tf, double t);

		static double QuinticTimeScaling(double Tf, double t);

		static Matrix4d MatrixLog6(Mat T);

		static Matrix4d TransInv(Mat T);

		static double Trace(Mat M);

		static Matrix3d MatrixLog3(Mat R);

		static VectorXd SE3ToPoseZYZ(Mat T);

		static vector<double> vectorXDtoStdVector(VectorXd v);
		static VectorXd stdVectortoVectorXD(vector<double> v);

		static Matrix3d VecToso3(Mat omg);

		static Matrix4d rp2t(Matrix3d r, Vector3d p);
		static Matrix3d t2r(Matrix4d t);
		static Vector3d t2p(Matrix4d t);

		static Matrix3d AxisAngle2SO3(Vector3d w);

		//static vector<double> findAll(vector<double> source, double target);

	private:
		static vector<double> random_numbers;

	};


}
