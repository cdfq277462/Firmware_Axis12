#pragma once
#include <math.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <random>
#include <ctime>
#include <algorithm>

#include "stewart_lib.h"


#define real __cminpack_real__


#pragma region  steward


NIKOutput Stewart_GMT::N_Stewart_IK(VectorXd P, Mat pp, Mat bb, double h0, double g0)
{
    _pp = pp;
    _bb = bb;
    _h0 = h0;
    _g0 = g0;

    return N_Stewart_IK(P);
}

NIKOutput Stewart_GMT::N_Stewart_IK(VectorXd P)
{
    //%??????matlab??code


    Vector3d bd = P.segment(0,3);

    Mat R = MathTool::poseRPY2SO3_2(P);

    Vector3d bb1 = _bb.col(0);
    Vector3d bb2 = _bb.col(1);
    Vector3d bb3 = _bb.col(2);
    Vector3d bb4 = _bb.col(3);
    Vector3d bb5 = _bb.col(4);
    Vector3d bb6 = _bb.col(5);

    Vector3d pp1 = _pp.col(0);
    Vector3d pp2 = _pp.col(1);
    Vector3d pp3 = _pp.col(2);
    Vector3d pp4 = _pp.col(3);
    Vector3d pp5 = _pp.col(4);
    Vector3d pp6 = _pp.col(5);

    Vector3d Bx1 = bd - bb1;
    Vector3d Bx2 = bd - bb2;
    Vector3d Bx3 = bd - bb3;
    Vector3d Bx4 = bd - bb4;
    Vector3d Bx5 = bd - bb5;
    Vector3d Bx6 = bd - bb6;

    Vector3d Bp1 = R * pp1;
    Vector3d Bp2 = R * pp2;
    Vector3d Bp3 = R * pp3;
    Vector3d Bp4 = R * pp4;
    Vector3d Bp5 = R * pp5;
    Vector3d Bp6 = R * pp6;

    Vector3d Lv1 = Bx1 + Bp1;
    Vector3d Lv2 = Bx2 + Bp2;
    Vector3d Lv3 = Bx3 + Bp3;
    Vector3d Lv4 = Bx4 + Bp4;
    Vector3d Lv5 = Bx5 + Bp5;
    Vector3d Lv6 = Bx6 + Bp6;

    double lz1 = Lv1(2);
    double lz2 = Lv2(2);
    double lz3 = Lv3(2);
    double lz4 = Lv4(2);
    double lz5 = Lv5(2);
    double lz6 = Lv6(2);
    VectorXd alpha(6);
    alpha <<  lz1, lz2, lz3, lz4, lz5, lz6 ;

    double l1 = sqrt(SQUARE(Bx1(0) + Bp1(0)) + SQUARE(Bx1(1) + Bp1(1)) + SQUARE(Bx1(2) + Bp1(2)));
    double l2 = sqrt(SQUARE(Bx2(0) + Bp2(0)) + SQUARE(Bx2(1) + Bp2(1)) + SQUARE(Bx2(2) + Bp2(2)));
    double l3 = sqrt(SQUARE(Bx3(0) + Bp3(0)) + SQUARE(Bx3(1) + Bp3(1)) + SQUARE(Bx3(2) + Bp3(2)));
    double l4 = sqrt(SQUARE(Bx4(0) + Bp4(0)) + SQUARE(Bx4(1) + Bp4(1)) + SQUARE(Bx4(2) + Bp4(2)));
    double l5 = sqrt(SQUARE(Bx5(0) + Bp5(0)) + SQUARE(Bx5(1) + Bp5(1)) + SQUARE(Bx5(2) + Bp5(2)));
    double l6 = sqrt(SQUARE(Bx6(0) + Bp6(0)) + SQUARE(Bx6(1) + Bp6(1)) + SQUARE(Bx6(2) + Bp6(2)));

    VectorXd LL(6);
    LL << l1, l2, l3, l4, l5, l6;

    return NIKOutput(alpha, LL);
    /*
    Mat tarr = alpha.array().square() + _h0 * _h0 - LL.array().square();
    auto illegal_lenth = (tarr.array() < 0);
    if (illegal_lenth.any()) {
        ERROR_MESSAGE("Pose cannot be reached")
            ERROR_MESSAGE("IK fail !!!")
    }

    VectorXd tv = tarr.array().sqrt();
    VectorXd g = alpha - tv;
    VectorXd dg = g.array() - _g0;
    */

    ////debug out
    //DEBUG_PRINT_VALUE(tarr);
    //DEBUG_PRINT_VALUE(tv);
    //DEBUG_PRINT_VALUE(g);
    //DEBUG_PRINT_VALUE(dg);

    //end
    //return dg;
}

VectorXd Stewart_GMT::F_Stewart_IK(VectorXd P, Mat pp, Mat bb)
{
    _pp = pp;
    _bb = bb;

    return F_Stewart_IK(P);
}

VectorXd Stewart_GMT::F_Stewart_IK(VectorXd P)
{

    Vector3d bd = P.segment(0, 3);

    Mat R = MathTool::poseRPY2SO3_2(P);

    Vector3d bb1 = _bb.col(0);
    Vector3d bb2 = _bb.col(1);
    Vector3d bb3 = _bb.col(2);
    Vector3d bb4 = _bb.col(3);
    Vector3d bb5 = _bb.col(4);
    Vector3d bb6 = _bb.col(5);

    Vector3d pp1 = _pp.col(0);
    Vector3d pp2 = _pp.col(1);
    Vector3d pp3 = _pp.col(2);
    Vector3d pp4 = _pp.col(3);
    Vector3d pp5 = _pp.col(4);
    Vector3d pp6 = _pp.col(5);

    Vector3d Bx1 = bd - bb1;
    Vector3d Bx2 = bd - bb2;
    Vector3d Bx3 = bd - bb3;
    Vector3d Bx4 = bd - bb4;
    Vector3d Bx5 = bd - bb5;
    Vector3d Bx6 = bd - bb6;

    Vector3d BP1 = R * pp1;
    Vector3d BP2 = R * pp2;
    Vector3d BP3 = R * pp3;
    Vector3d BP4 = R * pp4;
    Vector3d BP5 = R * pp5;
    Vector3d BP6 = R * pp6;

    double l1 = sqrt(pow(Bx1(0) + BP1(0), 2) + pow(Bx1(1) + BP1(1), 2) + pow(Bx1(2) + BP1(2), 2));
    double l2 = sqrt(pow(Bx2(0) + BP2(0), 2) + pow(Bx2(1) + BP2(1), 2) + pow(Bx2(2) + BP2(2), 2));
    double l3 = sqrt(pow(Bx3(0) + BP3(0), 2) + pow(Bx3(1) + BP3(1), 2) + pow(Bx3(2) + BP3(2), 2));
    double l4 = sqrt(pow(Bx4(0) + BP4(0), 2) + pow(Bx4(1) + BP4(1), 2) + pow(Bx4(2) + BP4(2), 2));
    double l5 = sqrt(pow(Bx5(0) + BP5(0), 2) + pow(Bx5(1) + BP5(1), 2) + pow(Bx5(2) + BP5(2), 2));
    double l6 = sqrt(pow(Bx6(0) + BP6(0), 2) + pow(Bx6(1) + BP6(1), 2) + pow(Bx6(2) + BP6(2), 2));

    VectorXd L(6);
    L << l1, l2, l3, l4, l5, l6;

    return L;
}



Mat Stewart_GMT::stewartJacobian(VectorXd P, Mat pp, Mat bb)
{

    //% Position and orientation vector
    //bd = Pose(1:3);

    Vector3d bd = P.segment(0, 3);

    //%
    //alpha = Pose(4);
    //beta = Pose(5);
    //gamma = Pose(6);
    //%--------Z(alpha) - Y(beta) - Z(gamma)------------------------
    //r11 = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
    //r21 = cos(alpha)*sin(gamma) + cos(beta)*cos(gamma)*sin(alpha);
    //r31 = -cos(gamma)*sin(beta);
    //r12 = -cos(gamma)*sin(alpha) - cos(alpha)*cos(beta)*sin(gamma);
    //r22 = cos(alpha)*cos(gamma) - cos(beta)*sin(alpha)*sin(gamma);
    //r32 = sin(beta)*sin(gamma);
    //r13 = cos(alpha)*sin(beta);
    //r23 = sin(alpha)*sin(beta);
    //r33 = cos(beta);
    //%----------------------------------------------------------
    Mat R = MathTool::poseZYZ2SO3(P);

    //%bbi fixed platform global coordinates
    //bb1 = bb(:, 1);
    //bb2 = bb(:, 2);
    //bb3 = bb(:, 3);
    //bb4 = bb(:, 4);
    //bb5 = bb(:, 5);
    //bb6 = bb(:, 6);

    Vector3d bb1 = bb.col(0);
    Vector3d bb2 = bb.col(1);
    Vector3d bb3 = bb.col(2);
    Vector3d bb4 = bb.col(3);
    Vector3d bb5 = bb.col(4);
    Vector3d bb6 = bb.col(5);

    //% ppi moving platform coordinates
    //pp1 = pp(:, 1);
    //pp2 = pp(:, 2);
    //pp3 = pp(:, 3);
    //pp4 = pp(:, 4);
    //pp5 = pp(:, 5);
    //pp6 = pp(:, 6);

    Vector3d pp1 = pp.col(0);
    Vector3d pp2 = pp.col(1);
    Vector3d pp3 = pp.col(2);
    Vector3d pp4 = pp.col(3);
    Vector3d pp5 = pp.col(4);
    Vector3d pp6 = pp.col(5);

    //%inverse kinematics for lengths of the actuators
    //Bx1 = bd - bb1;
    //Bx2 = bd - bb2;
    //Bx3 = bd - bb3;
    //Bx4 = bd - bb4;
    //Bx5 = bd - bb5;
    //Bx6 = bd - bb6;

    Vector3d Bx1 = bd - bb1;
    Vector3d Bx2 = bd - bb2;
    Vector3d Bx3 = bd - bb3;
    Vector3d Bx4 = bd - bb4;
    Vector3d Bx5 = bd - bb5;
    Vector3d Bx6 = bd - bb6;

    //%Bpi
    //Bp1 = R * pp1;
    //Bp2 = R * pp2;
    //Bp3 = R * pp3;
    //Bp4 = R * pp4;
    //Bp5 = R * pp5;
    //Bp6 = R * pp6;

    Vector3d Bp1 = R * pp1;
    Vector3d Bp2 = R * pp2;
    Vector3d Bp3 = R * pp3;
    Vector3d Bp4 = R * pp4;
    Vector3d Bp5 = R * pp5;
    Vector3d Bp6 = R * pp6;

    //%lzi
    //	lz1 = Bx1 + Bp1;
    //lz2 = Bx2 + Bp2;
    //lz3 = Bx3 + Bp3;
    //lz4 = Bx4 + Bp4;
    //lz5 = Bx5 + Bp5;
    //lz6 = Bx6 + Bp6;

    Vector3d lz1 = Bx1 + Bp1;
    Vector3d lz2 = Bx2 + Bp2;
    Vector3d lz3 = Bx3 + Bp3;
    Vector3d lz4 = Bx4 + Bp4;
    Vector3d lz5 = Bx5 + Bp5;
    Vector3d lz6 = Bx6 + Bp6;

    //% zi
    //	z1 = lz1 / norm(lz1);
    //z2 = lz2 / norm(lz2);
    //z3 = lz3 / norm(lz3);
    //z4 = lz4 / norm(lz4);
    //z5 = lz5 / norm(lz5);
    //z6 = lz6 / norm(lz6);

    Vector3d z1 = lz1 / lz1.norm();
    Vector3d z2 = lz2 / lz2.norm();
    Vector3d z3 = lz3 / lz3.norm();
    Vector3d z4 = lz4 / lz4.norm();
    Vector3d z5 = lz5 / lz5.norm();
    Vector3d z6 = lz6 / lz6.norm();

    //% six[six]
    //	s1x = [0 -Bp1(3) Bp1(2); ...
    //	Bp1(3) 0 -Bp1(1); ...
    //	-Bp1(2) Bp1(1) 0];

    Mat s1x(3, 3);
    s1x <<
        0, -Bp1(2), Bp1(1),
        Bp1(2), 0, -Bp1(0),
        -Bp1(1), Bp1(0), 0;

    //s2x = [0 -Bp2(3) Bp2(2); ...
    //	Bp2(3) 0 -Bp2(1); ...
    //	-Bp2(2) Bp2(1) 0];

    Mat s2x(3, 3);
    s2x <<
        0, -Bp2(2), Bp2(1),
        Bp2(2), 0, -Bp2(0),
        -Bp2(1), Bp2(0), 0;

    //s3x = [0 -Bp3(3) Bp3(2); ...
    //	Bp3(3) 0 -Bp3(1); ...
    //	-Bp3(2) Bp3(1) 0];

    Mat s3x(3, 3);
    s3x <<
        0, -Bp3(2), Bp3(1),
        Bp3(2), 0, -Bp3(0),
        -Bp3(1), Bp3(0), 0;

    //s4x = [0 -Bp4(3) Bp4(2); ...
    //	Bp4(3) 0 -Bp4(1); ...
    //	-Bp4(2) Bp4(1) 0];

    Mat s4x(3, 3);
    s4x <<
        0, -Bp4(2), Bp4(1),
        Bp4(2), 0, -Bp4(0),
        -Bp4(1), Bp4(0), 0;

    //s5x = [0 -Bp5(3) Bp5(2); ...
    //	Bp5(3) 0 -Bp5(1); ...
    //	-Bp5(2) Bp5(1) 0];

    Mat s5x(3, 3);
    s5x <<
        0, -Bp5(2), Bp5(1),
        Bp5(2), 0, -Bp5(0),
        -Bp5(1), Bp5(0), 0;

    //s6x = [0 -Bp6(3) Bp6(2); ...
    //	Bp6(3) 0 -Bp6(1); ...
    //	-Bp6(2) Bp6(1) 0];

    Mat s6x(3, 3);
    s6x <<
        0, -Bp6(2), Bp6(1),
        Bp6(2), 0, -Bp6(0),
        -Bp6(1), Bp6(0), 0;

    //% Jp
    //	Jp = [z1' (s1x*z1)'; ...
    //	z2' (s2x*z2)'; ...
    //	z3' (s3x*z3)'; ...
    //	z4' (s4x*z4)'; ...
    //	z5' (s5x*z5)'; ...
    //	z6' (s6x*z6)'];

    Mat Jp(6, 6);
    Jp <<
        z1.adjoint(), (s1x*z1).adjoint(),
        z2.adjoint(), (s2x*z2).adjoint(),
        z3.adjoint(), (s3x*z3).adjoint(),
        z4.adjoint(), (s4x*z4).adjoint(),
        z5.adjoint(), (s5x*z5).adjoint(),
        z6.adjoint(), (s6x*z6).adjoint();


    //% Js
    //	Js = [z1'*R -z1' zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3); ...
    //	zeros(1, 3) zeros(1, 3) z2'*R -z2' zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3); ...
    //	zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) z3'*R -z3' zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3); ...
    //	zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) z4'*R -z4' zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3); ...
    //	zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) z5'*R -z5' zeros(1, 3) zeros(1, 3); ...
    //	zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) zeros(1, 3) z6'*R -z6'];

    Mat zero(1, 3);
    zero << 0, 0, 0;

    Mat Js(6, 36);
    Js <<
        z1.adjoint()*R, -z1.adjoint(), zero, zero, zero, zero, zero, zero, zero, zero, zero, zero,
        zero, zero, z2.adjoint()*R, -z2.adjoint(), zero, zero, zero, zero, zero, zero, zero, zero,
        zero, zero, zero, zero, z3.adjoint()*R, -z3.adjoint(), zero, zero, zero, zero, zero, zero,
        zero, zero, zero, zero, zero, zero, z4.adjoint()*R, -z4.adjoint(), zero, zero, zero, zero,
        zero, zero, zero, zero, zero, zero, zero, zero, z5.adjoint()*R, -z5.adjoint(), zero, zero,
        zero, zero, zero, zero, zero, zero, zero, zero, zero, zero, z6.adjoint()*R, -z6.adjoint();


    //% A
    //	A = [inv(Jp) - inv(Jp)*Js];

    Mat A(6, 42);
    A << Jp.inverse(), -Jp.inverse()*Js;

    return A;

    ////% svd
    ////	[U, D, V] = svd(A);
    //JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    //auto U = svd.matrixU();
    ////auto V = svd.matrixV();
    ////auto A2 = svd.singularValues();

}

VectorXd Stewart_GMT::N_Stewart_FK()
{
    VectorXd xinit(6);
    xinit << 0, 0, 75, 0, 0, 0;
    return N_Stewart_FK(xinit);
}

//x:6*1??????,??minpack?hybrid???matlab fsolve????????????
VectorXd Stewart_GMT::N_Stewart_FK(VectorXd x)
{
#pragma region hybrid1??

    double *fvec;  //output array of length n which contains the functions evaluated at the output x.
    int iflag;     //...not quite sure on this one
    int info;      //integer output variable. If the user has terminated execution, info is set to the (negative) value of iflag.  Otherwise RTFM
    int lwa;       //length of work array
    int n = 6;     //number of unknowns (x1, x2)
    double tol = 1.5e-8;
    double *wa;    //work array of length lwa
    //double *x;     //array of length n. On input x must contain an initial estimate of the solution vector. On output x contains the final estimate of the solution vector.

    lwa = (n * (3 * n + 13)) / 2;
    fvec = new double[n];  //mathematical output
    wa = new double[lwa];
    //	x = new double[n];     //mathematical input

    iflag = 1;
    N_myfun(&n, x.data(), fvec, &iflag);

#pragma endregion
#pragma region hybrid1 ?????


    /* Initialized data */

    const real factor = 6.226e-6;

    /* System generated locals */
    int i__1;

    /* Local variables */
    int j, ml, lr, mu, mode, nfev;
    real xtol;
    int index;
    real epsfcn;
    int maxfev, nprint;

    //--fvec;
    //--x;
    //--wa;

    /* Function Body */
    info = 0;

    /*     check the input parameters for errors. */

    if (n <= 0 || tol < 0. || lwa < n * (n * 3 + 13) / 2) {
        /* goto L20; */
        return x;
    }

    /*     call hybrd. */
    double *x_ptr = x.data();

    maxfev = (n + 1) * 200;
    xtol = tol;
    ml = n - 1;
    mu = n - 1;
    epsfcn = 0.;
    mode = 2;
    i__1 = n;
    for (j = 0; j < i__1; ++j) {
        wa[j] = 1.;
        /* L10: */
    }
    nprint = 0;
    lr = n * (n + 1) / 2;
    index = n * 6 + lr;
    __minpack_func__(hybrd)(N_myfun, &n, x_ptr, &fvec[0], &xtol, &maxfev, &ml, &mu, &epsfcn, &
        wa[0], &mode, &factor, &nprint, &info, &nfev, &wa[index], &n, &
        wa[n * 6], &lr, &wa[n], &wa[(n << 1)], &wa[n * 3
        ], &wa[(n << 2)], &wa[n * 5]);
    if (info == 5) {
        info = 4;
    }

#pragma endregion

    if (_is_show_fk_debug)
    {
        DEBUG_PRINT_MESSAGE( "  Returned value of INFO = " << info )
        DEBUG_PRINT_VALUE(x)
        DEBUG_PRINT_MESSAGE("  F(X)" << endl
         << fvec[0] << endl
         << fvec[1] << endl
         << fvec[2] << endl
         << fvec[3] << endl
         << fvec[4] << endl
         << fvec[5] << endl
         << endl
        )
    }

    delete[] fvec;
    delete[] wa;

    return x;
}

VectorXd Stewart_GMT::F_Stewart_FK(FTypeParameter fpara, VectorXd x)
{
    auto init_x = x;
    DEBUG_PRINT_VALUE(init_x);

#pragma region hybrid1??

    double *fvec;  //output array of length n which contains the functions evaluated at the output x.
    int iflag;     //...not quite sure on this one
    int info;      //integer output variable. If the user has terminated execution, info is set to the (negative) value of iflag.  Otherwise RTFM
    int lwa;       //length of work array
    int n = 6;     //number of unknowns (x1, x2)
    double tol = 1.5e-8;
    double *wa;    //work array of length lwa
    //double *x;     //array of length n. On input x must contain an initial estimate of the solution vector. On output x contains the final estimate of the solution vector.

    lwa = (n * (3 * n + 13)) / 2;
    fvec = new double[n];  //mathematical output
    wa = new double[lwa];
    //	x = new double[n];     //mathematical input

    iflag = 1;
    F_myfun(&n, x.data(), fvec, &iflag);

#pragma endregion
#pragma region hybrid1 ?????


    /* Initialized data */

    const real factor = 6.226e-6;

    /* System generated locals */
    int i__1;

    /* Local variables */
    int j, ml, lr, mu, mode, nfev;
    real xtol;
    int index;
    real epsfcn;
    int maxfev, nprint;

    //--fvec;
    //--x;
    //--wa;

    /* Function Body */
    info = 0;

    /*     check the input parameters for errors. */

    if (n <= 0 || tol < 0. || lwa < n * (n * 3 + 13) / 2) {
        /* goto L20; */
        return x;
    }

    /*     call hybrd. */
    double *x_ptr = x.data();

    maxfev = (n + 1) * 200;
    xtol = tol;
    ml = n - 1;
    mu = n - 1;
    epsfcn = 0.;
    mode = 2;
    i__1 = n;
    for (j = 0; j < i__1; ++j) {
        wa[j] = 1.;
        /* L10: */
    }
    nprint = 0;
    lr = n * (n + 1) / 2;
    index = n * 6 + lr;
    __minpack_func__(hybrd)(F_myfun, &n, x_ptr, &fvec[0], &xtol, &maxfev, &ml, &mu, &epsfcn, &
        wa[0], &mode, &factor, &nprint, &info, &nfev, &wa[index], &n, &
        wa[n * 6], &lr, &wa[n], &wa[(n << 1)], &wa[n * 3
        ], &wa[(n << 2)], &wa[n * 5]);
    if (info == 5) {
        info = 4;
    }

#pragma endregion

    if (_is_show_fk_debug)
    {
        DEBUG_PRINT_MESSAGE("  Returned value of INFO = " << info)
            DEBUG_PRINT_VALUE(x)
            DEBUG_PRINT_MESSAGE("  F(X)" << endl
                << fvec[0] << endl
                << fvec[1] << endl
                << fvec[2] << endl
                << fvec[3] << endl
                << fvec[4] << endl
                << fvec[5] << endl
                << endl
            )
    }

    delete[] fvec;
    delete[] wa;

    return x;
}

VectorXd Stewart_GMT::F_Stewart_FK(FTypeParameter fpara)
{
    VectorXd ini_pose(6);
    ini_pose << 0,
        0,
        168.3237959,
        0,
        0,
        0;

    DEBUG_PRINT_MESSAGE("FK???????")

    return F_Stewart_FK(fpara, ini_pose);
}


Mat Stewart_GMT::_pp(3,6);
Mat Stewart_GMT::_bb(3,6);
VectorXd Stewart_GMT::_L(6);
bool Stewart_GMT::_is_show_fk_debug = false;
double Stewart_GMT::_h0 = 0;
double Stewart_GMT::_g0 = 0;
double Stewart_GMT::_Lx = 0;
double Stewart_GMT::_Llow = 0;
double Stewart_GMT::_Lupp = 0;
double Stewart_GMT::_work_space_delta_trans = 0;
double Stewart_GMT::_work_space_delta_rot = 0;
vector<double> Stewart_GMT::_work_space_test_limit;
VectorXd Stewart_GMT::_phome(6);
Matrix4d Stewart_GMT::_TposeStart;
bool Stewart_GMT::_firstRun = true;
WSOutput Stewart_GMT::_LastLegalResult;


vector<double> MathTool::random_numbers = vector<double>();

void Stewart_GMT::set_F_Type_Stewart_DefaultParameter()
{
    _bb.col(0) << 97.25217186425962, -100.70757204741115, 0;
    _bb.col(1) << 38.58922981437988, -134.57663743136464, 0;
    _bb.col(2) << -135.8414016786395, -33.86906538395348, 0;
    _bb.col(3) << -135.8414016786395, 33.86906538395348, 0;
    _bb.col(4) << 38.58922981437988, 134.57663743136464, 0;
    _bb.col(5) << 97.25217186425962, 100.70757204741115, 0;

    _pp.col(0) << 83.40418343188286, -26.4573654406638, -20.3;
    _pp.col(1) << -18.78934112711802, -85.4588243542396, -20.3;
    _pp.col(2) << -64.6148423047648, -59.00145891357579, -20.3;
    _pp.col(3) << -64.6148423047648, 59.00145891357579, -20.3;
    _pp.col(4) << -18.78934112711802, 85.4588243542396, -20.3;
    _pp.col(5) << 83.40418343188286, 26.4573654406638, -20.3;


    _Lx = 167.5;
    _Lupp = 181;
    _Llow = 164;
    _work_space_delta_trans = 0.001;
    _work_space_delta_rot = 0.0001;

    //_phome << 0, 0, 66.4236, 0, 0, 0;

    _work_space_test_limit.clear();
    _work_space_test_limit.reserve(12);
    //x
    _work_space_test_limit.push_back(50);
    _work_space_test_limit.push_back(-50);
    //y
    _work_space_test_limit.push_back(40);
    _work_space_test_limit.push_back(-40);
    //z
    _work_space_test_limit.push_back(40);
    _work_space_test_limit.push_back(-40);
    //phi
    _work_space_test_limit.push_back(0.25);
    _work_space_test_limit.push_back(-0.25);
    //theta
    _work_space_test_limit.push_back(0.5);
    _work_space_test_limit.push_back(-0.5);
    //psi
    _work_space_test_limit.push_back(0.5);
    _work_space_test_limit.push_back(-0.5);
}

void Stewart_GMT::set_N_Type_Stewart_DefaultParameter()
{
#if 0U
    _bb.col(0) << 27.79095418441866, -25.6223118691413, 0;
    _bb.col(1) << 8.294095890154637, -36.87882825368669, 0;
    _bb.col(2) << -36.0850500745733, -11.25651638454532, 0;
    _bb.col(3) << -36.0850500745733, 11.25651638454532, 0;
    _bb.col(4) << 8.294095890154637, 36.87882825368669, 0;
    _bb.col(5) << 27.79095418441866, 25.6223118691413, 0;

    _pp.col(0) << 25.20360654775701, -8.3474976481494, 0;
    _pp.col(1) << -5.372658252550269, -26.00071236142009, 0;
    _pp.col(2) << -19.83094829520674, -17.65321471327068, 0;
    _pp.col(3) << -19.83094829520674, 17.65321471327068, 0;
    _pp.col(4) << -5.372658252550269, 26.00071236142009, 0;
    _pp.col(5) << 25.20360654775701, 8.3474976481494, 0;
#else
    _bb.col(5) << 27.79095418441866, -25.6223118691413, 0;
    _bb.col(4) << 8.294095890154637, -36.87882825368669, 0;
    _bb.col(3) << -36.0850500745733, -11.25651638454532, 0;
    _bb.col(2) << -36.0850500745733, 11.25651638454532, 0;
    _bb.col(1) << 8.294095890154637, 36.87882825368669, 0;
    _bb.col(0) << 27.79095418441866, 25.6223118691413, 0;

    _pp.col(5) << 25.20360654775701, -8.3474976481494, 0;
    _pp.col(4) << -5.372658252550269, -26.00071236142009, 0;
    _pp.col(3) << -19.83094829520674, -17.65321471327068, 0;
    _pp.col(2) << -19.83094829520674, 17.65321471327068, 0;
    _pp.col(1) << -5.372658252550269, 26.00071236142009, 0;
    _pp.col(0) << 25.20360654775701, 8.3474976481494, 0;
#endif
    _h0 = 21.2;
    _g0 = 49.41;
    _Lupp = 58.5;
    _Llow = 50.5;
    _work_space_delta_trans = 0.001;
    _work_space_delta_rot = 0.0001;

    _phome << 0, 0, 66.4236, 0, 0, 0;

    _work_space_test_limit.clear();
    _work_space_test_limit.reserve(12);
    //x
    _work_space_test_limit.push_back(50);
    _work_space_test_limit.push_back(-50);
    //y
    _work_space_test_limit.push_back(40);
    _work_space_test_limit.push_back(-40);
    //z
    _work_space_test_limit.push_back(40);
    _work_space_test_limit.push_back(-40);
    //phi
    _work_space_test_limit.push_back(0.5);
    _work_space_test_limit.push_back(-0.5);
    //theta
    _work_space_test_limit.push_back(0.5);
    _work_space_test_limit.push_back(-0.5);
    //psi
    _work_space_test_limit.push_back(0.5);
    _work_space_test_limit.push_back(-0.5);

}

void Stewart_GMT::set_A12_DefaultParameter()
{

    _phome << 0, 0, 0, 0, 0, 0;

    //x
    _work_space_test_limit.push_back(25);
    _work_space_test_limit.push_back(-25);
    //y
    _work_space_test_limit.push_back(25);
    _work_space_test_limit.push_back(-25);
    //z
    _work_space_test_limit.push_back(5);
    _work_space_test_limit.push_back(-5);
    //phi
    _work_space_test_limit.push_back(MathTool::degree2Radian(8.2));
    _work_space_test_limit.push_back(MathTool::degree2Radian(-8.2));
    //theta
    _work_space_test_limit.push_back(MathTool::degree2Radian(3.4));
    _work_space_test_limit.push_back(MathTool::degree2Radian(-3.4));
    //psi
    _work_space_test_limit.push_back(MathTool::degree2Radian(4.4));
    _work_space_test_limit.push_back(MathTool::degree2Radian(-4.4));

}

void Stewart_GMT::setPP(Mat pp)
{
    _pp = pp;
}
Mat Stewart_GMT::getPP()
{
    return _pp;
}
void Stewart_GMT::setBB(Mat bb)
{
    _bb = bb;
}
Mat Stewart_GMT::getBB()
{
    return _bb;
}
void Stewart_GMT::setH(double h0)
{
    _h0 = h0;
}
void Stewart_GMT::setG(double g0)
{
    _g0 = g0;
}
double Stewart_GMT::getG()
{
    return _g0;
}

void Stewart_GMT::setLx(double Lx)
{
    _Lx = Lx;
}

double Stewart_GMT::getLx()
{
    return _Lx;
}

bool Stewart_GMT::checkL(VectorXd L)
{
    /*
    % Check for the availability of length
        if ~(min(L0) >= Lx)
            error('The desired pose is out of working space')
            end
            */

    auto illegal_lenth = (L.array() < _Lx);
    if (illegal_lenth.any()) {
        DEBUG_PRINT_ERROR_MESSAGE("The desired pose is out of working space")
            return false;
    }

    return true;
}

void Stewart_GMT::setL(VectorXd L)
{
    _L = L;
}

VectorXd Stewart_GMT::getL()
{
    return _L;
}

void Stewart_GMT::setFKDebugMessage(bool show)
{
    _is_show_fk_debug = show;
}

pList Stewart_GMT::pivotTraj(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling)
{
    double n = ceill(tf / ts) + 1;

    Matrix4d xstart = MathTool::poseRPY2SE3(pstart);
    Matrix4d xs = MatrixXd::Identity(4, 4);
    Vector3d r = pivot;
    w = w / MathTool::norm(w);

    Matrix3d wx = MathTool::vect3tomatrix3(w);
    Matrix3d rx = MathTool::vect3tomatrix3(r);
    Matrix4d ms;
    ms << wx, rx*w, 0, 0, 0, 0;
    Matrix4d xend = MathTool::MatrixExp6(ms*theta);

    double timegap = ts;
    MatList traj;
    pList ptraj;
    traj.reserve(2 * n);
    ptraj.P.reserve(2 * n);

    //DEBUG_PRINT_VALUE(xs);
    //DEBUG_PRINT_VALUE(TransInv(xs));
    //DEBUG_PRINT_VALUE(TransInv(xs)*xend);
    //DEBUG_PRINT_VALUE(MatrixLog6(MathTool::TransInv(xs)*xend));
    Mat tmat = MathTool::MatrixLog6(MathTool::TransInv(xs)*xend);
    //DEBUG_PRINT_VALUE(tmat);
    for (double i = 0; i < n; i++)
    {
        double s;
        if (timescaling == enumTimeScaling::Cubic)
        {
            s = MathTool::CubicTimeScaling(tf, timegap*i);
        }
        else
            s = MathTool::QuinticTimeScaling(tf, timegap*i);

        Mat trajp = xstart * (xs * MathTool::MatrixExp6(tmat*s));
        //DEBUG_PRINT_VALUE(s);
        //DEBUG_PRINT_VALUE(tmat*s);
        //DEBUG_PRINT_VALUE(MatrixExp6(tmat*s));
        //DEBUG_PRINT_VALUE(xs * MathTool::MatrixExp6(tmat*s));
        //DEBUG_PRINT_VALUE(xstart * (xs * MathTool::MatrixExp6(tmat*s)));
        traj.push_back(trajp);
        VectorXd temp = MathTool::sE3ToPoseRPY(trajp);
        //DEBUG_PRINT_VALUE(temp);
        ptraj.add(temp);
    }

    for (int i = 0; i < n; i++)
    {
        traj.push_back(traj[n - i - 1]);
        ptraj.P.push_back(ptraj.P[n - i - 1]);
    }

    return ptraj;
}

TrajOnceOutput Stewart_GMT::pivotTrajOnce(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling, int iter)
{
    double n = ceill(tf / ts) + 1;

    Matrix4d xstart = MathTool::poseRPY2SE3(pstart);
    Matrix4d xs = MatrixXd::Identity(4, 4);
    Vector3d r = pivot;
    w = w / MathTool::norm(w);

    Matrix3d wx = MathTool::vect3tomatrix3(w);
    Matrix3d rx = MathTool::vect3tomatrix3(r);
    Matrix4d ms;
    ms << wx, rx*w, 0, 0, 0, 0;
    Matrix4d xend = MathTool::MatrixExp6(ms*theta);

    double timegap = ts;

    //DEBUG_PRINT_VALUE(xs);
    //DEBUG_PRINT_VALUE(TransInv(xs));
    //DEBUG_PRINT_VALUE(TransInv(xs)*xend);
    //DEBUG_PRINT_VALUE(MatrixLog6(MathTool::TransInv(xs)*xend));
    Mat tmat = MathTool::MatrixLog6(MathTool::TransInv(xs)*xend);
    //DEBUG_PRINT_VALUE(tmat);

    int titer = -1;
    if (iter <= 0)titer = 0;
    else if (iter < n) titer = iter;
    else if (iter >= n && iter < 2 * n) titer = 2 * n - iter - 1;
    else {
        DEBUG_PRINT_ERROR_MESSAGE("pivot once out of index")
        return TrajOnceOutput();
    }

        double s;
        if (timescaling == enumTimeScaling::Cubic)
        {
            s = MathTool::CubicTimeScaling(tf, timegap*titer);
        }
        else
            s = MathTool::QuinticTimeScaling(tf, timegap*titer);

        Mat trajp = xstart * (xs * MathTool::MatrixExp6(tmat*s));
        //DEBUG_PRINT_VALUE(s);
        //DEBUG_PRINT_VALUE(tmat*s);
        //DEBUG_PRINT_VALUE(MatrixExp6(tmat*s));
        //DEBUG_PRINT_VALUE(xs * MathTool::MatrixExp6(tmat*s));
        //DEBUG_PRINT_VALUE(xstart * (xs * MathTool::MatrixExp6(tmat*s)));
        VectorXd vtrajp = MathTool::sE3ToPoseRPY(trajp);
        //DEBUG_PRINT_VALUE(temp);


    return TrajOnceOutput(vtrajp, titer, n);
}

pList Stewart_GMT::pivotTraj(PivotInput input)
{
    return pivotTraj(
        input.Pstart,
        input.Ppivot,
        input.w,
        input.theta,
        input.tf,
        input.ts,
        input.tscale
    );
}

VectorXd Stewart_GMT::pivotLimit(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling)
{
    double n = tf / ts + 1;

    VectorXd Pose;
    Matrix4d xstart = MathTool::poseRPY2SE3(pstart);
    Matrix4d xs = MatrixXd::Identity(4, 4);
    Vector3d r = pivot;
    w = w / MathTool::norm(w);

    Matrix3d wx = MathTool::vect3tomatrix3(w);
    Matrix3d rx = MathTool::vect3tomatrix3(r);
    Matrix4d ms;
    ms << wx, rx* w, 0, 0, 0, 0;
    Matrix4d xend = MathTool::MatrixExp6(ms * theta);

    double timegap = ts;

    Mat tmat = MathTool::MatrixLog6(MathTool::TransInv(xs) * xend);
    double s;
    if (timescaling == enumTimeScaling::Cubic)
    {
        s = MathTool::CubicTimeScaling(tf, timegap * n);
    }
    else
        s = MathTool::QuinticTimeScaling(tf, timegap * n);
    Mat trajp = xstart * (xs * MathTool::MatrixExp6(tmat * s));
    Pose = MathTool::sE3ToPoseRPY(trajp);

    return Pose;
}


pList Stewart_GMT::pivotMotion(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling)
{
    Matrix3d w0;
    w0 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Matrix3d w = w0.adjoint();

    pList ptraj;
    for (int i = 0; i < 3; i++)
    {
        Vector3d ww = w.col(i);
        double theta1 = angle(i);
        ptraj.append(
            Stewart_GMT::pivotTraj(
                pstart, pivot,
                ww,	theta1,
                tf, ts, timescaling
            )
        );
        double theta2 = -angle(i);
        ptraj.append(
            Stewart_GMT::pivotTraj(
                pstart, pivot,
                ww, theta2,
                tf, ts, timescaling
            )
        );
    }

    return ptraj;
}

pList Stewart_GMT::pivotMotionN(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling)
{
    DEBUG_PRINT_MESSAGE("motion n input")
    DEBUG_PRINT_VALUE(pstart);
    DEBUG_PRINT_VALUE(pivot);
    DEBUG_PRINT_VALUE(angle);
    DEBUG_PRINT_VALUE(tf);
    DEBUG_PRINT_VALUE(ts);
    DEBUG_PRINT_VALUE(timescaling);

    Matrix3d w0;
    w0 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Matrix3d w = w0.adjoint();

    pList ptraj;
    for (int i = 0; i < 3; i++)
    {
        Vector3d ww = w.col(i);
        //double theta1 = angle(i);
        //ptraj.append(
        //	Stewart_GMT::pivotTraj(
        //		pstart, pivot,
        //		ww, theta1,
        //		tf, ts, timescaling
        //	)
        //);
        double theta2 = -angle(i);
        ptraj.append(
            Stewart_GMT::pivotTraj(
                pstart, pivot,
                ww, theta2,
                tf, ts, timescaling
            )
        );
    }

    return ptraj;
}

pList Stewart_GMT::pivotMotionP(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling)
{
    DEBUG_PRINT_MESSAGE("motion p input")
    DEBUG_PRINT_VALUE(pstart);
    DEBUG_PRINT_VALUE(pivot);
    DEBUG_PRINT_VALUE(angle);
    DEBUG_PRINT_VALUE(tf);
    DEBUG_PRINT_VALUE(ts);
    DEBUG_PRINT_VALUE(timescaling);

    Matrix3d w0;
    w0 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Matrix3d w = w0.adjoint();

    pList ptraj;
    for (int i = 0; i < 3; i++)
    {
        Vector3d ww = w.col(i);
        double theta1 = angle(i);
        ptraj.append(
            Stewart_GMT::pivotTraj(
                pstart, pivot,
                ww, theta1,
                tf, ts, timescaling
            )
        );
        //double theta2 = -angle(i);
        //ptraj.append(
        //	Stewart_GMT::pivotTraj(
        //		pstart, pivot,
        //		ww, theta2,
        //		tf, ts, timescaling
        //	)
        //);
    }

    return ptraj;
}

 pList Stewart_GMT::moveL_Stewart(VectorXd pstart6, VectorXd pend6, double tf, double ts, enumTimeScaling timescaling)
{
     DEBUG_PRINT_MESSAGE("moveL_Stewart input")
     DEBUG_PRINT_VALUE(pstart6);
     DEBUG_PRINT_VALUE(pend6);
     DEBUG_PRINT_VALUE(tf);
     DEBUG_PRINT_VALUE(ts);
     DEBUG_PRINT_VALUE(timescaling);

    Matrix4d xstart = MathTool::poseRPY2SE3(pstart6);
    Matrix4d xend = MathTool::poseRPY2SE3(pend6);


    double n = tf / ts + 1;

    MatList traj;
    pList ptraj;
    traj.reserve(n);
    ptraj.P.reserve(n);

    Matrix3d rstart = MathTool::t2r(xstart);
    Vector3d pstart = MathTool::t2p(xstart);
    //Matrix3d rend = rstart;
    Matrix3d rend = MathTool::t2r(xend);
    Vector3d pend = MathTool::t2p(xend);


    //DEBUG_PRINT_VALUE(xstart)
    //DEBUG_PRINT_VALUE(xend)
    //	DEBUG_PRINT_VALUE(rstart)
    //	DEBUG_PRINT_VALUE(rend)
    //	DEBUG_PRINT_VALUE(pstart)
    //	DEBUG_PRINT_VALUE(pend)
    //	DEBUG_PRINT_VALUE(n)


    Mat tmat = MathTool::MatrixLog3(rstart.adjoint()*rend);
    //DEBUG_PRINT_VALUE(tmat);
    for (double i = 0; i < n; i++)
    {
        double s;
        if (timescaling == enumTimeScaling::Cubic)
        {
            s = MathTool::CubicTimeScaling(tf, ts*i);
        }
        else
            s = MathTool::QuinticTimeScaling(tf, ts*i);
        //DEBUG_PRINT_VALUE(i)
        //DEBUG_PRINT_VALUE(s)

        Matrix3d ri = rstart * MathTool::MatrixExp3(tmat*s);
        Vector3d pi = pstart + s * (pend - pstart);
        Matrix4d traji = MathTool::rp2t(ri, pi);
        //DEBUG_PRINT_VALUE(traji);
        traj.push_back(traji);
        VectorXd posei = MathTool::sE3ToPoseRPY(traji);
        ptraj.add(posei);
    }


    return ptraj;
}

 pList Stewart_GMT::freeRun(VectorXd pstart, Vector3d angle, double tf, double ts, enumTimeScaling timescaling)
 {
     Vector3d pivot{ 0,0,0 };
     double dz = 5;
     VectorXd vdz(6);
     vdz << 0, 0, dz, 0, 0, 0;
     double dx = 15;
     VectorXd vdx(6);
     vdx << dx, 0, 0, 0, 0, 0;
     double dy = 15;
     VectorXd vdy(6);
     vdy << 0, dy, 0, 0, 0, 0;

     VectorXd pendz1 = pstart + vdz;
     VectorXd pendz2 = pstart - vdz;
     VectorXd pendy1 = pstart + vdy;
     VectorXd pendy2 = pstart - vdy;
     VectorXd pendx1 = pstart + vdx;
     VectorXd pendx2 = pstart - vdx;

     DEBUG_PRINT_MESSAGE("ptrajz01")
     pList ptrajz01 = moveL_Stewart(pstart, pendz1, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajz10")
         pList ptrajz10 = moveL_Stewart(pendz1, pstart, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajz02")
         pList ptrajz02 = moveL_Stewart(pstart, pendz2, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajz20")
         pList ptrajz20 = moveL_Stewart(pendz2, pstart, tf, ts, timescaling);

     DEBUG_PRINT_MESSAGE("ptrajx01")
         pList ptrajx01 = moveL_Stewart(pstart, pendx1, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajx10")
         pList ptrajx10 = moveL_Stewart(pendx1, pstart, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajx02")
         pList ptrajx02 = moveL_Stewart(pstart, pendx2, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajx20")
         pList ptrajx20 = moveL_Stewart(pendx2, pstart, tf, ts, timescaling);

     DEBUG_PRINT_MESSAGE("ptrajy01")
         pList ptrajy01 = moveL_Stewart(pstart, pendy1, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajy10")
         pList ptrajy10 = moveL_Stewart(pendy1, pstart, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajy02")
         pList ptrajy02 = moveL_Stewart(pstart, pendy2, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajy20")
         pList ptrajy20 = moveL_Stewart(pendy2, pstart, tf, ts, timescaling);

     DEBUG_PRINT_MESSAGE("ptrajrp")
         pList ptrajrp = pivotMotionP(pstart, pivot, angle, tf, ts, timescaling);
     DEBUG_PRINT_MESSAGE("ptrajrn")
         pList ptrajrn = pivotMotionN(pstart, pivot, angle, tf, ts, timescaling);

     pList ptraj;
     ptraj.append(ptrajz01);
     ptraj.append(ptrajz10);
     ptraj.append(ptrajx01);
     ptraj.append(ptrajx10);
     ptraj.append(ptrajy01);
     ptraj.append(ptrajy10);
     ptraj.append(ptrajrp);

     ptraj.append(ptrajz02);
     ptraj.append(ptrajz20);
     ptraj.append(ptrajx02);
     ptraj.append(ptrajx20);
     ptraj.append(ptrajy02);
     ptraj.append(ptrajy20);
     ptraj.append(ptrajrn);

     return ptraj;
 }

 //????
 pList Stewart_GMT::SpiralTrack_v1(VectorXd Pstart, double t, double Ts, double b, double N)
 {
     b = b / 6.283;
     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double Lupp = 181;
     double Llow = 164;
     double Tf = N * 2 * MathPI;

     int K = ceil(Tf / (Ts*(1 / t))) + 1;

     vector<double> X;
     vector<double> Y;

     pList traj;
     for (int i = 0; i < K; i++)
     {
         X.push_back(
             ax + (b * i*Ts*(1 / t))*cos(i*Ts*(1 / t))
         );
         Y.push_back(
             ay + (b * i*Ts*(1 / t))*sin(i*Ts*(1 / t))
         );
         VectorXd P(6);
         P << X[i], Y[i], az, Pstart(3), Pstart(4), Pstart(5);

         VectorXd LL = F_Stewart_IK(P, _pp, _bb);

         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             DEBUG_PRINT_VALUE(i);
             DEBUG_PRINT_VALUE(P);
             DEBUG_PRINT_VALUE(LL);
             DEBUG_PRINT_VALUE(illegal_lenth);
             break;
         }

         traj.add(P);
     }

     return traj;
 }


 pList Stewart_GMT::N_ToolSpiralTrack(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal)
 {
     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     Vector3d PMtool;
     PMtool << TMtool(0, 3), TMtool(1, 3), TMtool(2, 3);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTranslx = toolTrans.col(0);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);


     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double Lupp = _Lupp;
     double Llow = _Llow;

     double h0 = _h0;
     double g0 = _g0;

     MatrixXd bb = _bb;
     MatrixXd pp = _pp;

     double b = lineSpacing / (2 * MathPI);
     double v = velocity/100;
     double tf = 2 * MathPI*scanRange / lineSpacing;
     int K = ceil(tf / 0.0001) + 1;

     Vector3d X;
     Vector3d Y;
     Vector3d Z;

     pList traj;
     for (int k = 1; k <= K; k++)
     {
         double x0 = 0.1;
         SpiralTEvaluateParameter sp2;
         sp2.setsp2(b, k, v);
         double t = newton(x0, 1e-5, 50, &spiral_time_eval_f2, &sp2);
         t = min(t, ts);
         X = (b * (k-1 )*t)*cos((k-1 )*t)*toolTranslx;
         Y = (b * (k-1 )*t)*sin((k-1 )*t)*toolTransly;
         Z = 0 * toolTranslz;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + X + Y + Z;
         //DEBUG_PRINT_VALUE(t)
         //DEBUG_PRINT_VALUE(X)
         //DEBUG_PRINT_VALUE(Y)
         //DEBUG_PRINT_VALUE(Z)
         //DEBUG_PRINT_VALUE(trans)
         Vector3d tooltrans = PMtool + trans;
         VectorXd traji(6);
         traji << trans, Pstart.segment(3, 3);
         //DEBUG_PRINT_VALUE(traji)

         if (illegal_lenth(traji, TypeN))
         {
             if (traj.getSize() == 0)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
                     DEBUG_PRINT_VALUE(Tstart)
                     DEBUG_PRINT_VALUE(TMtool)
                     DEBUG_PRINT_VALUE(v)
                     DEBUG_PRINT_VALUE(scanRange)

                 DEBUG_PRINT_VALUE(traji)
             }
             break;
         }

         traj.add(traji);

     }
     if (traj.getSize() == 0) return traj;

     VectorXd Pend = Pstart;
     Pstart = stdVectortoVectorXD(
         traj.end()
     );

     pList traj2 = N_moveL_PP(Pstart, Pend, tscal, velocity, ts);

     traj.append(traj2);

     //auto Xv = traj.getRow(0);
     //double xmin = *min_element(Xv.begin(), Xv.end());
     //double xmax = *max_element(Xv.begin(), Xv.end());
     //auto Yv = traj.getRow(1);
     //double ymin = *min_element(Yv.begin(), Yv.end());
     //double ymax = *max_element(Yv.begin(), Yv.end());
     //auto Zv = traj.getRow(2);
     //double zmin = *min_element(Zv.begin(), Zv.end());
     //double zmax = *max_element(Zv.begin(), Zv.end());
     //VectorXd xyz(6);
     //xyz << xmin, xmax, ymin, ymax, zmin, zmax;

     return traj;
 }

 //TODO:??moveL????
 TrajOnceOutput Stewart_GMT::N_ToolSpiralTrackOnce(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, int iter, int stage, Vector2d r, Vector2d u, double thd0, double thd1)
 {
     static double ldataM_ = 0;
     static WeightAvarage avg_;
     static VectorXd scan_end_;//??sin???,??????moveL????
     static VectorXd target_after_scan_;//??sin???,??????moveL????
     static VectorXd trajM_ = VectorXd(6);//
     static double message_code_;
     static Vector3d r3;

     if (iter == 1 && stage == 1)
     {
         ldataM_ = 0;
         avg_.reset();
         scan_end_ = VectorXd(6);
         target_after_scan_ = VectorXd(6);
         trajM_ = VectorXd(6);
         message_code_ = 3;
         r3 << 0, r;
     }

     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     Vector3d PMtool;
     PMtool << TMtool(0, 3), TMtool(1, 3), TMtool(2, 3);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTranslx = toolTrans.col(0);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);


     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);



     int current_stage = stage;
     int current_iter = iter;

     TrajOnceOutput spiralout;
     //?????:spiral
     if (current_stage == 1)
     {
         double b = lineSpacing / (2 * MathPI);
         double v = velocity / 100;
         double tf = 2 * MathPI*scanRange / lineSpacing;
         int K = ceil(tf / 0.0001) + 1;

         Vector3d X;
         Vector3d Y;
         Vector3d Z;

         int k = iter;
         if (k > K || k==0) return TrajOnceOutput("out of range");

         double x0 = 0.1;
         SpiralTEvaluateParameter sp2;
         sp2.setsp2(b, k, v);
         double t = newton(x0, 1e-5, 50, &spiral_time_eval_f2, &sp2);
         t = min(t, ts);
         X = (b * (k - 1)*t)*cos((k - 1)*t)*toolTranslx;
         Y = (b * (k - 1)*t)*sin((k - 1)*t)*toolTransly;
         Z = 0 * toolTranslz;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + X + Y + Z;
         //DEBUG_PRINT_VALUE(t)
         //DEBUG_PRINT_VALUE(X)
         //DEBUG_PRINT_VALUE(Y)
         //DEBUG_PRINT_VALUE(Z)
         //DEBUG_PRINT_VALUE(trans)
         Vector3d tooltrans = PMtool + trans;
         VectorXd traji(6);
         traji << trans, Pstart.segment(3, 3);
         //DEBUG_PRINT_VALUE(traji)

         if (illegal_lenth(traji, TypeN))
         {
             if (k == 1)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
                     DEBUG_PRINT_VALUE(Tstart)
                     DEBUG_PRINT_VALUE(TMtool)
                     DEBUG_PRINT_VALUE(v)
                     DEBUG_PRINT_VALUE(scanRange)

                     DEBUG_PRINT_VALUE(traji)


                     return TrajOnceOutput("illeagle input");
             }

             current_stage = 2;
             current_iter = 1;
         }
         else
         {
             scan_end_ = traji;
             spiralout = TrajOnceOutput(traji, current_iter + 1, current_stage);
         }
     }

    //??????:spiral????
    if (current_stage == 2)
    {
        VectorXd Pend;
        if (message_code_ == 3) Pend = Pstart;
        else Pend = target_after_scan_;
        Pstart = scan_end_;
        spiralout = N_moveL_PPOnce(Pstart, Pend, tscal, velocity, ts, current_iter,r,u);

        if (spiralout.error || spiralout.is_finished)
        {
            return spiralout;
        }
    }


    //************??????***********************
    VectorXd trajp = spiralout.trajp;

    double y = trajp(1);
    double z = trajp(2);

    double ldata = Stewart_GMT::GetPowerMeterData_Wang(y, z, r, u);
    //else ldata = _Ldata;

    if (ldata > ldataM_)
    {
        ldataM_ = ldata;
        trajM_ = trajp;

        if (ldata > thd1)
        {
            target_after_scan_ = trajp;
            message_code_ = 1;
            DEBUG_PRINT_MESSAGE("FA max found")
        }
    }

    if (ldata > thd0)
    {
        avg_.update(ldata, trajp);

        if (message_code_ > 1)
        {
            message_code_ = 2;
            target_after_scan_ = avg_.preAve;
        }
    }

    Vector3d p3 = trajp.segment(0, 3);

    spiralout.next_stage = current_stage;
    spiralout.Ldata = ldata;
    spiralout.LdataM = ldataM_;
    spiralout.trajM = trajM_;
    spiralout.normP2R = (p3 - r3).norm();
    spiralout.message_code = message_code_;

    return spiralout;

 }

 FADataPinOutput Stewart_GMT::N_FA_SpiralTrack_DataPin(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1)
 {

     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     Vector3d PMtool;
     PMtool << TMtool(0, 3), TMtool(1, 3), TMtool(2, 3);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTranslx = toolTrans.col(0);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);


     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double Lupp = _Lupp;
     double Llow = _Llow;

     double h0 = _h0;
     double g0 = _g0;

     MatrixXd bb = _bb;
     MatrixXd pp = _pp;

     double b = lineSpacing / (2 * MathPI);
     double v = velocity / 1000;
     double tf = 2 * MathPI*scanRange / lineSpacing;
     int K = ceil(tf / 0.0001) + 1;

     Vector3d X;
     Vector3d Y;
     Vector3d Z;
     double y1 = remain12[WS_LIMIT_Y_MINUS];
     double y2 = remain12[WS_LIMIT_Y_PLUS];
     double z1 = remain12[WS_LIMIT_Z_MINUS];
     double z2 = remain12[WS_LIMIT_Z_PLUS];

     pList traj;
     vector<double> tldata;
     for (int k = 1; k <= K; k++)
     {
         double x0 = 0.1;
         SpiralTEvaluateParameter sp2;
         sp2.setsp2(b, k, v);
         double t = newton(x0, 1e-5, 50, &spiral_time_eval_f2, &sp2);
         X = (b * (k - 1)*t)*cos((k - 1)*t)*toolTranslx;
         Y = (b * (k - 1)*t)*sin((k - 1)*t)*toolTransly;
         Z = 0 * toolTranslz;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + X + Y + Z;
         //DEBUG_PRINT_VALUE(t)
         //DEBUG_PRINT_VALUE(X)
         //DEBUG_PRINT_VALUE(Y)
         //DEBUG_PRINT_VALUE(Z)
         //DEBUG_PRINT_VALUE(trans)
         Vector3d tooltrans = PMtool + trans;
         VectorXd traji(6);
         traji << trans, Pstart.segment(3, 3);
         //DEBUG_PRINT_VALUE(traji)

         if (illegal_lenth(traji, TypeN))
         {
             if (traj.getSize() == 0)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
                     DEBUG_PRINT_VALUE(Tstart)
                     DEBUG_PRINT_VALUE(v)
                     DEBUG_PRINT_VALUE(scanRange)

                     DEBUG_PRINT_VALUE(traji)
             }
             break;
         }

         traj.add(traji);

         double yy = traji(1);
         double zz = traji(2);
         double Ldatai = Stewart_GMT::GetPowerMeterData_Wang(yy, zz, r, u);
         tldata.push_back(Ldatai);

     }
     FADataPinOutput out = FADataPinOutput(traj, stdVectortoVectorXD(tldata));
     if (traj.getSize() == 0) return FADataPinOutput("traj size 0");


     auto fthd0 = out.Ldata.array() >= thd0;
     auto fthd1 = out.Ldata.array() >= thd1;
     bool flag0 = fthd0.any();
     bool flag1 = fthd1.any();

     VectorXd Pend;
     if (flag1)
     {
         VectorXd::Index maxindex;
         out.LdataM = out.Ldata.maxCoeff(&maxindex);
         out.trajM = out.traj.get((int)maxindex);
         Pend = stdVectortoVectorXD(out.trajM);
         out.message_code = 1;
     }
     else if (flag0)
     {
         //vector<double> tldat;
         //vector<double> tid0;

         double ldataM_ = 0;
         vector<double> trajM_;
         WeightAvarage avg;
         for (int i = 0; i < out.Ldata.rows(); i++)
         {
             double ldata = out.Ldata(i);

             if (ldata > ldataM_)
             {
                 ldataM_ = ldata;
                 trajM_ = out.traj.get(i);
             }

             if (fthd0(i))
             {

                 avg.update(ldata, stdVectortoVectorXD(trajM_));
                 //tldat.push_back(out.Ldata(i));
                 //tid0.push_back(i);
             }
         }
         // VectorXd ldat = MathTool::stdVectortoVectorXD(tldat);
         // VectorXd id0 = MathTool::stdVectortoVectorXD(tid0);
         // //DEBUG_PRINT_VALUE(ldat)
         ////	 DEBUG_PRINT_VALUE(id0)
         //double weit = ldat.sum();
         // VectorXd idw = ldat.adjoint()*id0 / weit;
         // VectorXd ids = (id0.array() - idw(0)).abs();
         // //DEBUG_PRINT_VALUE(idw)
         ////	 DEBUG_PRINT_VALUE(ids)
         //VectorXd::Index idm;
         //ids.minCoeff(&idm);
         out.LdataM = ldataM_;
         out.trajM = trajM_;
         Pend = avg.preAve;
         out.message_code = 2;
     }
     else
     {
         Pend = Pstart;
         out.message_code = 3;
     }

     Pstart = stdVectortoVectorXD(
         out.traj.end()
     );

     //DEBUG_PRINT_VALUE(Pend)
        // DEBUG_PRINT_VALUE(Pstart)
     Tstart = poseRPY2SE3(Pstart);

     FADataPinOutput out2 = N_FA_moveL_PP_Data(TMtool, Tstart, Pend, tscal, velocity, ts, remain12, r, u);

     out.append(out2);

     //auto Xv = traj.getRow(0);
     //double xmin = *min_element(Xv.begin(), Xv.end());
     //double xmax = *max_element(Xv.begin(), Xv.end());
     //auto Yv = traj.getRow(1);
     //double ymin = *min_element(Yv.begin(), Yv.end());
     //double ymax = *max_element(Yv.begin(), Yv.end());
     //auto Zv = traj.getRow(2);
     //double zmin = *min_element(Zv.begin(), Zv.end());
     //double zmax = *max_element(Zv.begin(), Zv.end());
     //VectorXd xyz(6);
     //xyz << xmin, xmax, ymin, ymax, zmin, zmax;
     Vector3d r3;
     r3 << 0, r;
     Vector3d p3 = stdVectortoVectorXD(out.traj.end()).segment(0, 3);
     out.normP2R = norm(p3 - r3);

     return out;
 }

 //TODO:??????
 //TODO:???????
 pList Stewart_GMT::N_ToolSinTrack(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12)
 {

     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     //Matrix3d ry = rotY(alpha);
     //Matrix3d rx = rotX(beta);
     //Vector3d PMtool = pivot;
     //Matrix3d RMtool = ry * rx;
     //Matrix4d TMtool = rp2t(RMtool, PMtool);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTranslx = toolTrans.col(0);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);


     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double Lupp = _Lupp;
     double Llow = _Llow;

     double h0 = _h0;
     double g0 = _g0;

     MatrixXd bb = _bb;
     MatrixXd pp = _pp;

     double step_range = stepRange * 0.5;
     double scan_range = scanRange * 0.5;
     double tf = (step_range * 2 * MathPI) / (2 * lineSpacing);
     double t = 1 / velocity;

     int K = ceil(tf / (ts*1/t));
     int M = 2 * K + 1;


     //DEBUG_PRINT_VALUE(remain12)
     double stepLimit = min(abs(remain12[WS_LIMIT_X_PLUS]), abs(remain12[WS_LIMIT_X_MINUS]));
     double scanLimit = min(abs(remain12[WS_LIMIT_Y_PLUS]), abs(remain12[WS_LIMIT_Y_MINUS]));
     double dz = min(abs(remain12[WS_LIMIT_Z_PLUS]), abs(remain12[WS_LIMIT_Z_MINUS]));

     //double stepLimit;
     Vector3d translv;
     Vector3d translh;
     if (scanMode == horizontal || scanMode == Both)//TODO:??both???
     {
         translv = toolTransly;
         translh = toolTranslx;
     }
     else if (scanMode == vertical)
     {
         translv = toolTranslx;
         translh = toolTransly;
     }
     //DEBUG_PRINT_CODE_LOCATION
        // DEBUG_PRINT_VALUE(toolTransly)
        // DEBUG_PRINT_VALUE(toolTranslx)
        // DEBUG_PRINT_VALUE(translv)
        // DEBUG_PRINT_VALUE(translh)

    double scan = min(abs(scan_range), scanLimit);
     double step = min(abs(step_range), stepLimit);

     VectorXd Pend;
     while (true)
     {
         Vector3d X1 = step * (-1)*translh;
         Vector3d Y1 = scan * translv;
         Vector3d Z1 = 0 * toolTranslz;
         //DEBUG_PRINT_VALUE(X1)
            // DEBUG_PRINT_VALUE(Y1)
            // DEBUG_PRINT_VALUE(Z1)

         Vector3d trans1;
         trans1 << ax, ay, az;
         trans1 = trans1 + X1 + Y1 + Z1;
         VectorXd trajp1(6);
         trajp1 << trans1, Pstart.segment(3, 3);
         Pend = trajp1;

         if (!illegal_lenth(Pend, TypeN)) break;

         step *= 0.75;
         scan *= 0.75;
     }


     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(trans1)
     //DEBUG_PRINT_VALUE(Pstart)
     //DEBUG_PRINT_VALUE(Pend)
     pList traj0 = N_moveL_PP(Pstart, Pend, tscal, velocity, ts);
     //traj0.show();
     if (traj0.getSize() == 0) return traj0;

     Vector3d X;
     Vector3d Y;
     Vector3d Z;

     pList traj;
     Vector3d pTMtool = t2p(TMtool);
     for (double k = 1; k <= M; k++)
     {
         X = step * ((k - 1)/K-1)*translh;
         Y = scan * cos((k - 1)*ts*(1.0f / t))*translv;
         Z = 0.0*toolTranslz;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + X + Y + Z;
         //DEBUG_PRINT_VALUE(t)
         //DEBUG_PRINT_VALUE(X)
         //DEBUG_PRINT_VALUE(Y)
         //DEBUG_PRINT_VALUE(Z)
         //DEBUG_PRINT_VALUE(trans)
         Vector3d tooltrans = pTMtool + trans;
         VectorXd traji(6);
         traji << trans, Pstart.segment(3, 3);
         //DEBUG_PRINT_VALUE(traji)

         if (illegal_lenth(traji, TypeN))
         {
             if (traj.getSize() == 0)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
                     DEBUG_PRINT_VALUE(Tstart)
                     DEBUG_PRINT_VALUE(scanRange)

                     DEBUG_PRINT_VALUE(traji)
             }
             break;
         }

         traj.add(traji);

     }

     Pend = Pstart;
     //auto sinEnd = traj.end();
     Pstart =  stdVectortoVectorXD(
             traj.end()
         );

     //DEBUG_PRINT_VALUE(Pend)
        // DEBUG_PRINT_VALUE(Pstart)
     pList traj2 = N_moveL_PP(Pstart, Pend, tscal, velocity, ts);
     //traj2.show();

     pList output;
     output.append(traj0);
     output.append(traj);
     output.append(traj2);

     //auto Xv = traj.getRow(0);
     //double xmin = *min_element(Xv.begin(), Xv.end());
     //double xmax = *max_element(Xv.begin(), Xv.end());
     //auto Yv = traj.getRow(1);
     //double ymin = *min_element(Yv.begin(), Yv.end());
     //double ymax = *max_element(Yv.begin(), Yv.end());
     //auto Zv = traj.getRow(2);
     //double zmin = *min_element(Zv.begin(), Zv.end());
     //double zmax = *max_element(Zv.begin(), Zv.end());
     //VectorXd xyz(6);
     //xyz << xmin, xmax, ymin, ymax, zmin, zmax;

     return output;
 }

 //TODO:?????
 TrajOnceOutput Stewart_GMT::N_ToolSinTrackOnece(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, int stage, int iter, Vector2d r, Vector2d u, double thd0, double thd1)
 {
     static double ldataM_;
     static WeightAvarage avg_;
     static VectorXd scan_end_;//??sin???,??????moveL????
     static VectorXd target_after_scan_;//??sin???,??????moveL????
     static VectorXd trajM_ = VectorXd(6);//
     static double message_code_;
     static Vector3d r3;

     if (iter == 1 && stage == 1)
     {
         ldataM_ = 0;
         avg_.reset();
         scan_end_ = VectorXd(6);
         target_after_scan_ = VectorXd(6);
         trajM_ = VectorXd(6);
         message_code_ = 3;
         r3 << 0, r;
     }


     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     //Matrix3d ry = rotY(alpha);
     //Matrix3d rx = rotX(beta);
     //Vector3d PMtool = pivot;
     //Matrix3d RMtool = ry * rx;
     //Matrix4d TMtool = rp2t(RMtool, PMtool);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTranslx = toolTrans.col(0);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);


     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double step_range = stepRange;// *0.5;
     double scan_range = scanRange;// *0.5;
     double tf = (step_range * 2 * MathPI) / (2 * lineSpacing);
     double t = 1 / velocity;

     int K = ceil(tf / (ts * 1 / t));
     int M = 2 * K + 1;

     Vector3d translv;
     Vector3d translh;
     if (scanMode == horizontal || scanMode == Both)//TODO:??both???
     {
         translv = toolTransly;
         translh = toolTranslx;
     }
     else if (scanMode == vertical)
     {
         translv = toolTranslx;
         translh = toolTransly;
     }

     Vector3d X1 = step_range * (-1)*translh;
     Vector3d Y1 = scan_range * translv;
     Vector3d Z1 = 0 * toolTranslz;

     Vector3d trans1;
     trans1 << ax, ay, az;
     trans1 = trans1 + X1 + Y1 + Z1;
     VectorXd trajp1(6);
     trajp1 << trans1, Pstart.segment(3, 3);
     VectorXd Pend = trajp1;

     //DEBUG_PRINT_CODE_LOCATION
         //DEBUG_PRINT_VALUE(trans1)
         //DEBUG_PRINT_VALUE(Pstart)
        // DEBUG_PRINT_VALUE(Pend)

     int current_stage = stage;
     int current_iter = iter;

     TrajOnceOutput sinout;
     //?????:Tstart?sin??
    if (current_stage == 1)
    {
        sinout = N_moveL_PPOnce(Pstart, Pend, tscal, velocity, ts, current_iter,r,u);

        if (sinout.error)
        {
            return sinout;
        }

        if(sinout.is_finished)
        {
            //DEBUG_PRINT_VALUE(Pstart)
            //	DEBUG_PRINT_VALUE(Pend)
            //	DEBUG_PRINT_VALUE(current_iter)
            current_stage = 2;
            current_iter = 1;
        }
    }

    //?????:sin??
    //TODO:?????
    if (current_stage == 2)
    {
        Vector3d X;
        Vector3d Y;
        Vector3d Z;

        Vector3d pTMtool = t2p(TMtool);
        double k = current_iter;
        if (k <= M)
        {
            X = step_range * ((k - 1) / K - 1)*translh;
            Y = scan_range * cos((k - 1)*ts*(1.0f / t))*translv;
            Z = 0.0*toolTranslz;

            Vector3d trans;
            trans << ax, ay, az;
            trans = trans + X + Y + Z;
            //DEBUG_PRINT_VALUE(t)
            //DEBUG_PRINT_VALUE(X)
            //DEBUG_PRINT_VALUE(Y)
            //DEBUG_PRINT_VALUE(Z)
            //DEBUG_PRINT_VALUE(trans)
            //Vector3d tooltrans = pTMtool + trans;
            VectorXd traji(6);
            traji << trans, Pstart.segment(3, 3);
            //DEBUG_PRINT_VALUE(traji)

            if (illegal_lenth(traji, TypeN))
            {
                DEBUG_PRINT_VALUE(((k - 1) / K - 1))
                    DEBUG_PRINT_VALUE(((k - 1) / K - 1)*translh)
                    DEBUG_PRINT_VALUE(step_range * ((k - 1) / K - 1)*translh)
                    DEBUG_PRINT_VALUE(X)
                    DEBUG_PRINT_VALUE(Y)
                    DEBUG_PRINT_VALUE(ay)
                DEBUG_PRINT_VALUE(k)
                    DEBUG_PRINT_VALUE(K)
                    DEBUG_PRINT_VALUE(step_range)
                    DEBUG_PRINT_VALUE(scan_range)
                    DEBUG_PRINT_VALUE(translh)
                    DEBUG_PRINT_VALUE(translv)
                DEBUG_PRINT_VALUE(traji)
                    DEBUG_PRINT_VALUE(illegal_lenth(traji, TypeN))
                return TrajOnceOutput("lenth error");
            }
            else
            {
                scan_end_ = traji;
                sinout = TrajOnceOutput(traji, current_iter + 1, current_stage);
            }
        }
        else
        {
            //DEBUG_PRINT_VALUE(current_iter)
            //	DEBUG_PRINT_VALUE(k)
            //	DEBUG_PRINT_VALUE(M)

            //	DEBUG_PRINT_VALUE(t)
            //	DEBUG_PRINT_VALUE(velocity)
            //	DEBUG_PRINT_VALUE(tf)
            //	DEBUG_PRINT_VALUE(ts)
            //	DEBUG_PRINT_VALUE(M)
            current_stage = 3;
            current_iter = 1;
        }

    }

    //??????:sin??????
    if (current_stage == 3)
    {
        if (message_code_ == 3) Pend = Pstart;
        else Pend = target_after_scan_;
        Pstart = scan_end_;
        sinout = N_moveL_PPOnce(Pstart, Pend, tscal, velocity, ts, current_iter, r, u);

        if (sinout.error || sinout.is_finished)
        {
            //DEBUG_PRINT_VALUE(current_iter)
            return sinout;
        }
    }


    //************??????***********************
    VectorXd trajp = sinout.trajp;

    double y = trajp(1);
    double z = trajp(2);

    double ldata = Stewart_GMT::GetPowerMeterData_Wang(y, z, r, u);
    //else ldata = _Ldata;

    if (ldata > ldataM_)
    {
        ldataM_ = ldata;
        trajM_ = trajp;

        if (ldata > thd1)
        {
            target_after_scan_ = trajp;
            message_code_ = 1;
            DEBUG_PRINT_MESSAGE("FA max found")
                DEBUG_PRINT_VALUE(ldata)
                DEBUG_PRINT_VALUE(current_stage)
                DEBUG_PRINT_VALUE(current_iter)
        }
    }

    if (ldata > thd0)
    {
        avg_.update(ldata, trajp);

        if (message_code_ > 1)
        {
            message_code_ = 2;
            target_after_scan_ = avg_.preAve;
        }
    }

    Vector3d p3 = trajp.segment(0, 3);

    sinout.next_stage = current_stage;
    sinout.Ldata = ldata;
    sinout.LdataM = ldataM_;
    sinout.trajM = trajM_;
    sinout.normP2R = (p3 - r3).norm();
    sinout.message_code = message_code_;

    return sinout;
 }

 FADataPinOutput Stewart_GMT::N_FA_SinTrack_DataPin(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1)
 {

     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     //Matrix3d ry = rotY(alpha);
     //Matrix3d rx = rotX(beta);
     //Vector3d PMtool = pivot;
     //Matrix3d RMtool = ry * rx;
     //Matrix4d TMtool = rp2t(RMtool, PMtool);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTranslx = toolTrans.col(0);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);


     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double step_range = stepRange;// *0.5;
     double scan_range = scanRange;// *0.5;


     //DEBUG_PRINT_VALUE(remain12)
     double scanLimit;
     double stepLimit;
     double dz = min(abs(remain12[WS_LIMIT_Z_PLUS]), abs(remain12[WS_LIMIT_Z_MINUS]));
     Vector3d translv;
     Vector3d translh;
     if (scanMode == horizontal || scanMode == Both)//TODO:??both???
     {
         translv = toolTransly;
         translh = toolTranslx;
         scanLimit = min(abs(remain12[WS_LIMIT_Y_PLUS]), abs(remain12[WS_LIMIT_Y_MINUS]));
         stepLimit = min(abs(remain12[WS_LIMIT_X_PLUS]), abs(remain12[WS_LIMIT_X_MINUS]));
     }
     else if (scanMode == vertical)
     {
         translv = toolTranslx;
         translh = toolTransly;
         scanLimit = min(abs(remain12[WS_LIMIT_X_PLUS]), abs(remain12[WS_LIMIT_X_MINUS]));
         stepLimit = min(abs(remain12[WS_LIMIT_Y_PLUS]), abs(remain12[WS_LIMIT_Y_MINUS]));
     }
     //DEBUG_PRINT_CODE_LOCATION
        // DEBUG_PRINT_VALUE(toolTransly)
        // DEBUG_PRINT_VALUE(toolTranslx)
        // DEBUG_PRINT_VALUE(translv)
        // DEBUG_PRINT_VALUE(translh)

     double scan = min(abs(scan_range), scanLimit);
     double step = min(abs(step_range), stepLimit);
      //DEBUG_PRINT_VALUE(scan)
      //DEBUG_PRINT_VALUE(step)

     double y1 = -step;
     double y2 = step;
     double z1 = -scan;
     double z2 = scan;

     VectorXd Pend;
     while (true)
     {
         Vector3d X1 = step * (-1)*translh;
         Vector3d Y1 = scan * translv;
         Vector3d Z1 = 0 * toolTranslz;
         //DEBUG_PRINT_VALUE(X1)
            // DEBUG_PRINT_VALUE(Y1)
            // DEBUG_PRINT_VALUE(Z1)

         Vector3d trans1;
         trans1 << ax, ay, az;
         //DEBUG_PRINT_VALUE(trans1)
            // DEBUG_PRINT_VALUE(X1)
            // DEBUG_PRINT_VALUE(Y1)
         trans1 = trans1 + X1 + Y1 + Z1;
         VectorXd trajp1(6);
         trajp1 << trans1, Pstart.segment(3, 3);
         Pend = trajp1;

         //DEBUG_PRINT_VALUE(Pend)
         if (!illegal_lenth(Pend, TypeN)) break;

         step *= 0.75;
         scan *= 0.75;
     }

     //NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(Pend);
     //Mat tarr = nik_result.alpha.array().square() + SQUARE(_h0) - nik_result.ls.array().square();
     //VectorXd LL = nik_result.alpha.array() - tarr.array().sqrt();
     ////DEBUG_PRINT_VALUE(LL)


     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(trans1)
     //DEBUG_PRINT_VALUE(TMtool)
     //DEBUG_PRINT_VALUE(Tstart)
     FADataPinOutput out = N_FA_moveL_PP_Data(TMtool, Tstart, Pend, tscal, velocity, ts, remain12, r, u);
     out.step = step;
     out.scan = scan;
     //traj0.show();
        // DEBUG_PRINT_VALUE(Pend)
        // DEBUG_PRINT_VALUE(remain12)
     //DEBUG_PRINT_VALUE(out.traj.getSize())
     if (out.error) return out;


     //DEBUG_PRINT_VALUE(out.traj.getSize())

     Vector3d X;
     Vector3d Y;
     Vector3d Z;

     double tf = (step * 2 * MathPI) / (2 * lineSpacing);
     double t = 1 / velocity;

     int K = ceil(tf / (ts * 1 / t));
     int M = 2 * K + 1;

     //DEBUG_PRINT_VALUE(Pstart)
        // DEBUG_PRINT_VALUE(Pend)
        // DEBUG_PRINT_VALUE(step)
        // DEBUG_PRINT_VALUE(scan)
        // DEBUG_PRINT_VALUE(translh)
        // DEBUG_PRINT_VALUE(translv)
        // DEBUG_PRINT_VALUE(K)

     pList traj;
     vector<double> Ldata;
     Vector3d pTMtool = t2p(TMtool);
     for (double k = 1; k <= M; k++)
     {
         X = step * ((k - 1) / K - 1)*translh;
         Y = scan * cos((k - 1)*ts*(1.0f / t))*translv;
         Z = 0.0*toolTranslz;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + X + Y + Z;
         //DEBUG_PRINT_VALUE(t)
         //DEBUG_PRINT_VALUE(X)
         //DEBUG_PRINT_VALUE(Y)
         //DEBUG_PRINT_VALUE(Z)
         //DEBUG_PRINT_VALUE(trans)
         Vector3d tooltrans = pTMtool + trans;
         VectorXd traji(6);
         traji << trans, Pstart.segment(3, 3);
         //DEBUG_PRINT_VALUE(traji)

         if (illegal_lenth(traji, TypeN))
         {
             if (traj.getSize() == 0)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
             }
             else
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle pose")
             }

             DEBUG_PRINT_VALUE(step)
                 DEBUG_PRINT_VALUE(scan)
                 DEBUG_PRINT_VALUE(translh)
                 DEBUG_PRINT_VALUE(translv)
                 DEBUG_PRINT_VALUE(k)
                 DEBUG_PRINT_VALUE(K)
                 DEBUG_PRINT_VALUE(traji)

             break;
         }

         traj.add(traji);

         double yy = traji(1);
         double zz = traji(2);
         double Ldatai = Stewart_GMT::GetPowerMeterData_Wang(yy, zz, r, u);
         Ldata.push_back(Ldatai);
         //Ldata.conservativeResize(k);
         //out.Ldata[k - 1] = Ldatai;
     }


     //DEBUG_PRINT_VALUE(traj.getSize())

     //DEBUG_PRINT_VALUE(out.traj.getSize())
     out.append(traj, stdVectortoVectorXD(Ldata));

     auto fthd0 = out.Ldata.array() >= thd0;
     auto fthd1 = out.Ldata.array() >= thd1;
     bool flag0 = fthd0.any();
     bool flag1 = fthd1.any();

     if(flag1)
     {
         VectorXd::Index maxindex;
         out.LdataM = out.Ldata.maxCoeff(&maxindex);
         out.trajM = out.traj.get((int)maxindex);
         Pend = stdVectortoVectorXD(out.trajM);
         out.message_code = 1;
     }
     else if (flag0)
     {
         //vector<double> tldat;
         //vector<double> tid0;

         double ldataM_ = 0;
         vector<double> trajM_;
         WeightAvarage avg;
         for (int i = 0; i < out.Ldata.rows(); i++)
         {

            double ldata = out.Ldata(i);

            if (ldata > ldataM_)
            {
                ldataM_ = ldata;
                trajM_ = out.traj.get(i);
            }

            if (fthd0(i))
            {

                avg.update(ldata, stdVectortoVectorXD(trajM_));
                //tldat.push_back(out.Ldata(i));
                //tid0.push_back(i);
            }
         }
             // VectorXd ldat = MathTool::stdVectortoVectorXD(tldat);
             // VectorXd id0 = MathTool::stdVectortoVectorXD(tid0);
             // //DEBUG_PRINT_VALUE(ldat)
             ////	 DEBUG_PRINT_VALUE(id0)
             //double weit = ldat.sum();
             // VectorXd idw = ldat.adjoint()*id0 / weit;
             // VectorXd ids = (id0.array() - idw(0)).abs();
             // //DEBUG_PRINT_VALUE(idw)
             ////	 DEBUG_PRINT_VALUE(ids)
             //VectorXd::Index idm;
             //ids.minCoeff(&idm);
            out.LdataM = ldataM_;
            out.trajM = trajM_;
            Pend = avg.preAve;
            out.message_code = 2;
     }
     else
     {
         Pend = Pstart;
         out.message_code = 3;
     }

     Pstart = stdVectortoVectorXD(
         out.traj.end()
     );

     //DEBUG_PRINT_VALUE(Pend)
        // DEBUG_PRINT_VALUE(Pstart)
     Tstart = poseRPY2SE3(Pstart);
     FADataPinOutput out2 = N_FA_moveL_PP_Data(TMtool, Tstart, Pend, tscal, velocity, ts, remain12, r, u);

     out.append(out2);
     //traj2.show();

     //DEBUG_PRINT_VALUE(out2.traj.getSize())


     //pList output;
     //output.append(traj0);
     //output.append(traj);
     //output.append(traj2);

     //auto Xv = traj.getRow(0);
     //double xmin = *min_element(Xv.begin(), Xv.end());
     //double xmax = *max_element(Xv.begin(), Xv.end());
     //auto Yv = traj.getRow(1);
     //double ymin = *min_element(Yv.begin(), Yv.end());
     //double ymax = *max_element(Yv.begin(), Yv.end());
     //auto Zv = traj.getRow(2);
     //double zmin = *min_element(Zv.begin(), Zv.end());
     //double zmax = *max_element(Zv.begin(), Zv.end());
     //VectorXd xyz(6);
     //xyz << xmin, xmax, ymin, ymax, zmin, zmax;
     Vector3d r3;
     r3 << 0, r;
     Vector3d p3 = stdVectortoVectorXD(out.traj.end()).segment(0, 3);
     out.normP2R = norm(p3 - r3);

     return out;
 }


 pList Stewart_GMT::N_moveL_PP(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts)
 {
     set_N_Type_Stewart_DefaultParameter();

     //DEBUG_PRINT_MESSAGE("moveL_Stewart input")
     //DEBUG_PRINT_VALUE(pstart6);
     //DEBUG_PRINT_VALUE(pend6);
     //DEBUG_PRINT_VALUE(tf);
     //DEBUG_PRINT_VALUE(ts);
     //DEBUG_PRINT_VALUE(timescaling);

     Matrix4d Tstart = MathTool::poseRPY2SE3(Pstart);
     Matrix4d Tend = MathTool::poseRPY2SE3(Pend);

     if (illegal_lenth(Pstart, TypeN))
     {
         DEBUG_PRINT_ERROR_MESSAGE("Pstart out of range")
             return pList();
     }
     if (illegal_lenth(Pend, TypeN))
     {
         DEBUG_PRINT_ERROR_MESSAGE("Pend out of range")
             return pList();
     }
     NIKOutput PstartIK = N_Stewart_IK(Pstart);
     NIKOutput PendIK = N_Stewart_IK(Pend);
     Mat Lstart = PstartIK.alpha.array() - (PstartIK.alpha.array().square() + SQUARE(_h0) - PstartIK.ls.array().square()).sqrt();
     Mat Lend = PendIK.alpha.array() - (PendIK.alpha.array().square() + SQUARE(_h0) - PendIK.ls.array().square()).sqrt();
     //DEBUG_PRINT_VALUE(PendIK.ls)

     Matrix3d rstart = t2r(Tstart);
     Vector3d pstart = t2p(Tstart);
     Matrix3d rend = t2r(Tend);
     Vector3d pend = t2p(Tend);


     //TODO:TimeScaling == '5'
     double coef;
     if (timescaling == Cubic) coef = 3.f / 2.f;
     else coef = 15.f / 8.f;

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(Pstart)
     //DEBUG_PRINT_VALUE(Lstart)
     //DEBUG_PRINT_VALUE(Pend)
     //DEBUG_PRINT_VALUE(Lend)


         //DEBUG_PRINT_VALUE(Lstart - Lend)
         //DEBUG_PRINT_VALUE((Lstart - Lend).lpNorm<Infinity>())
     double t = coef *((Lstart - Lend).lpNorm<Infinity>() / velocity);
     int n = ceil(t / ts) + 1;

     pList ptraj;
     ptraj.P.reserve(n);

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(rstart)
     //DEBUG_PRINT_VALUE(rend)


     Mat tmat = MathTool::MatrixLog3(rstart.adjoint()*rend);
     //DEBUG_PRINT_VALUE(tmat);
     for (double i = 1; i <= n; i++)
     {
         double s;
         if (timescaling == enumTimeScaling::Cubic)
         {
             s = MathTool::CubicTimeScaling(t, ts*(i-1));
         }
         else
             s = MathTool::QuinticTimeScaling(t, ts*(i - 1));
         //DEBUG_PRINT_VALUE(i)
         //DEBUG_PRINT_VALUE(s)

         Matrix3d ri = rstart * MathTool::MatrixExp3(tmat*s);
         Vector3d pi = pstart + s * (pend - pstart);
         Matrix4d traji = MathTool::rp2t(ri, pi);
         VectorXd posei = MathTool::sE3ToPoseRPY(traji);
         //DEBUG_PRINT_VALUE(traji);
         //DEBUG_PRINT_VALUE(posei);
         ptraj.add(posei);
     }


     return ptraj;
 }

 TrajOnceOutput Stewart_GMT::N_moveL_PPOnce(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, int iter, Vector2d r, Vector2d u)
 {

     set_N_Type_Stewart_DefaultParameter();
		 static TrajOnceOutput moveLout;
		 std::cout << "start moveLout :" << moveLout.trajp << std::endl;

     //DEBUG_PRINT_MESSAGE("moveL_Stewart input")
     //DEBUG_PRINT_VALUE(pstart6);
     //DEBUG_PRINT_VALUE(pend6);
     //DEBUG_PRINT_VALUE(tf);
     //DEBUG_PRINT_VALUE(ts);
     //DEBUG_PRINT_VALUE(timescaling);

     Matrix4d Tstart = MathTool::poseRPY2SE3(Pstart);
     Matrix4d Tend = MathTool::poseRPY2SE3(Pend);

     if (illegal_lenth(Pstart, TypeN))
     {
         DEBUG_PRINT_ERROR_MESSAGE("Pstart out of range")
             return TrajOnceOutput("Pstart out of range");
     }
     if (illegal_lenth(Pstart, TypeN))
     {
         DEBUG_PRINT_ERROR_MESSAGE("Pend out of range")
             return TrajOnceOutput("Pend out of range");
     }
     NIKOutput PstartIK = N_Stewart_IK(Pstart);
     NIKOutput PendIK = N_Stewart_IK(Pend);
     Mat Lstart = PstartIK.alpha.array() - (PstartIK.alpha.array().square() + SQUARE(_h0) - PstartIK.ls.array().square()).sqrt();
     Mat Lend = PendIK.alpha.array() - (PendIK.alpha.array().square() + SQUARE(_h0) - PendIK.ls.array().square()).sqrt();

     Matrix3d rstart = t2r(Tstart);
     Vector3d pstart = t2p(Tstart);
     Matrix3d rend = t2r(Tend);
     Vector3d pend = t2p(Tend);


     //TODO:TimeScaling == '5'
     double coef;
     if (timescaling == Cubic) coef = 3.f / 2.f;
     else coef = 15 / 8;

     //DEBUG_PRINT_CODE_LOCATION
        // DEBUG_PRINT_VALUE(Pstart)
        // DEBUG_PRINT_VALUE(Lstart)
        // DEBUG_PRINT_VALUE(Pend)
        // DEBUG_PRINT_VALUE(Lend)


         double t = coef * ((Lstart - Lend).lpNorm<Infinity>() / velocity);
     int n = ceil(t / ts) + 1;


     //DEBUG_PRINT_CODE_LOCATION
        // DEBUG_PRINT_VALUE(rstart)
        // DEBUG_PRINT_VALUE(rend)

         double i = iter;
     if (i > n){
			 moveLout.is_finished = true;
			 std::cout << "finish moveLout :" << moveLout.trajp << std::endl;

			 return moveLout;
		 }
			else
			{
         Mat tmat = MathTool::MatrixLog3(rstart.adjoint()*rend);
     //DEBUG_PRINT_VALUE(tmat);
         double s;
         if (timescaling == enumTimeScaling::Cubic)
         {
             s = MathTool::CubicTimeScaling(t, ts*(i - 1));
         }
         else
             s = MathTool::QuinticTimeScaling(t, ts*(i - 1));
         //DEBUG_PRINT_VALUE(i)
         //DEBUG_PRINT_VALUE(s)
				 				 std::cout << "rstart :" << rstart << std::endl;
				 				 std::cout << "pstart :" << pstart << std::endl;
				 				 std::cout << "pend :" << pend << std::endl;
								 std::cout << "s :" << s << std::endl;

         Matrix3d ri = rstart * MathTool::MatrixExp3(tmat*s);
         Vector3d pi = pstart + s * (pend - pstart);
         Matrix4d traji = MathTool::rp2t(ri, pi);
				 				 std::cout << "ri :" << ri << std::endl;
				 				 std::cout << "pi :" << pi << std::endl;
				 				 std::cout << "traji :" << pi << std::endl;

         //DEBUG_PRINT_VALUE(traji);
         VectorXd posei = MathTool::sE3ToPoseRPY(traji);

         double y = posei(1);
         double z = posei(2);
         double ldata = Stewart_GMT::GetPowerMeterData_Wang(y, z, r, u);

         moveLout = TrajOnceOutput(posei, iter + 1);
         moveLout.Ldata = ldata;
				 
				 std::cout << "posei :" << posei << std::endl;
				 std::cout << "moveLout :" << moveLout.trajp << std::endl;
				 
         return moveLout;
			 }
 }

 FADataPinOutput Stewart_GMT::N_FA_moveL_PP_Data(Matrix4d TMtool, Matrix4d Tstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, VectorXd remain12, Vector2d r, Vector2d u)
 {
     set_N_Type_Stewart_DefaultParameter();

     //DEBUG_PRINT_MESSAGE("moveL_Stewart input")
     //DEBUG_PRINT_VALUE(pstart6);
     //DEBUG_PRINT_VALUE(pend6);
     //DEBUG_PRINT_VALUE(tf);
     //DEBUG_PRINT_VALUE(ts);
     //DEBUG_PRINT_VALUE(timescaling);

     Matrix4d TtoolStart = Tstart * TMtool;

     Matrix3d toolTrans = t2r(TtoolStart);
     Vector3d toolTransly = toolTrans.col(1);
     Vector3d toolTranslz = toolTrans.col(2);

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix4d Tend = MathTool::poseRPY2SE3(Pend);

     if (illegal_lenth(Pstart, TypeN))
     {
         DEBUG_PRINT_ERROR_MESSAGE("Pstart out of range")
             return FADataPinOutput("Pstart out of range");
     }
     if (illegal_lenth(Pend, TypeN))
     {
         DEBUG_PRINT_ERROR_MESSAGE("Pend out of range")
             return FADataPinOutput("Pend out of range");
     }
     NIKOutput PstartIK = N_Stewart_IK(Pstart);
     NIKOutput PendIK = N_Stewart_IK(Pend);
     Mat Lstart = PstartIK.alpha.array() - (PstartIK.alpha.array().square() + SQUARE(_h0) - PstartIK.ls.array().square()).sqrt();
     Mat Lend = PendIK.alpha.array() - (PendIK.alpha.array().square() + SQUARE(_h0) - PendIK.ls.array().square()).sqrt();
     //DEBUG_PRINT_VALUE(PendIK.ls)

     Matrix3d rstart = t2r(Tstart);
     Vector3d pstart = t2p(Tstart);
     Matrix3d rend = t2r(Tend);
     Vector3d pend = t2p(Tend);


     //TODO:TimeScaling == '5'
     double coef;
     if (timescaling == Cubic) coef = 3.f / 2.f;
     else coef = 15.f / 8.f;

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(Pstart)
     //DEBUG_PRINT_VALUE(Lstart)
     //DEBUG_PRINT_VALUE(Pend)
     //DEBUG_PRINT_VALUE(Lend)


         //DEBUG_PRINT_VALUE(Lstart - Lend)
         //DEBUG_PRINT_VALUE((Lstart - Lend).lpNorm<Infinity>())
     double t = coef * ((Lstart - Lend).lpNorm<Infinity>() / velocity);
     int n = ceil(t / ts) + 1;

     pList ptraj;
     ptraj.P.reserve(n);
     vector<double> ldata;
     ldata.reserve(n);

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(rstart)
     //DEBUG_PRINT_VALUE(rend)

     double y1 = remain12[WS_LIMIT_Y_MINUS];
     double y2 = remain12[WS_LIMIT_Y_PLUS];
     double z1 = remain12[WS_LIMIT_Z_MINUS];
     double z2 = remain12[WS_LIMIT_Z_PLUS];

     Mat tmat = MathTool::MatrixLog3(rstart.adjoint()*rend);
     //DEBUG_PRINT_VALUE(tmat);
     for (double i = 1; i <= n; i++)
     {
         double s;
         if (timescaling == enumTimeScaling::Cubic)
         {
             s = MathTool::CubicTimeScaling(t, ts*(i - 1));
         }
         else
             s = MathTool::QuinticTimeScaling(t, ts*(i - 1));
         //DEBUG_PRINT_VALUE(i)
         //DEBUG_PRINT_VALUE(s)

         Matrix3d ri = rstart * MathTool::MatrixExp3(tmat*s);
         Vector3d pi = pstart + s * (pend - pstart);
         Matrix4d traji = MathTool::rp2t(ri, pi);
         VectorXd posei = MathTool::sE3ToPoseRPY(traji);
         //DEBUG_PRINT_VALUE(traji);
         //DEBUG_PRINT_VALUE(posei);
         ptraj.add(posei);

         double yy = posei(1);
         double zz = posei(2);
         double Ldatai = Stewart_GMT::GetPowerMeterData_Wang(yy, zz, r, u);
         ldata.push_back(Ldatai);
     }

     return FADataPinOutput(ptraj, stdVectortoVectorXD(ldata));
 }

 WSOutput Stewart_GMT::A12_WS(VectorXd ToolPoseWorking)
 {
     if (_firstRun)
     {
         set_A12_DefaultParameter();
         Reset_TposeStart();
         _firstRun = false;
     }

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(_TposeStart)

         VectorXd delta = ToolPoseWorking - _LastLegalResult.input;
         DEBUG_PRINT_CODE_LOCATION
     DEBUG_PRINT_VALUE(delta)

         WSOutput newResult = a12_ws(delta);

     if (newResult.inputIllegal)
     {
         _LastLegalResult.inputIllegal = newResult.inputIllegal;
         _TposeStart = _LastLegalResult.tstart;
         return _LastLegalResult;
     }
     else
     {
         newResult.input = ToolPoseWorking;
         _LastLegalResult = newResult;
     }

     return newResult;
 }

 pList Stewart_GMT::A12_SpiralTrack(Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal)
 {
     set_A12_DefaultParameter();

     Matrix3d rstart = t2r(Tstart);
     Vector3d pstart = t2p(Tstart);
     Vector3d ptp = sO3ToPoseRPY(rstart);
     double phi = ptp(0);
     double theta = ptp(1);
     double psi = ptp(2);

     double ax = pstart(0);
     double ay = pstart(1);
     double az = pstart(2);

     double pdx = _work_space_test_limit[0] - ax;
     double mdx = _work_space_test_limit[1] - ax;
     double pdy = _work_space_test_limit[2] - ay;
     double mdy = _work_space_test_limit[3] - ay;
     double pdz = _work_space_test_limit[4] - az;
     double mdz = _work_space_test_limit[5] - az;
     double pdrz = _work_space_test_limit[6] - phi;
     double mdrz = _work_space_test_limit[7] - phi;
     double pdry = _work_space_test_limit[8] - theta;
     double mdry = _work_space_test_limit[9] - theta;
     double pdrx = _work_space_test_limit[10] - psi;
     double mdrx = _work_space_test_limit[11] - psi;

     double dx = min(abs(pdx), abs(mdx));
     double dy = min(abs(pdy), abs(mdy));
     double dz = min(abs(pdz), abs(mdz));

     Vector3d toolTranslx = rstart.col(0);
     Vector3d toolTransly = rstart.col(1);
     Vector3d toolTranslz = rstart.col(2);

     double b = lineSpacing / (2 * MathPI);

     double scan;
     scanRange <= min(dy, dz) ? scan = scanRange : scan = min(dy, dz);
     double v = velocity / 100;
     double tf = (2 * MathPI)*scan / lineSpacing;
     int K = ceil(tf / 0.0001) + 1;



     Vector3d X;
     Vector3d Y;
     Vector3d Z;

     pList traj;
     for (int k = 1; k <= K; k++)
     {
         double x0 = 0.1;
         SpiralTEvaluateParameter sp2;
         sp2.setsp2(b, k, v);
         double t = newton(x0, 1e-5, 50, &spiral_time_eval_f2, &sp2);
         //DEBUG_PRINT_VALUE(t);

         double y = (b * (k - 1)*t)*cos((k - 1)*t);
         double z = (b * (k - 1)*t)*sin((k - 1)*t);

         if ((abs(y) > scan || (abs(y) > scan)))
         {
             if (traj.getSize() == 0)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
                     DEBUG_PRINT_VALUE(Tstart)
                     DEBUG_PRINT_VALUE(v)
                     DEBUG_PRINT_VALUE(scanRange)

                     DEBUG_PRINT_VALUE(y)
                     DEBUG_PRINT_VALUE(z)
             }
             break;
         }

         Y = y * toolTransly;
         Z = z * toolTranslz;
         X = 0 * toolTranslz;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + X + Y + Z;
         //DEBUG_PRINT_VALUE(t)
         //DEBUG_PRINT_VALUE(X)
         //DEBUG_PRINT_VALUE(Y)
         //DEBUG_PRINT_VALUE(Z)
         //DEBUG_PRINT_VALUE(trans)

         VectorXd traji(6);
         traji << trans, ptp;
         //DEBUG_PRINT_VALUE(traji)

         traj.add(traji);

     }
     if (traj.getSize() == 0) return traj;

     VectorXd Pend = sE3ToPoseRPY(Tstart);
     VectorXd Pstart = stdVectortoVectorXD(
         traj.end()
     );

     pList traj2 = A12_moveL_PP(Pstart, Pend, tscal, velocity, ts);

     traj.append(traj2);

     //auto Xv = traj.getRow(0);
     //double xmin = *min_element(Xv.begin(), Xv.end());
     //double xmax = *max_element(Xv.begin(), Xv.end());
     //auto Yv = traj.getRow(1);
     //double ymin = *min_element(Yv.begin(), Yv.end());
     //double ymax = *max_element(Yv.begin(), Yv.end());
     //auto Zv = traj.getRow(2);
     //double zmin = *min_element(Zv.begin(), Zv.end());
     //double zmax = *max_element(Zv.begin(), Zv.end());
     //VectorXd xyz(6);
     //xyz << xmin, xmax, ymin, ymax, zmin, zmax;

     return traj;

 }

 pList Stewart_GMT::A12_SinTrack(Matrix4d Tstart, double velocity, double ts,enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange,enumSinScanMode scanMode)
 {

     set_A12_DefaultParameter();

     VectorXd Pstart = sE3ToPoseRPY(Tstart);
     Matrix3d rstart = t2r(Tstart);
     Vector3d pstart = t2p(Tstart);
     Vector3d ptp = sO3ToPoseRPY(rstart);
     double phi = ptp(0);
     double theta = ptp(1);
     double psi = ptp(2);

     double ax = pstart(0);
     double ay = pstart(1);
     double az = pstart(2);
     //DEBUG_PRINT_VALUE(ax)
        // DEBUG_PRINT_VALUE(ay)
        // DEBUG_PRINT_VALUE(az)
        // DEBUG_PRINT_VALUE(pstart)
        // DEBUG_PRINT_VALUE(Tstart)

     double pdx = _work_space_test_limit[0] - ax;
     double mdx = _work_space_test_limit[1] - ax;
     double pdy = _work_space_test_limit[2] - ay;
     double mdy = _work_space_test_limit[3] - ay;
     double pdz = _work_space_test_limit[4] - az;
     double mdz = _work_space_test_limit[5] - az;
     double pdrz = _work_space_test_limit[6] - phi;
     double mdrz = _work_space_test_limit[7] - phi;
     double pdry = _work_space_test_limit[8] - theta;
     double mdry = _work_space_test_limit[9] - theta;
     double pdrx = _work_space_test_limit[10] - psi;
     double mdrx = _work_space_test_limit[11] - psi;

     double dx = min(abs(pdx), abs(mdx));
     double dy = min(abs(pdy), abs(mdy));
     double dz = min(abs(pdz), abs(mdz));

     Vector3d toolTranslx = rstart.col(0);
     Vector3d toolTransly = rstart.col(1);
     Vector3d toolTranslz = rstart.col(2);

     double scanLimit;
     double stepLimit;
     Vector3d translv;
     Vector3d translh;
     if (scanMode == vertical)
     {
         scanLimit = abs(dz);
         stepLimit = abs(dy);
         translv = toolTranslz;
         translh = toolTransly;
     }
     else if (scanMode == horizontal)
     {
         scanLimit = abs(dy);
         stepLimit = abs(dz);
         translv = toolTransly;
         translh = toolTranslz;
     }

     double scan = min(abs(scanRange), scanLimit);
     double step = min(abs(stepRange), stepLimit);


     DEBUG_PRINT_VALUE(step)
         DEBUG_PRINT_VALUE(scan)
     double hs = step * (-1);
     Vector3d H1 = hs *translh;
     double vs = scan;
     Vector3d V1 = vs * translv;
     double x = 0.0;
     Vector3d X1 = x * toolTranslx;
     //DEBUG_PRINT_VALUE(X1)
        // DEBUG_PRINT_VALUE(H1)
        // DEBUG_PRINT_VALUE(V1)

    Vector3d trans1;
     trans1 << ax, ay, az;
     trans1 = trans1 + X1 + H1 + V1;
     VectorXd trajp1(6);
     trajp1 << trans1, ptp;
     VectorXd Pend = trajp1;

     //DEBUG_PRINT_CODE_LOCATION
        // DEBUG_PRINT_VALUE(trans1)
        // DEBUG_PRINT_VALUE(Pstart)
        // DEBUG_PRINT_VALUE(Pend)
         pList traj0 = A12_moveL_PP(Pstart, Pend, tscal, velocity, ts);
         if (traj0.getSize() == 0) return traj0;

     //traj0.show();


     double tf = (step * 2 * MathPI) / (2 * lineSpacing);
     double t = 1 / velocity;
     int K = ceil(tf / (ts*(1 / t)));

     int M = 2 * K + 1;
     Vector3d H;
     Vector3d V;

     pList traj;
     for (double k = 1; k <= M; k++)
     {
         double hs = step * ((k - 1) / K-1);
         double vs = scan * cos((k - 1)*ts*(1.0f / t));

         if(abs(hs)>step || abs(vs)> scan)
         {
             if (traj.getSize() == 0)
             {
                 DEBUG_PRINT_ERROR_MESSAGE("illeagle input")
                     DEBUG_PRINT_VALUE(Tstart)
                     DEBUG_PRINT_VALUE(scanRange)

                     DEBUG_PRINT_VALUE(hs)
                     DEBUG_PRINT_VALUE(vs)
             }
             break;
         }

         H = hs * translh;
         V = vs * translv;

         Vector3d trans;
         trans << ax, ay, az;
         trans = trans + H + V;
         //DEBUG_PRINT_VALUE(ax)
            // DEBUG_PRINT_VALUE(ay)
            // DEBUG_PRINT_VALUE(az)
         //DEBUG_PRINT_VALUE(H)
         //DEBUG_PRINT_VALUE(V)
         //DEBUG_PRINT_VALUE(trans)
         VectorXd traji(6);
         traji << trans, ptp;
         //DEBUG_PRINT_VALUE(traji)

         traj.add(traji);

     }

     Pend = Pstart;
     Pstart = stdVectortoVectorXD(
         traj.end()
     );

     pList traj2 = A12_moveL_PP(Pstart, Pend, tscal, velocity, ts);

     //auto Xv = traj.getRow(0);
     //double xmin = *min_element(Xv.begin(), Xv.end());
     //double xmax = *max_element(Xv.begin(), Xv.end());
     //auto Yv = traj.getRow(1);
     //double ymin = *min_element(Yv.begin(), Yv.end());
     //double ymax = *max_element(Yv.begin(), Yv.end());
     //auto Zv = traj.getRow(2);
     //double zmin = *min_element(Zv.begin(), Zv.end());
     //double zmax = *max_element(Zv.begin(), Zv.end());
     //VectorXd xyz(6);
     //xyz << xmin, xmax, ymin, ymax, zmin, zmax;

     pList traj_out;
     traj_out.append(traj0);
     traj_out.append(traj);
     traj_out.append(traj2);

     return traj_out;



 }

 pList Stewart_GMT::A12_moveL_PP(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts)
 {

     set_A12_DefaultParameter();

     //DEBUG_PRINT_MESSAGE("moveL_Stewart input")
     //DEBUG_PRINT_VALUE(pstart6);
     //DEBUG_PRINT_VALUE(pend6);
     //DEBUG_PRINT_VALUE(tf);
     //DEBUG_PRINT_VALUE(ts);
     //DEBUG_PRINT_VALUE(timescaling);

     Matrix4d Tstart = MathTool::poseRPY2SE3(Pstart);
     Matrix4d Tend = MathTool::poseRPY2SE3(Pend);

     //if (illegal_lenthA12(Pstart))
     //{
        // DEBUG_PRINT_ERROR_MESSAGE("Pstart out of range")
        //	 return pList();
     //}
     //if (illegal_lenthA12(Pend))
     //{
        // DEBUG_PRINT_ERROR_MESSAGE("Pend out of range")
        //	 return pList();
     //}

     Matrix3d rstart = t2r(Tstart);
     Vector3d pstart = t2p(Tstart);
     Matrix3d rend = t2r(Tend);
     Vector3d pend = t2p(Tend);

     double coef;
     if (timescaling == Cubic) coef = 3.f / 2.f;
     else if (timescaling == Quintic) coef = 15.f / 8.f;

     //DEBUG_PRINT_CODE_LOCATION
        // DEBUG_PRINT_VALUE(Pstart)
        // DEBUG_PRINT_VALUE(Pend)

         VectorXd Lstart = Pstart;
     VectorXd Lend = Pend;


         //DEBUG_PRINT_VALUE(Lstart - Lend)
         //DEBUG_PRINT_VALUE((Lstart - Lend).lpNorm<Infinity>())
         double t = coef * ((Lstart - Lend).lpNorm<Infinity>() / velocity);
     int n = ceil(t / ts) + 1;

     pList ptraj;
     ptraj.P.reserve(n);

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(rstart)
     //DEBUG_PRINT_VALUE(rend)


     Mat tmat = MathTool::MatrixLog3(rstart.adjoint()*rend);
     //DEBUG_PRINT_VALUE(tmat);
     for (double i = 1; i <= n; i++)
     {
         double s;
         if (timescaling == enumTimeScaling::Cubic)
         {
             s = MathTool::CubicTimeScaling(t, ts*(i - 1));
         }
         else
             s = MathTool::QuinticTimeScaling(t, ts*(i - 1));
         //DEBUG_PRINT_VALUE(i)
         //DEBUG_PRINT_VALUE(s)

         Matrix3d ri = rstart * MathTool::MatrixExp3(tmat*s);
         Vector3d pi = pstart + s * (pend - pstart);
         Matrix4d traji = MathTool::rp2t(ri, pi);
         //DEBUG_PRINT_VALUE(traji);
         VectorXd posei = MathTool::sE3ToPoseRPY(traji);
         ptraj.add(posei);
     }


     return ptraj;
 }


 //???? ?????
 //??pose?????????
 //??1*12??,??:X??????X??????Y?????Y?????...Z...phi...theta...psi????
 //return 1*12 remaing x+ x- y+ y- ... z... phi...theta...psi
 VectorXd Stewart_GMT::F_WorkSpace1(VectorXd Pose)
 {
     Stewart_GMT::set_F_Type_Stewart_DefaultParameter();

     double interval = _work_space_delta_trans;
     double Lupp = _Lupp;
     double Llow = _Llow;

     VectorXd P = Pose;
     VectorXd PTest(6);
     VectorXd RemValue(12);
     RemValue.setZero();


     for (double i = 0; i < _work_space_test_limit[0]; i = i + interval)
     {
         PTest << P + (VectorXd(6) << i, 0, 0, 0, 0, 0).finished();
         PTest << P(0) + i, P(1), P(2), P(3), P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[0] = i - interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[1]; i = i - interval)
     {
         PTest << P(0) + i, P(1), P(2), P(3), P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[1] = i + interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[2]; i = i + interval)
     {
         PTest << P(0), P(1) + i, P(2), P(3), P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[2] = i - interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[3]; i = i - interval)
     {
         PTest << P(0), P(1) + i, P(2), P(3), P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[3] = i + interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[4]; i = i + interval)
     {
         PTest << P(0), P(1), P(2) + i, P(3), P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[4] = i - interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[5]; i = i - interval)
     {
         PTest << P(0), P(1), P(2) + i, P(3), P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[5] = i + interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[6]; i = i + interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3) + i, P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[6] = i - interval * interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[7]; i = i - interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3) + i, P(4), P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[7] = i + interval * interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[8]; i = i + interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4) + i, P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[8] = i - interval * interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[9]; i = i - interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4) + i, P(5);
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[9] = i + interval * interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[10]; i = i + interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4), P(5) + i;
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[10] = i - interval * interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[11]; i = i - interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4), P(5) + i;
         VectorXd LL = Stewart_GMT::F_Stewart_IK(PTest);
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[11] = i + interval * interval;
             break;
         }
     }

     return RemValue;
 }

 //??pose?????????
//??1*12??,??:X??????X??????Y?????Y?????...Z...phi...theta...psi????
//return 1*12 remaing x+ x- y+ y- ... z... phi...theta...psi
VectorXd Stewart_GMT::F_WorkSpace(VectorXd Pose)
{
    return WorkSpace(Pose, TypeF);
}

//??pose?????????
//??1*12??,??:X??????X??????Y?????Y?????...Z...phi...theta...psi????
//return 1*12 remaing x+ x- y+ y- ... z... phi...theta...psi
VectorXd Stewart_GMT::N_WorkSpace(VectorXd Pose)
{
    return WorkSpace(Pose, TypeN);
}

VectorXd Stewart_GMT::F_PivotWorkSpace(VectorXd Pstart, Vector3d Ppivot, double alpha, double beta)
{
    //return WorkSpacePivot(Pstart, Ppivot, alpha, beta, TypeF);
    return VectorXd();
}

WSOutput Stewart_GMT::N_ToolWS(VectorXd ToolPoseStart, Vector3d Ppivot, double alpha, double beta, enumBaseMode baseMode)
{
    if (_firstRun)
    {
        set_N_Type_Stewart_DefaultParameter();
        Reset_TposeStart();
        _firstRun = false;
    }
    DEBUG_PRINT_VALUE(_TposeStart)

    VectorXd delta = ToolPoseStart - _LastLegalResult.input;
    DEBUG_PRINT_VALUE(delta)

    WSOutput newResult = WorkSpacePivot(delta, Ppivot, alpha, beta, TypeN, baseMode);

    if (newResult.inputIllegal)
    {
        _TposeStart = _LastLegalResult.tstart;
        newResult = _LastLegalResult;
        newResult.inputIllegal = true;
        return newResult;
    }
    else
    {
        newResult.input = ToolPoseStart;
        _LastLegalResult = newResult;
    }

    return newResult;
}

pList Stewart_GMT::N_ToolPivotTraj(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double tf, double ts, enumTimeScaling tscale, enumToolPivotTrajType pivotTrajType)
{
    return ToolPivotTrajCal(pivot, alpha, beta,  Tstart, tf, ts, tscale, TypeN, pivotTrajType);
}


void Stewart_GMT::Reset_TposeStart()
{
    _TposeStart = poseRPY2SE3(_phome);
    _LastLegalResult.tstart = poseRPY2SE3(_phome);
    //_LastLegalResult.lastLegalInput = (VectorXd(6) << 0, 0, 0, 0, 0, 0).finished();
    _LastLegalResult.input = VectorXd::Zero(6);
    _LastLegalResult.remain12 = VectorXd::Zero(12);
}

void Stewart_GMT::SetWSdelta(double trans, double rot)
{
    _work_space_delta_trans = trans;
    _work_space_delta_rot = rot;
}

pList Stewart_GMT::ToolPivotTrajCal(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double tf, double ts, enumTimeScaling tscale, enumStewartType stewart_type, enumToolPivotTrajType pivotTrajType)
{
    //type check
    if (!check_stewart_type_and_set_parameter(stewart_type)) return pList();


    Matrix3d ry = rotY(alpha);
    Matrix3d rx = rotX(beta);
    Vector3d PMtool = pivot;
    Matrix3d RMtool = ry * rx;
    Matrix4d TMtool = rp2t(RMtool, PMtool);

    Matrix4d TtoolStart = Tstart * TMtool;
    Vector3d MposiStart = t2p(TtoolStart);
    Matrix3d MorientStart = t2r(TtoolStart);

    VectorXd poseStart = sE3ToPoseRPY(Tstart);

    Vector3d toolTranslx = MorientStart.col(0);
    Vector3d toolTransly = MorientStart.col(1);
    Vector3d toolTranslz = MorientStart.col(2);

    double interval_rot = _work_space_delta_rot;

    pList ptraj;
    WSLimitTestOutput limit_angle_z;
    if (pivotTrajType == ToolPivotTraj_Z || pivotTrajType == ToolPivotTraj_XYZ)
    {
        limit_angle_z = ToolWorkSpaceLoopPivot(poseStart, pivot, toolTranslz, stewart_type, _work_space_test_limit[10], _work_space_test_limit[11], interval_rot);

        ptraj.append(
            pivotTraj(poseStart, pivot, toolTranslz, limit_angle_z.max, tf, ts, tscale)
        );
        ptraj.append(
            pivotTraj(poseStart, pivot, toolTranslz, limit_angle_z.min, tf, ts, tscale)
        );
    }

    WSLimitTestOutput limit_angle_y;
    if (pivotTrajType == ToolPivotTraj_Y || pivotTrajType == ToolPivotTraj_XYZ)
    {
        limit_angle_y = ToolWorkSpaceLoopPivot(poseStart, pivot, toolTransly, stewart_type, _work_space_test_limit[8], _work_space_test_limit[9], interval_rot);

        ptraj.append(
            pivotTraj(poseStart, pivot, toolTransly, limit_angle_y.max, tf, ts, tscale)
        );
        ptraj.append(
            pivotTraj(poseStart, pivot, toolTransly, limit_angle_y.min, tf, ts, tscale)
        );
    }

    WSLimitTestOutput limit_angle_x;
    if (pivotTrajType == ToolPivotTraj_X || pivotTrajType == ToolPivotTraj_XYZ)
    {
        limit_angle_x = ToolWorkSpaceLoopPivot(poseStart, pivot, toolTranslx, stewart_type, _work_space_test_limit[6], _work_space_test_limit[7], interval_rot);

        ptraj.append(
            pivotTraj(poseStart, pivot, toolTranslx, limit_angle_x.max, tf, ts, tscale)
        );
        ptraj.append(
            pivotTraj(poseStart, pivot, toolTranslx, limit_angle_x.min, tf, ts, tscale)
        );
    }


    VectorXd RemValue6(6);
    RemValue6 <<
        limit_angle_z.max, limit_angle_z.min,
        limit_angle_y.max, limit_angle_y.min,
        limit_angle_x.max, limit_angle_x.min;



    VectorXd RemValue12(12);

    return ptraj;
}

 VectorXd Stewart_GMT::WorkSpace(VectorXd Pose, enumStewartType stewart_type)
 {
     //type check
     if (stewart_type == TypeF) Stewart_GMT::set_F_Type_Stewart_DefaultParameter();
     else if (stewart_type == TypeN) Stewart_GMT::set_N_Type_Stewart_DefaultParameter();
     else {
         DEBUG_PRINT_MESSAGE("type unknow")
         return VectorXd();
     }

     // 1,0,0,0,0,0
     // 0,1,0,0,0,0...
     MatrixXd mask = MatrixXd::Identity(6, 6);

     double interval_trans = _work_space_delta_trans;
     double interval_rot = _work_space_delta_trans* _work_space_delta_trans;

     VectorXd limit_x = WorkSpaceLoop(Pose, stewart_type, mask.col(0), _work_space_test_limit[0], _work_space_test_limit[1], interval_trans);
     VectorXd limit_y = WorkSpaceLoop(Pose, stewart_type, mask.col(1), _work_space_test_limit[2], _work_space_test_limit[3], interval_trans);
     VectorXd limit_z = WorkSpaceLoop(Pose, stewart_type, mask.col(2), _work_space_test_limit[4], _work_space_test_limit[5], interval_trans);
     VectorXd limit_phi = WorkSpaceLoop(Pose, stewart_type, mask.col(3), _work_space_test_limit[6], _work_space_test_limit[7], interval_rot);
     VectorXd limit_theta = WorkSpaceLoop(Pose, stewart_type, mask.col(4), _work_space_test_limit[8], _work_space_test_limit[9], interval_rot);
     VectorXd limit_psi = WorkSpaceLoop(Pose, stewart_type, mask.col(5), _work_space_test_limit[10], _work_space_test_limit[11], interval_rot);

     VectorXd RemValue12(12);
     RemValue12 << limit_x, limit_y, limit_z, limit_phi, limit_theta, limit_psi;

     return RemValue12;
 }


 WSOutput Stewart_GMT::WorkSpacePivot(VectorXd ToolPosistart, Vector3d pivot, double alpha, double beta, enumStewartType stewart_type, enumBaseMode baseMode)
 {
     //type check
     if (!check_stewart_type_and_set_parameter(stewart_type)) return WSOutput();

     WSOutput out;

     toolTrans m = ToolCalculateM(ToolPosistart, pivot, alpha, beta, baseMode);
     out.tstart = _TposeStart;
     out.TMtool = m.TMtool;
     DEBUG_PRINT_VALUE(m.poseStart)

     double interval_trans = _work_space_delta_trans;
     WSLimitTestOutput limit_posi_x = ToolWorkSpaceLoop(m.poseStart, stewart_type, m.xrtool, _work_space_test_limit[WS_LIMIT_X_PLUS], _work_space_test_limit[WS_LIMIT_X_MINUS], interval_trans);
     WSLimitTestOutput limit_posi_y = ToolWorkSpaceLoop(m.poseStart, stewart_type, m.yrtool, _work_space_test_limit[WS_LIMIT_Y_PLUS], _work_space_test_limit[WS_LIMIT_Y_MINUS], interval_trans);
     WSLimitTestOutput limit_posi_z = ToolWorkSpaceLoop(m.poseStart, stewart_type, m.zrtool, _work_space_test_limit[WS_LIMIT_Z_PLUS], _work_space_test_limit[WS_LIMIT_Z_MINUS], interval_trans);

     double interval_rot = _work_space_delta_rot;
     WSLimitTestOutput limit_angle_z = ToolWorkSpaceLoopPivot(m.poseStart, pivot, m.zrtool, stewart_type, _work_space_test_limit[WS_LIMIT_PHI_PLUS], _work_space_test_limit[WS_LIMIT_PHI_MINUS], interval_rot);
     WSLimitTestOutput limit_angle_y = ToolWorkSpaceLoopPivot(m.poseStart, pivot, m.yrtool, stewart_type, _work_space_test_limit[WS_LIMIT_THETA_PLUS], _work_space_test_limit[WS_LIMIT_THETA_MINUS], interval_rot);
     WSLimitTestOutput limit_angle_x = ToolWorkSpaceLoopPivot(m.poseStart, pivot, m.xrtool, stewart_type, _work_space_test_limit[WS_LIMIT_PSI_PLUS], _work_space_test_limit[WS_LIMIT_PSI_MINUS], interval_rot);

     VectorXd RemValue12(12);
     RemValue12 << limit_posi_x.max, limit_posi_x.min,
         limit_posi_y.max, limit_posi_y.min,
         limit_posi_z.max, limit_posi_z.min,
         limit_angle_z.max, limit_angle_z.min,
         limit_angle_y.max, limit_angle_y.min,
         limit_angle_x.max, limit_angle_x.min;

     out.remain12 = RemValue12;

     out.inputIllegal = limit_posi_x.inputIllegal
         || limit_posi_x.inputIllegal
         || limit_posi_y.inputIllegal
         || limit_angle_z.inputIllegal
         || limit_angle_y.inputIllegal
         || limit_angle_x.inputIllegal;

     return out;
 }

 VectorXd Stewart_GMT::WorkSpaceLoop(VectorXd Pose, enumStewartType stewart_type, VectorXd w, double limit_plus, double limit_minus, double interval)
 {
    VectorXd PTest(6);
    VectorXd RemValue2 = VectorXd::Zero(2);
    VectorXd w6(6);
    w6 << w, 0, 0, 0;

    for (double i=0;i < limit_plus;i+=interval)
    {
        PTest << Pose + w6 * i;

        if (illegal_lenth(PTest, stewart_type))
        {
            if (i == 0) {
                DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                DEBUG_PRINT_VALUE(PTest)
                RemValue2[0] = -1;
            }
            else RemValue2[0] = i - interval;

            break;
        }
    }

    for (double i = 0; i > limit_minus;i-=interval)
    {
        PTest << Pose + w6 * i;

        if (illegal_lenth(PTest, stewart_type))
        {
            if (i == 0) {
                DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                DEBUG_PRINT_VALUE(PTest)
                RemValue2[1] = -1;
            }
            else RemValue2[1] = i + interval;
            break;
        }
    }

     return RemValue2;
 }


 WSLimitTestOutput Stewart_GMT::ToolWorkSpaceLoop(VectorXd Pose, enumStewartType stewart_type, VectorXd w, double limit_plus, double limit_minus, double interval)
 {
     VectorXd PTest(6);
     WSLimitTestOutput out;
     VectorXd w6(6);
     w6 << w, 0, 0, 0;


     for (double i = 0; i < limit_plus; i += interval)
     {
         PTest << Pose + w6 * i;

         if (illegal_lenth(PTest, stewart_type))
         {
             if (i == 0) {
                 DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                     DEBUG_PRINT_VALUE(PTest)
                     out.inputIllegal = true;
             }
             else out.max = i - interval;

             break;
         }
     }

     for (double i = 0; i > limit_minus; i -= interval)
     {
         PTest << Pose + w6 * i;

         if (illegal_lenth(PTest, stewart_type))
         {
             if (i == 0) {
                 DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                     DEBUG_PRINT_VALUE(PTest)
                     out.inputIllegal = true;
             }
             else out.min = i + interval;
             break;
         }
     }

     return out;
 }


 WSLimitTestOutput Stewart_GMT::ToolWorkSpaceLoopPivot(VectorXd Pstart, VectorXd Ppivot, VectorXd w, enumStewartType stewart_type, double limit_plus, double limit_minus, double interval)
 {
     WSLimitTestOutput out;

     //clock_t pos_loop_start = clock();
     bool posi_break = false;
     //DEBUG_PRINT_MESSAGE("pos limit")
     for (double posi_angle_test = 0;posi_angle_test < limit_plus; posi_angle_test += interval)
     {
         double theta = posi_angle_test;

         //clock_t pivot_start = clock();
         VectorXd trajn = PivotTrajForCheck(Pstart, Ppivot, w, posi_angle_test);

             if (illegal_lenth(trajn, stewart_type))
             {
                 if (posi_angle_test == 0) {
                     DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                         DEBUG_PRINT_VALUE(posi_angle_test)
                         DEBUG_PRINT_VALUE(Pstart)
                         out.inputIllegal = true;
                 }
                 else {
                     out.max = posi_angle_test - interval;
                 }

                 posi_break = true;
                 break;
             }
     }

     //RELEASE_PRINT_MESSAGE("positive loop??:" << clock() - pos_loop_start << "(ms)" << endl)


     //clock_t nag_loop_start = clock();
     //DEBUG_PRINT_MESSAGE("nag limit")
     bool nag_break = false;

     for (double nag_angle_test = 0; nag_angle_test > limit_minus; nag_angle_test -= interval)
     {
         double theta = nag_angle_test;
         VectorXd trajn = PivotTrajForCheck(Pstart, Ppivot, w, nag_angle_test);

         if (illegal_lenth(trajn, stewart_type))
             {
                 if (nag_angle_test == 0) {
                     DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                         DEBUG_PRINT_VALUE(nag_angle_test)
                         out.inputIllegal = true;
                 }
                 else {
                     out.min = nag_angle_test + interval;
                 }

                 nag_break = true;
                 break;
             }

         //}

         //if (nag_break) break;


         //loop_counter++;
     }
     //RELEASE_PRINT_MESSAGE("nagtive loop??:" << clock() - nag_loop_start << "(ms)" << endl)
     //RELEASE_PRINT_VALUE(loop_counter)

     return out;
 }


 ToolPivotTrajLoopOutput Stewart_GMT::ToolPivotTrajloop(VectorXd Pstart, VectorXd Ppivot, VectorXd w, double tf, double ts, enumTimeScaling tscale, enumStewartType stewart_type, double limit_plus, double limit_minus, double interval)
 {
     ToolPivotTrajLoopOutput out;

     bool posi_break = false;
     for (double posi_angle_test = 0; posi_angle_test < limit_plus; posi_angle_test += interval)
     {
         double theta = posi_angle_test;
         TrajOnceOutput traj_0 = pivotTrajOnce(Pstart, Ppivot, w, posi_angle_test, tf, ts, tscale, 0);
         TrajOnceOutput traj_n = pivotTrajOnce(Pstart, Ppivot, w, posi_angle_test, tf, ts, tscale, traj_0.n);

         if (illegal_lenth(traj_n.trajp, stewart_type))
         {
             if (posi_angle_test == 0) {
                 DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                     DEBUG_PRINT_VALUE(posi_angle_test)
                     DEBUG_PRINT_VALUE(traj_0.trajp)
                     DEBUG_PRINT_VALUE(traj_n.trajp)
                     DEBUG_PRINT_VALUE(Pstart)
             }
             else {
                 pList traj = pivotTraj(Pstart, Ppivot, w, posi_angle_test, tf, ts, tscale);
                 traj.removeLast();
                 out.trajplus = traj;
                 out.toollimitplus = posi_angle_test - interval;
             }

             posi_break = true;
             break;
         }
     }

     //RELEASE_PRINT_MESSAGE("positive loop??:" << clock() - pos_loop_start << "(ms)" << endl)


     //clock_t nag_loop_start = clock();
     //DEBUG_PRINT_MESSAGE("nag limit")
     bool nag_break = false;

     for (double nag_angle_test = 0; nag_angle_test > limit_minus; nag_angle_test -= interval)
     {
         double theta = nag_angle_test;
         TrajOnceOutput traj_0 = pivotTrajOnce(Pstart, Ppivot, w, nag_angle_test, tf, ts, tscale, 0);
         TrajOnceOutput traj_n = pivotTrajOnce(Pstart, Ppivot, w, nag_angle_test, tf, ts, tscale, traj_0.n);

         if (illegal_lenth(traj_n.trajp, stewart_type))
         {
             if (nag_angle_test == 0) {
                 DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
                     DEBUG_PRINT_VALUE(nag_angle_test)
                     DEBUG_PRINT_VALUE(traj_0.trajp)
                     DEBUG_PRINT_VALUE(traj_n.trajp)
             }
             else {

                 pList traj = pivotTraj(Pstart, Ppivot, w, nag_angle_test, tf, ts, tscale);
                 traj.removeLast();
                 out.trajminus = traj;
                 out.toollimitminus = nag_angle_test + interval;
             }

             nag_break = true;
             break;
         }

         //}

         //if (nag_break) break;


         //loop_counter++;
     }
     //RELEASE_PRINT_MESSAGE("nagtive loop??:" << clock() - nag_loop_start << "(ms)" << endl)
     //RELEASE_PRINT_VALUE(loop_counter)

     return out;
 }

 WSOutput Stewart_GMT::a12_ws(VectorXd ToolPoseWorking)
 {

     WSOutput out;

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(ToolPoseWorking)
        // DEBUG_PRINT_VALUE(_TposeStart)

     Matrix4d TtoolPoseWorking = poseRPY2SE3(ToolPoseWorking);
     _TposeStart = _TposeStart * TtoolPoseWorking;
     out.tstart = _TposeStart;
     //DEBUG_PRINT_VALUE(_TposeStart)
        // DEBUG_PRINT_VALUE(out.tstart)
     Matrix3d rstart = t2r(_TposeStart);
     Vector3d pstart = t2p(_TposeStart);
     Vector3d ptp = sO3ToPoseRPY(rstart);
     double phi = ptp(0);
     double theta = ptp(1);
     double psi = ptp(2);

     double ax = pstart(0);
     double ay = pstart(1);
     double az = pstart(2);

     double pdx = _work_space_test_limit[0] - ax;
     double mdx = _work_space_test_limit[1] - ax;
     double pdy = _work_space_test_limit[2] - ay;
     double mdy = _work_space_test_limit[3] - ay;
     double pdz = _work_space_test_limit[4] - az;
     double mdz = _work_space_test_limit[5] - az;
     double pdrz = _work_space_test_limit[6] - phi;
     double mdrz = _work_space_test_limit[7] - phi;
     double pdry = _work_space_test_limit[8] - theta;
     double mdry = _work_space_test_limit[9] - theta;
     double pdrx = _work_space_test_limit[10] - psi;
     double mdrx = _work_space_test_limit[11] - psi;

     VectorXd pd(6);
     pd << pdx, pdy, pdz, pdrz, pdry, pdrx;
     VectorXd md(6);
     md << mdx, mdy, mdz, mdrz, mdry, mdrx;

     //DEBUG_PRINT_CODE_LOCATION
     //DEBUG_PRINT_VALUE(pd)
        // DEBUG_PRINT_VALUE(md)

     auto illegalp = (pd.array() < 0);
     auto illegalm = (md.array() > 0);
     if (illegalp.any() || illegalm.any()) out.inputIllegal = true;

     VectorXd RemValue12(12);
     RemValue12 << pdx, mdx,
         pdy, mdy,
         pdz, mdz,
         pdrz, mdrz,
         pdry, mdry,
         pdrx, mdrx;

     out.remain12 = RemValue12;

     return out;
 }

 VectorXd Stewart_GMT::PivotTrajForCheck(VectorXd Pstart, VectorXd pivot, VectorXd w, double theta)
 {
     Matrix4d xstart = poseRPY2SE3(Pstart);
     Vector3d r = pivot;

     w = w / norm(w);
     Matrix3d wx = vect3tomatrix3(w);
     Matrix3d rx = vect3tomatrix3(r);
     Matrix4d s;
     s << wx, rx*w, 0, 0, 0, 0;
     Matrix4d xstart_to_end = MatrixExp6(s*theta);
     Matrix4d traj_end = xstart * xstart_to_end;
     VectorXd ptraj_end = sE3ToPoseRPY(traj_end);

     return ptraj_end;
 }


 toolTrans Stewart_GMT::ToolCalculateM(VectorXd ToolPoseWorking, Vector3d pivot, double alpha, double beta, enumBaseMode baseMode)
 {
     toolTrans t_out;

     Matrix3d ry = rotY(alpha);
     Matrix3d rx = rotX(beta);
     Vector3d PMtool = pivot;
     Matrix3d RMtool = ry * rx;
     Matrix4d TMtool = rp2t(RMtool, PMtool);
     t_out.TMtool = TMtool;

     Matrix4d TtoolPoseWorking = poseRPY2SE3(ToolPoseWorking);
     Vector3d MposiWork;
     if (baseMode == base) MposiWork = t2p(TtoolPoseWorking);
     else if (baseMode == tool)MposiWork = RMtool * t2p(TtoolPoseWorking);
     else {
         DEBUG_PRINT_ERROR_MESSAGE("base mode unknow")
             return t_out;
     }
     Matrix3d ToolOrietWork = t2r(TtoolPoseWorking);
     VectorXd w = sO3ToPoseAxisAngle(ToolOrietWork);
     VectorXd wm = RMtool * w;

     double theta = norm(wm);
     Matrix4d trotStart;
     if (NearZero(theta))
     {
         trotStart = Matrix4d::Identity();
     }
     else
     {
         VectorXd w3 = wm / theta;
         Vector3d r = pivot;
         Matrix3d wx = vect3tomatrix3(w3);
         Matrix4d s;
         s << wx, -wx * r, 0, 0, 0, 0;
         trotStart = MatrixExp6(s*theta);
     }

     Matrix4d THMwork = rp2t(Matrix3d::Identity(), MposiWork);
     Matrix4d Twork = THMwork*  MatrixExp6(MatrixLog6(THMwork.inverse()*trotStart));
     _TposeStart = _TposeStart * THMwork*Twork;
     Matrix4d Tstart = _TposeStart;
     t_out.poseStart = sE3ToPoseRPY(_TposeStart);
     //DEBUG_PRINT_VALUE(_TposeStart);
     //DEBUG_PRINT_VALUE(t_out.poseStart);

     t_out.xrtool = RMtool.col(0);
     t_out.yrtool = RMtool.col(1);
     t_out.zrtool = RMtool.col(2);

     return t_out;
 }

 //?? ?????
 //??pose?????????
 //??1*12??,??:X??????X??????Y?????Y?????...Z...phi...theta...psi????
 //return 1*12 remaing x+ x- y+ y- ... z... phi...theta...psi
 VectorXd Stewart_GMT::N_WorkSpace1(VectorXd Pose)
 {
     Stewart_GMT::set_N_Type_Stewart_DefaultParameter();

     double interval = _work_space_delta_trans;
     double Lupp = _Lupp;
     double Llow = _Llow;

     VectorXd P = Pose;
     VectorXd PTest(6);
     VectorXd RemValue(12);
     RemValue.setZero();


     DEBUG_PRINT_VALUE(Pose);
     for (double i = 0; i < _work_space_test_limit[0]; i = i + interval)
     {
         PTest << P(0) + i, P(1), P(2), P(3), P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[0] = i - interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[1]; i = i - interval)
     {
         PTest << P(0) + i, P(1), P(2), P(3), P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[1] = i + interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[2]; i = i + interval)
     {
         PTest << P(0), P(1) + i, P(2), P(3), P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[2] = i - interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[3]; i = i - interval)
     {
         PTest << P(0), P(1) + i, P(2), P(3), P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[3] = i + interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[4]; i = i + interval)
     {
         PTest << P(0), P(1), P(2) + i, P(3), P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[4] = i - interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[5]; i = i - interval)
     {
         PTest << P(0), P(1), P(2) + i, P(3), P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[5] = i + interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[6]; i = i + interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3) + i, P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[6] = i - interval * interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[7]; i = i - interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3) + i, P(4), P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[7] = i + interval * interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[8]; i = i + interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4) + i, P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[8] = i - interval * interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[9]; i = i - interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4) + i, P(5);
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[9] = i + interval * interval;
             break;
         }
     }
     for (double i = 0; i < _work_space_test_limit[10]; i = i + interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4), P(5) + i;
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[10] = i - interval * interval;
             break;
         }
     }
     for (double i = 0; i > _work_space_test_limit[11]; i = i - interval * interval)
     {
         PTest << P(0), P(1), P(2), P(3), P(4), P(5) + i;
         NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);
         VectorXd dL = nik_result.ls;
         VectorXd LL = dL.array() + _g0;
         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             RemValue[11] = i + interval * interval;
             break;
         }
     }

     return RemValue;
 }




 pList Stewart_GMT::ScrewTrack(VectorXd Pstart, double t, double Ts, double b, double zb, double N)
 {
     b = b / 6.283;
     double ax = Pstart(0);
     double ay = Pstart(1);
     double az = Pstart(2);

     double Lupp = 181;
     double Llow = 164;
     double Tf = N * 2 * MathPI;

     int K = ceil(Tf / (Ts * (1 / t))) + 1;

     vector<double> X;
     vector<double> Y;
     vector<double> Z;

     pList traj;
     for (int i = 0; i < K; i++)
     {
         X.push_back(
             ax + (b * i * Ts * (1 / t)) * cos(i * Ts * (1 / t))
         );
         Y.push_back(
             ay + (b * i * Ts * (1 / t)) * sin(i * Ts * (1 / t))
         );
         Z.push_back(
             az + zb * (double(i) / K)
         );
         VectorXd P(6);
         P << X[i], Y[i], Z[i], Pstart(3), Pstart(4), Pstart(5);

         VectorXd LL = F_Stewart_IK(P, _pp, _bb);

         auto illegal_lenth = (LL.array() < Llow || LL.array() > Lupp);
         if (illegal_lenth.any())
         {
             DEBUG_PRINT_VALUE(i);
             DEBUG_PRINT_VALUE(P);
             DEBUG_PRINT_VALUE(LL);
             DEBUG_PRINT_VALUE(illegal_lenth);
             break;
         }

         traj.add(P);
     }

     return traj;
 }


 //n:????? x:????? fvec:N_myfun?? iflag:?"1"?????hybrid??
 //??hybrid(=matlab fsolve)?????????
 void Stewart_GMT::N_myfun(const int *n, const double *x, double *fvec, int *iflag)
 {
#pragma region
     //if (_is_show_fk_debug)
        // cout << "x in : " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << endl;

     VectorXd P(6);
     P << x[0], x[1], x[2], x[3], x[4], x[5];
     Mat R = MathTool::poseRPY2SO3_2(P);
     Vector3d X = P.segment(0, 3);

     Vector3d bb1 = _bb.col(0);
     Vector3d bb2 = _bb.col(1);
     Vector3d bb3 = _bb.col(2);
     Vector3d bb4 = _bb.col(3);
     Vector3d bb5 = _bb.col(4);
     Vector3d bb6 = _bb.col(5);

     Vector3d pp1 = _pp.col(0);
     Vector3d pp2 = _pp.col(1);
     Vector3d pp3 = _pp.col(2);
     Vector3d pp4 = _pp.col(3);
     Vector3d pp5 = _pp.col(4);
     Vector3d pp6 = _pp.col(5);

     Vector3d Bp1 = R * pp1;
     Vector3d Bp2 = R * pp2;
     Vector3d Bp3 = R * pp3;
     Vector3d Bp4 = R * pp4;
     Vector3d Bp5 = R * pp5;
     Vector3d Bp6 = R * pp6;

     double l1 = _L(0);
     double l2 = _L(1);
     double l3 = _L(2);
     double l4 = _L(3);
     double l5 = _L(4);
     double l6 = _L(5);

#pragma endregion

     double h02 = SQUARE(_h0);
     fvec[0] = SQUARE(norm(X + R * pp1 - bb1)) - (h02 - SQUARE(l1)) - 2 * l1*(x[2] + Bp1(2) - bb1(2));
     fvec[1] = SQUARE(norm(X + R * pp2 - bb2)) - (h02 - SQUARE(l2)) - 2 * l2*(x[2] + Bp2(2) - bb2(2));
     fvec[2] = SQUARE(norm(X + R * pp3 - bb3)) - (h02 - SQUARE(l3)) - 2 * l3*(x[2] + Bp3(2) - bb3(2));
     fvec[3] = SQUARE(norm(X + R * pp4 - bb4)) - (h02 - SQUARE(l4)) - 2 * l4*(x[2] + Bp4(2) - bb4(2));
     fvec[4] = SQUARE(norm(X + R * pp5 - bb5)) - (h02 - SQUARE(l5)) - 2 * l5*(x[2] + Bp5(2) - bb5(2));
     fvec[5] = SQUARE(norm(X + R * pp6 - bb6)) - (h02 - SQUARE(l6)) - 2 * l6*(x[2] + Bp6(2) - bb6(2));

     //if (_is_show_fk_debug)
        // cout << "f: " << fvec[0] << ", " << fvec[1] << ", " << fvec[2] << ", " << fvec[3] << ", " << fvec[4] << ", " << fvec[5] << endl << endl;

     return;
 }


 void Stewart_GMT::F_myfun(const int * n, const double * x, double * fvec, int * iflag)
 {
#pragma region
     if (_is_show_fk_debug)
         cout << "x in : " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << endl;

     //syms x1 x2 x3 a b g


     VectorXd P(6);
     P << x[0], x[1], x[2], x[3], x[4], x[5];
     Mat R = MathTool::poseRPY2SO3_2(P);
     Vector3d X = P.segment(0, 3);


     Vector3d bb1 = _bb.col(0);
     Vector3d bb2 = _bb.col(1);
     Vector3d bb3 = _bb.col(2);
     Vector3d bb4 = _bb.col(3);
     Vector3d bb5 = _bb.col(4);
     Vector3d bb6 = _bb.col(5);

     Vector3d pp1 = _pp.col(0);
     Vector3d pp2 = _pp.col(1);
     Vector3d pp3 = _pp.col(2);
     Vector3d pp4 = _pp.col(3);
     Vector3d pp5 = _pp.col(4);
     Vector3d pp6 = _pp.col(5);

     Vector3d Bp1 = R * pp1;
     Vector3d Bp2 = R * pp2;
     Vector3d Bp3 = R * pp3;
     Vector3d Bp4 = R * pp4;
     Vector3d Bp5 = R * pp5;
     Vector3d Bp6 = R * pp6;

     double l1 = _L(0);
     double l2 = _L(1);
     double l3 = _L(2);
     double l4 = _L(3);
     double l5 = _L(4);
     double l6 = _L(5);

#pragma endregion

     fvec[0] = SQUARE(MathTool::norm(X + R * pp1 - bb1)) - SQUARE(l1);
     fvec[1] = SQUARE(MathTool::norm(X + R * pp2 - bb2)) - SQUARE(l2);
     fvec[2] = SQUARE(MathTool::norm(X + R * pp3 - bb3)) - SQUARE(l3);
     fvec[3] = SQUARE(MathTool::norm(X + R * pp4 - bb4)) - SQUARE(l4);
     fvec[4] = SQUARE(MathTool::norm(X + R * pp5 - bb5)) - SQUARE(l5);
     fvec[5] = SQUARE(MathTool::norm(X + R * pp6 - bb6)) - SQUARE(l6);

     if (_is_show_fk_debug)
         cout << "f: " << fvec[0] << ", " << fvec[1] << ", " << fvec[2] << ", " << fvec[3] << ", " << fvec[4] << ", " << fvec[5] << endl << endl;

     return;
 }

double Stewart_GMT::newton(double x0, double tolx, int maxiter, double (*newton_evaluate_function)(double t, void *p), void *p)
{
    double h = 1e-4;
    double h2 = 2 * h;
    double tolfun = std::numeric_limits<double>::epsilon();

    //double xm = 19.0;
    //double c = 10;
    //double b = xm / (2 * MathPI*c);
    //double n = 5000;
    //double rx = xm *0.5;

    double fx = newton_evaluate_function(x0, p);

    double xx = x0;
    for (int i = 0; i < maxiter; i++)
    {
        double dfdx = (newton_evaluate_function(xx + h, p) - newton_evaluate_function(xx - h, p))/h2;
        double dx = -fx / dfdx;
        xx += dx;
        fx = newton_evaluate_function(xx, p);
        //DEBUG_PRINT_VALUE(dx);
        //DEBUG_PRINT_VALUE(fx);
        //DEBUG_PRINT_VALUE(abs(fx));
        //DEBUG_PRINT_VALUE(tolfun);
        //DEBUG_PRINT_VALUE(abs(dx));
        //DEBUG_PRINT_VALUE(tolx);
        //if (abs(fx) < tolfun || abs(dx) < tolx) break;
        if (abs(fx) < tolfun || abs(dx) < maxiter) break;
    }

    return xx;
}
double Stewart_GMT::spiral_time_eval_f1(double t, void *p)
{
    SpiralTEvaluateParameter *input = (SpiralTEvaluateParameter *)p;
    double b = input->b;
    double rx = input->rx;
    double n = input->n;

    return (b / 2)*(log(t + sqrt(t*t + 1)) + t * sqrt(t*t + 1)) - 2 * MathPI*rx*n;
}

double Stewart_GMT::spiral_time_eval_f2(double t, void *p)
{
    SpiralTEvaluateParameter *input = (SpiralTEvaluateParameter *)p;
    double b = input->b;
    double k = input->k;
    double v = input->v;

    return (b / 2)*(log((t*k + sqrt(t*t*k*k + 1)) / (t*(k - 1) + sqrt(t*t*SQUARE(k - 1) + 1)))
        + t * k*sqrt(t*t*k*k + 1) - t * (k - 1)*sqrt(t*t*SQUARE(k - 1) + 1)) - v*t;
}

double Stewart_GMT::GetPowerMeterData(double y,double z, double y1, double y2, double z1, double z2, Vector2d r, Vector2d u, Vector3d xyz, Vector3d toolTransly, Vector3d toolTranslz)
{
    double c1 = r(0)*y1 + (1 - r(0))*y2;
    double c2 = r(1)*z1 + (1 - r(1))*z2;
    Vector3d c = xyz + c1 * toolTransly + c2 * toolTranslz;
    double cy = c(1);
    double cz = c(2);
    double s1 = u(0);
    double s2 = u(1);

    return exp(-(SQUARE(y - cy) / SQUARE(s1) + SQUARE(z - cz) / SQUARE(s2)));
}


double Stewart_GMT::GetPowerMeterData_Wang(double y, double z, Vector2d r, Vector2d u)
{

    //double y1 = yzRange[0]; //y max
    //double y2 = yzRange[1];	//y min
    //double z1 = yzRange[2];	//z max
    //double z2 = yzRange[3];	//z min
    double cy = r(0);// *y1 + (1 - _r(0)) * y2;
    double cz = r(1);// * z1 + (1 - _r(1)) * z2;

    double s1 = u(0);
    double s2 = u(1);

    return exp(-(SQUARE(y - cy) / SQUARE(s1) + SQUARE(z - cz) / SQUARE(s2)));
}


//TODO:??+LL
bool Stewart_GMT::illegal_lenth(VectorXd PTest, enumStewartType stewart_type)
{
    VectorXd LL;
    if (stewart_type == TypeF)
        LL = Stewart_GMT::F_Stewart_IK(PTest);

    else if (stewart_type == TypeN) {
        NIKOutput nik_result = Stewart_GMT::N_Stewart_IK(PTest);

        Mat tarr = nik_result.alpha.array().square() + SQUARE(_h0) - nik_result.ls.array().square();
        auto illegal_lenth = (tarr.array() < 0);
        if (illegal_lenth.any()) return true;

        LL = nik_result.alpha.array() - tarr.array().sqrt();
        //DEBUG_PRINT_VALUE(nik_result.alpha)
        //DEBUG_PRINT_VALUE(tarr)
        //DEBUG_PRINT_VALUE( LL)
    }

    else {
        DEBUG_PRINT_MESSAGE("type unknow")
            return true;
    }

    auto illegal_lenth = (LL.array() < _Llow || LL.array() > _Lupp);

    if (illegal_lenth.any()) {
        //DEBUG_PRINT_VALUE(LL)
        return true;
    }
    else return false;
}

bool Stewart_GMT::illegal_lenth(vector<double> PTest, enumStewartType stewart_type)
{
    return illegal_lenth(stdVectortoVectorXD(PTest), stewart_type);
}
bool Stewart_GMT::illegal_lenthA12(VectorXd PTest)
{
    return false;
}
bool Stewart_GMT::check_stewart_type_and_set_parameter(enumStewartType stype)
{
    if (stype == TypeF) Stewart_GMT::set_F_Type_Stewart_DefaultParameter();
    else if (stype == TypeN) Stewart_GMT::set_N_Type_Stewart_DefaultParameter();
    else {
        DEBUG_PRINT_ERROR_MESSAGE("type unknow")
        return false;
    }
    return true;
}
//
//bool Stewart_GMT::F_out_of_range(VectorXd LL)
//{
//	auto illegal_lenth = (LL.array() < _Llow || LL.array() > _Lupp);
//
//	if (illegal_lenth.any())return true;
//	else return false;
//}



#pragma endregion


#pragma region tool

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
    return ZYZ2SO3(Pose.segment(3,3));
}

//????1*6???
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

vector<double> MathTool::randomFactory(double min, double max, int how_many)
{
    /* ???? */
    std::random_device rd;

    /* ??????? */
    std::mt19937 generator(rd());

    std::uniform_real_distribution<float> unif(min, max);

    vector<double> radom_numbers;
    for (int i = 0; i < how_many; i++)
        radom_numbers.push_back(unif(generator));

    /* ??????? */
    if (rd.entropy() < 10) {//??????,??entropy?32
        DEBUG_PRINT_MESSAGE("warning! random seed runing out");
    }

    return radom_numbers;
}

double MathTool::getRandomNumber(double min, double max)
{
    if (random_numbers.size() < 1)
        random_numbers = randomFactory(0, 1, 100);

    double first = random_numbers[0];
    random_numbers.erase(random_numbers.begin());

    double scale = max - min;

    return first * scale + min;
}

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
    Matrix3d so3 = ZYZ2SO3_2(pose.segment(3,3));
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

    //TODO:??matlab code????????
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
    //????????matlab code,?????????????

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

            omega << 0, 0, 0 ;
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



    //TODO:??matlab code????????
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

inline Matrix3d MathTool::rotZ(double theta)
{
    Matrix3d r;
    r << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    return r;
}

inline Matrix3d MathTool::rotY(double theta)
{
    Matrix3d r;
    r << cos(theta),  0, sin(theta),
        0, 1, 0,
        -sin(theta), 0, cos(theta);
    return r;
}

inline Matrix3d MathTool::rotX(double theta)
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

inline double MathTool::AxisAng3(Mat expc3, Mat out_omghat)
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
    if(Tf == 0)
        return 0;
    double A = t / Tf;
    double s = 3 * A * A - 2 * A * A * A;
    return s;
}

double MathTool::QuinticTimeScaling(double Tf, double t)
{
    // s = 10 * (t / Tf) ^ 3 - 15 * (t / Tf) ^ 4 + 6 * (t / Tf) ^ 5;
    if(Tf == 0)
        return 0;
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

inline vector<double> MathTool::vectorXDtoStdVector(VectorXd v)
{
    vector<double> vd(v.data(),v.data()+v.size());
    return vd;
}

inline VectorXd MathTool::stdVectortoVectorXD(vector<double> v)
{
    return Eigen::Map<Eigen::VectorXd>(v.data(), v.size());
}

inline Matrix3d MathTool::VecToso3(Mat omg)
{
    Matrix3d so3Mat;
    // so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    so3Mat << 0, -omg(2, 0), omg(1, 0),
        omg(2, 0), 0, -omg(0, 0),
        -omg(1, 0), omg(0, 0), 0;

    return so3Mat;
}

inline Matrix4d MathTool::rp2t(Matrix3d r, Vector3d p)
{
    Matrix4d t;
    t << r, p, 0, 0, 0, 1;
    return t;
}
inline Matrix3d MathTool::t2r(Matrix4d t)
{
    MatrixXd R = t.block(0, 0, 3, 3);
    return R;
}
inline Vector3d MathTool::t2p(Matrix4d t)
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




#pragma endregion
