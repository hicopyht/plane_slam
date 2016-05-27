#include "utils.h"

void matrixTF2Eigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e)
{
    e.matrix()(0,0) = t[0][0];
    e.matrix()(0,1) = t[0][1];
    e.matrix()(0,2) = t[0][2];
    e.matrix()(1,0) = t[1][0];
    e.matrix()(1,1) = t[1][1];
    e.matrix()(1,2) = t[1][2];
    e.matrix()(2,0) = t[2][0];
    e.matrix()(2,1) = t[2][1];
    e.matrix()(2,2) = t[2][2];
}

Eigen::Matrix3d matrixTF2Eigen(const tf::Matrix3x3 &t)
{
    Eigen::Matrix3d m33;
    matrixTF2Eigen(t, m33);
    return m33;
}

void matrixEigen2TF( const Eigen::Matrix3d &e, tf::Matrix3x3 &t)
{
    t[0][0] = e.matrix()(0,0);
    t[0][1] = e.matrix()(0,1);
    t[0][2] = e.matrix()(0,2);
    t[1][0] = e.matrix()(1,0);
    t[1][1] = e.matrix()(1,1);
    t[1][2] = e.matrix()(1,2);
    t[2][0] = e.matrix()(2,0);
    t[2][1] = e.matrix()(2,1);
    t[2][2] = e.matrix()(2,2);
}

tf::Matrix3x3 matrixEigen2TF(const Eigen::Matrix3d &m33)
{
    tf::Matrix3x3 t;
    matrixEigen2TF( m33 , t );
    return t;
}
