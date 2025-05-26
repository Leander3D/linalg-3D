#ifndef MATRIX3X3D_H
#define MATRIX3X3D_H

#include "vector3d.h"

struct Matrix3x3D {
    Matrix3x3D() {}
    Matrix3x3D(double a00, double a01, double a02,
               double a10, double a11, double a12,
               double a20, double a21, double a22) {
        *(coefficient(0, 0)) = a00; *(coefficient(0, 1)) = a01; *(coefficient(0, 2)) = a02;
        *(coefficient(1, 0)) = a10; *(coefficient(1, 1)) = a11; *(coefficient(1, 2)) = a12;
        *(coefficient(2, 0)) = a20; *(coefficient(2, 1)) = a21; *(coefficient(2, 2)) = a22;
    }

    void zero() { for(int c=0;c<9;c++) a[c] = 0.0f; }

    static Matrix3x3D identity();

    double norm();
    Matrix3x3D transposed();
    Matrix3x3D inverted();

    double* coefficient(int i, int j) { return a +3*j + i; }
    double getCoefficient(int i, int j) const { return a[3*j + i]; }

    Matrix3x3D& operator+=(const Matrix3x3D& rhs);

private:
    double a[3*3];
};

Matrix3x3D operator+(const Matrix3x3D& lhs, const Matrix3x3D& rhs);
Matrix3x3D operator-(const Matrix3x3D& lhs, const Matrix3x3D& rhs);
Matrix3x3D operator*(const Matrix3x3D& lhs, const Matrix3x3D& rhs);
Vector3D operator*(const Matrix3x3D& lhs, const Vector3D& rhs);
Matrix3x3D operator*(const double& lhs, const Matrix3x3D& rhs);

#endif // MATRIX3X3D_H
