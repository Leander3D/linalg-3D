#include "matrix3x3d.h"


Matrix3x3D Matrix3x3D::identity() {
    return Matrix3x3D(1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0);
}

double Matrix3x3D::norm() {
    double sum = 0.0;
    for(int c=0;c<9;c++) sum += a[c]*a[c];
    return sqrt(sum);
}

Matrix3x3D Matrix3x3D::transposed(){
    Matrix3x3D result;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            *(result.coefficient(i, j)) = getCoefficient(j, i);
        }
    }
    return result;
}

Matrix3x3D Matrix3x3D::inverted(){
    Matrix3x3D inverse;
    // https://matrixcalc.org/en/#inverse({{a_0,a_3,a_6},{a_1,a_4,a_7},{a_2,a_5,a_8}})
    double det = a[2]*a[4]*a[6] - a[1]*a[5]*a[6] - a[2]*a[3]*a[7] + a[0]*a[5]*a[7] + a[1]*a[3]*a[8] -a[0]*a[4]*a[8];
    *inverse.coefficient(0, 0) = ( a[5]*a[7]-a[4]*a[8])/det;   *inverse.coefficient(0, 1) =(-a[5]*a[6]+a[3]*a[8])/det;   *inverse.coefficient(0, 2) = ( a[4]*a[6]-a[3]*a[7])/det;
    *inverse.coefficient(1, 0) = (-a[2]*a[7]+a[1]*a[8])/det;   *inverse.coefficient(1, 1) =( a[2]*a[6]-a[0]*a[8])/det;   *inverse.coefficient(1, 2) = (-a[1]*a[6]+a[0]*a[7])/det;
    *inverse.coefficient(2, 0) = ( a[2]*a[4]-a[1]*a[5])/det;   *inverse.coefficient(2, 1) =(-a[2]*a[3]+a[0]*a[5])/det;   *inverse.coefficient(2, 2) = ( a[1]*a[3]-a[0]*a[4])/det;
    return inverse;
}

Matrix3x3D Matrix3x3D::operator-() const {
    return Matrix3x3D(
        -getCoefficient(0, 0), -getCoefficient(0, 1), -getCoefficient(0, 2),
        -getCoefficient(1, 0), -getCoefficient(1, 1), -getCoefficient(1, 2),
        -getCoefficient(2, 0), -getCoefficient(2, 1), -getCoefficient(2, 2)
    );
}

Matrix3x3D &Matrix3x3D::operator+=(const Matrix3x3D &rhs) {
    for(int c=0;c<9;c++) a[c] += rhs.a[c];
    return *this;
}

Matrix3x3D operator+(const Matrix3x3D &lhs, const Matrix3x3D &rhs) {
    Matrix3x3D result;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            *(result.coefficient(i, j)) = lhs.getCoefficient(i, j) + rhs.getCoefficient(i, j);
        }
    }
    return result;
}
Matrix3x3D operator-(const Matrix3x3D &lhs, const Matrix3x3D &rhs) {
    Matrix3x3D result;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            *(result.coefficient(i, j)) = lhs.getCoefficient(i, j) - rhs.getCoefficient(i, j);
        }
    }
    return result;
}
Matrix3x3D operator*(const Matrix3x3D &lhs, const Matrix3x3D &rhs) {
    Matrix3x3D result;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            *(result.coefficient(i, j)) =
                    lhs.getCoefficient(i, 0)*rhs.getCoefficient(0, j) +
                    lhs.getCoefficient(i, 1)*rhs.getCoefficient(1, j) +
                    lhs.getCoefficient(i, 2)*rhs.getCoefficient(2, j);
        }
    }
    return result;
}

Vector3D operator*(const Matrix3x3D &lhs, const Vector3D &rhs) {
    return Vector3D(lhs.getCoefficient(0, 0)*rhs.x + lhs.getCoefficient(0, 1)*rhs.y + lhs.getCoefficient(0, 2)*rhs.z,
                    lhs.getCoefficient(1, 0)*rhs.x + lhs.getCoefficient(1, 1)*rhs.y + lhs.getCoefficient(1, 2)*rhs.z,
                    lhs.getCoefficient(2, 0)*rhs.x + lhs.getCoefficient(2, 1)*rhs.y + lhs.getCoefficient(2, 2)*rhs.z);
}

Matrix3x3D operator*(const double &lhs, const Matrix3x3D &rhs) {
    return Matrix3x3D(lhs*rhs.getCoefficient(0, 0), lhs*rhs.getCoefficient(0, 1), lhs*rhs.getCoefficient(0, 2),
                      lhs*rhs.getCoefficient(1, 0), lhs*rhs.getCoefficient(1, 1), lhs*rhs.getCoefficient(1, 2),
                      lhs*rhs.getCoefficient(2, 0), lhs*rhs.getCoefficient(2, 1), lhs*rhs.getCoefficient(2, 2));
}
