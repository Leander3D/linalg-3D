#ifndef QUATERNION_H
#define QUATERNION_H

#include "vector3d.h"
#include "matrix3x3d.h"

struct Quaternion {
    double scalar, x, y, z;

    // construction
    Quaternion() : scalar(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double scalar, double x, double y, double z) : scalar(scalar), x(x), y(y), z(z) {}
    Quaternion(double scalar, Vector3D vector) : scalar(scalar), x(vector.x), y(vector.y), z(vector.z) {}
    static Quaternion fromAxisAndAngle(Vector3D axis, double angle_degrees);

    // access
    Vector3D vector() const { return Vector3D(x, y, z); }
    void getAxisAndAngle(Vector3D* axis, double* angle_radians) const;
    double length() const;
    static Vector3D getRotationVectorFast(Quaternion q1, Quaternion q2);
    static Vector3D getRotationVectorAccurate(Quaternion q1, Quaternion q2);

    // manipulation
    Quaternion conjugated() const;
    Quaternion inverted() const;
    Quaternion invertedAssumingNormalized() const;
    void normalize();
    Quaternion normalized();
    void rotateByFast(Vector3D rotationVectorGlobal);
    void rotateByAccurate(Vector3D rotationVectorGlobal);

    Matrix3x3D rotationMatrix() {
        // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#From_a_quaternion_to_an_orthogonal_matrix
        double a = scalar;
        double b = x, c = y, d = z;
        return Matrix3x3D(a*a + b*b - c*c - d*d,    2.0*b*c - 2.0*a*d,      2.0*b*d + 2.0*a*c,
                          2.0*b*c + 2.0*a*d,        a*a - b*b + c*c - d*d,  2.0*c*d - 2.0*a*b,
                          2.0*b*d - 2.0*a*c,        2.0*c*d + 2.0*a*b,      a*a - b*b - c*c +d*d);
    }

    Quaternion& operator+=(const Quaternion& rhs);
    Quaternion& operator-=(const Quaternion& rhs);
    Quaternion& operator/=(const double& rhs);
};

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);
Vector3D operator*(const Quaternion& q, const Vector3D& v);
Quaternion operator*(const double& lhs, const Quaternion& rhs);
Quaternion operator/(const Quaternion& lhs, const double& rhs);
bool isfinite(Quaternion q);

#endif // QUATERNION_H
