#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <math.h>
#include <cmath>
#include <cassert>
#include <cmath>

struct Vector3D {
    double x, y, z;

    // construction
    Vector3D() {}
    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
    static Vector3D zero() { return Vector3D(0.0, 0.0, 0.0); }
    static Vector3D ex() { return Vector3D(1.0, 0.0, 0.0); }
    static Vector3D ey() { return Vector3D(0.0, 1.0, 0.0); }
    static Vector3D ez() { return Vector3D(0.0, 0.0, 1.0); }

    // access
    double length() const { return sqrt(x*x+y*y+z*z); }
    double lengthSquared() const { return x*x+y*y+z*z; }
    bool isfinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }

    Vector3D operator/(const double& rhs) const { return Vector3D(x/rhs, y/rhs, z/rhs); }
    Vector3D normalized() const { return (*this)/length(); }
    void normalize() { (*this)/=length(); }
    static double dotProduct(const Vector3D& lhs, const Vector3D& rhs);
    static Vector3D crossProduct(const Vector3D& lhs, const Vector3D& rhs);

    Vector3D& operator+=(const Vector3D& rhs);
    Vector3D& operator-=(const Vector3D& rhs);
    Vector3D& operator*=(const double& rhs);
    Vector3D& operator/=(const double& rhs);

    // unary minus
    Vector3D operator-() const;

    // v: arbitrary vector (assumed to be nonzero)
    // if v is normalized, the other vectors will be normalized too
    void findOrthogonalVectors(Vector3D* orthogonalVector1, Vector3D* orthogonalVector2);
};

Vector3D operator+(const Vector3D& lhs, const Vector3D& rhs);
Vector3D operator-(const Vector3D& lhs, const Vector3D& rhs);
Vector3D operator*(const double& lhs, const Vector3D& rhs);
Vector3D operator*(const Vector3D& lhs, const double& rhs);
bool operator==(const Vector3D& lhs, const Vector3D& rhs);
bool operator!=(const Vector3D& lhs, const Vector3D& rhs);

bool isfinite(Vector3D v);


#endif // VECTOR3D_H
