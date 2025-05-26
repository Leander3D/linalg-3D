#include "vector3d.h"



double Vector3D::dotProduct(const Vector3D &lhs, const Vector3D &rhs){
    return lhs.x*rhs.x +
            lhs.y*rhs.y +
            lhs.z*rhs.z;
}

Vector3D Vector3D::crossProduct(const Vector3D &lhs, const Vector3D &rhs){
    return Vector3D(lhs.y*rhs.z - lhs.z*rhs.y,
                    lhs.z*rhs.x - lhs.x*rhs.z,
                    lhs.x*rhs.y - lhs.y*rhs.x);
}

Vector3D &Vector3D::operator+=(const Vector3D &rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

Vector3D &Vector3D::operator-=(const Vector3D &rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

Vector3D &Vector3D::operator*=(const double &rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
}

Vector3D &Vector3D::operator/=(const double &rhs){
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
}

Vector3D Vector3D::operator-() const {
    return Vector3D(-x, -y, -z);
}

Vector3D operator+(const Vector3D &lhs, const Vector3D &rhs){
    return Vector3D(lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z);
}

Vector3D operator-(const Vector3D &lhs, const Vector3D &rhs){
    return Vector3D(lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z);
}

Vector3D operator*(const double &lhs, const Vector3D &rhs){
    return Vector3D(lhs*rhs.x, lhs*rhs.y, lhs*rhs.z);
}

Vector3D operator*(const Vector3D &lhs, const double &rhs) {
    return Vector3D(lhs.x*rhs, lhs.y*rhs, lhs.z*rhs);
}

bool operator==(const Vector3D &lhs, const Vector3D &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Vector3D &lhs, const Vector3D &rhs) {
    return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z;
}

bool isfinite(Vector3D v) { return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);}

void Vector3D::findOrthogonalVectors(Vector3D *orthogonalVector1, Vector3D *orthogonalVector2){
    Vector3D basisVector;
    if(abs(x) < abs(y) && abs(x) < abs(z))
        basisVector = Vector3D::ex();
    else if(abs(y) < abs(x) && abs(y) < abs(z))
        basisVector = Vector3D::ey();
    else
        basisVector = Vector3D::ez();

    *orthogonalVector1 = Vector3D::crossProduct(*this, basisVector);
    *orthogonalVector2 = Vector3D::crossProduct(*this, *orthogonalVector1);
}
