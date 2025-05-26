#include "quaternion.h"


Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs){
    double yy = (lhs.scalar - lhs.y) * (rhs.scalar + rhs.z);
    double zz = (lhs.scalar + lhs.y) * (rhs.scalar - rhs.z);
    double ww = (lhs.z      + lhs.x) * (rhs.x      + rhs.y);
    double xx = ww + yy + zz;
    double qq = 0.5 * (xx + (lhs.z - lhs.x) * (rhs.x - rhs.y));

    double w = qq - ww + (lhs.z -       lhs.y       ) * (rhs.y      - rhs.z     );
    double x = qq - xx + (lhs.x +       lhs.scalar  ) * (rhs.x      + rhs.scalar);
    double y = qq - yy + (lhs.scalar -  lhs.x       ) * (rhs.y      + rhs.z     );
    double z = qq - zz + (lhs.z +       lhs.y       ) * (rhs.scalar - rhs.x     );

    return Quaternion(w, x, y, z);
}

Quaternion Quaternion::inverted() const {
    double len = length();
    return Quaternion(scalar/len, -x/len, -y/len, -z/len);
}

Quaternion Quaternion::invertedAssumingNormalized() const {
    return Quaternion(scalar, -x, -y, -z);
}

void Quaternion::normalize() {
    *this /= length();
}

Quaternion Quaternion::normalized() {
    return (*this)/length();
}

void Quaternion::rotateByFast(Vector3D rotationVectorGlobal) {
    // Mattias Müllers paper:
    // q <- q + h (1/2) [omega_x, omega_y, omega_z, 0]q, then normalize
    *this += 0.5 * (Quaternion(0.0, rotationVectorGlobal) * (*this));
    normalize();
}

void Quaternion::rotateByAccurate(Vector3D rotationVectorGlobal){
    // angle (radians)
    double alpha = rotationVectorGlobal.length();
    Vector3D axis = rotationVectorGlobal / alpha;
    if(isfinite(axis)){
        double s = std::sin(0.5 * alpha);
        double c = std::cos(0.5 * alpha);
        Quaternion rotation = Quaternion(c, axis.x*s, axis.y*s, axis.z*s).normalized();
        *this = rotation * (*this);
    }
    // if axis is NaN, assume that rotationVectorGlobal is zero, do nothing
    //else {
    //    // default to the fast formula
    //    *this += 0.5 * (Quaternion(0.0, rotationVectorGlobal) * (*this));
    //}
    normalize();
}

Quaternion &Quaternion::operator+=(const Quaternion &rhs) {
    scalar += rhs.scalar;
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

Quaternion &Quaternion::operator-=(const Quaternion &rhs){
    scalar -= rhs.scalar;
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

Quaternion &Quaternion::operator/=(const double &rhs){
    scalar /= rhs;
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
}

Quaternion Quaternion::fromAxisAndAngle(Vector3D axis, double angle_degrees){
    axis.normalize();
    double a = M_PI*(0.5 * angle_degrees)/180.0;
    double s = std::sin(a);
    double c = std::cos(a);
    return Quaternion(c, axis.x*s, axis.y*s, axis.z*s).normalized();
}

void Quaternion::getAxisAndAngle(Vector3D *axis, double *angle_radians) const {
    const double length = vector().length();
    *axis = vector()/length;
    *angle_radians = 2.0 * std::atan2(length, scalar);
}

double Quaternion::length() const {
    return sqrt(scalar*scalar + x*x + y*y + z*z);
}

Vector3D Quaternion::getRotationVectorFast(Quaternion q1, Quaternion q2) {
    Quaternion deltaQ = q2 * q1.inverted();
    // Stable but inaccurate! Matthias Müller:
    // omega <- 2[delta_q_x, delta_q_y, delta_q_z]/h
    Vector3D result = 2.0 * deltaQ.vector();
    // omega <- delta_q_w >= 0 ? omega : -omega
    if(deltaQ.scalar < 0.0) result *= -1.0;
    return result;
}

Vector3D Quaternion::getRotationVectorAccurate(Quaternion q1, Quaternion q2) {
    Quaternion deltaQ = q2 * q1.inverted();
    Vector3D deltaQAxis;
    double deltaQAngle;
    deltaQ.getAxisAndAngle(&deltaQAxis, &deltaQAngle);
    if(isfinite(deltaQAxis) && isfinite(deltaQAngle))
        return deltaQAngle * deltaQAxis;
    else {
        // default to the "fast" formula
        Vector3D result = 2.0 * deltaQ.vector();
        if(deltaQ.scalar < 0.0) result *= -1.0;
        return result;
    }
}

Quaternion Quaternion::conjugated() const {
    return Quaternion(scalar, -x, -y, -z);
}

Vector3D operator*(const Quaternion &q, const Vector3D &v){
    //return (q * Quaternion(0.0, v) * q.conjugated()).vector();

    // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    // Extract the vector part of the quaternion
    Vector3D u = q.vector();

    // Extract the scalar part of the quaternion
    double s = q.scalar;

        // Do the math
    return 2.0 * Vector3D::dotProduct(u, v) * u
              + (s*s - Vector3D::dotProduct(u, u)) * v
              + 2.0 * s * Vector3D::crossProduct(u, v);
}

Quaternion operator*(const double &lhs, const Quaternion &rhs) {
    return Quaternion(lhs*rhs.scalar, lhs*rhs.vector());
}

Quaternion operator+(const Quaternion &lhs, const Quaternion &rhs) {
    return Quaternion(lhs.scalar+rhs.scalar, lhs.vector()+rhs.vector());
}

Quaternion operator/(const Quaternion &lhs, const double &rhs) {
    return Quaternion(lhs.scalar/rhs, lhs.vector()/rhs);
}

bool isfinite(Quaternion q) { return isfinite(q.scalar) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z); }
