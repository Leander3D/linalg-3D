#include "tests.h"
#include <cassert>

double TestLinalg3D::URD(double low, double high){
    double rand01 = 0.0001*double(rand()%10000);
    return low + (high-low)*rand01;
}

Vector3D TestLinalg3D::URD(double maxMagnitude){
    Vector3D result;
    do { result = Vector3D(URD(-maxMagnitude, maxMagnitude), URD(-maxMagnitude, maxMagnitude), URD(-maxMagnitude, maxMagnitude)); } while (result.length() > maxMagnitude);
    return result;
}

Vector3D TestLinalg3D::randomDirection(){
    Vector3D result;
    do { result = Vector3D(URD(-1.0, 1.0), URD(-1.0, 1.0), URD(-1.0, 1.0)).normalized(); } while(!isfinite(result));
    return result;
}

Quaternion TestLinalg3D::randomOrientation(){
    Quaternion result;
    do { result = Quaternion(URD(-1.0, 1.0), URD(-1.0, 1.0), URD(-1.0, 1.0), URD(-1.0, 1.0)).normalized(); } while (!isfinite(result));
    return result;
}

Vector3D TestLinalg3D::randomVector(double low, double high){
    return Vector3D(URD(low, high), URD(low, high), URD(low, high));
}

Matrix3x3D TestLinalg3D::randomMatrix(double low, double high){
    return Matrix3x3D(URD(low, high), URD(low, high), URD(low, high),
                      URD(low, high), URD(low, high), URD(low, high),
                      URD(low, high), URD(low, high), URD(low, high));
}



void TestLinalg3D::testMatrixProducts() {
    for(int scenario = 0; scenario < 10; scenario++){
        Matrix3x3D Ma = randomMatrix(-10.0, 10.0);
        Matrix3x3D Mb = randomMatrix(-10.0, 10.0);
        Vector3D v = randomVector(-10.0, 10.0);
        Vector3D associativeA = (Ma * Mb) * v;
        Vector3D associativeB = Ma * (Mb * v);
        double error = (associativeB-associativeA).length();
        assert(error < 1e-12);
    }
}

void TestLinalg3D::testMatrixInverse(){
    // random matrix
    Matrix3x3D m(URD(-10.0, 10.0), URD(-10.0, 10.0), URD(-10.0, 10.0),
                 URD(-10.0, 10.0), URD(-10.0, 10.0), URD(-10.0, 10.0),
                 URD(-10.0, 10.0), URD(-10.0, 10.0), URD(-10.0, 10.0));
    // m * m^-1 should be extremely close to identity matrix
    double error = ((m * m.inverted()) - Matrix3x3D::identity()).norm();
    assert(error < 1e-15);
}

void TestLinalg3D::testQuaternionRotation(){
    for(int scenario = 0; scenario < 100; scenario++){
        Vector3D rotationVectorIn = URD(-3.0, 3.0) * randomDirection();
        Quaternion start = randomOrientation();
        Quaternion end = start; end.rotateByAccurate(rotationVectorIn);
        Vector3D rotationVectorOut = Quaternion::getRotationVectorAccurate(start, end);
        double error = (rotationVectorOut - rotationVectorIn).length();
        assert(error < 1e-12);
    }
}

void TestLinalg3D::testRotationMatrix(){
    for(int scenario = 0; scenario < 100; scenario++){
        Vector3D v = randomDirection();
        Quaternion q = randomOrientation();
        Vector3D vRotatedQuaternion = q * v;
        Vector3D vRotatedMatrix = q.rotationMatrix() * v;
        double error = (vRotatedMatrix - vRotatedQuaternion).length();
        assert(error < 1e-15);
    }
}

void TestLinalg3D::testCombinedMatrixOperations()
{
    for(int scenario = 0; scenario < 100; scenario++){
        // I scales but does not rotate (like an inertia tensor in main axis direction)
        double IxxIyy = URD(0.1, 10.0);
        double Izz = URD(0.1, 10.0);
        Matrix3x3D I(IxxIyy,    0.0,    0.0,
                     0.0,       IxxIyy, 0.0,
                     0.0,       0.0,    Izz);
        Quaternion q = randomOrientation();
        Matrix3x3D rot = q.rotationMatrix();
        Vector3D testDirection = q * Vector3D::ez();
        Vector3D vectorIn = URD(-10.0, 10.0) * testDirection;
        // this is used for position based braking with friction
        Vector3D vectorOut = (rot * I * rot.transposed()).inverted().inverted() * vectorIn;
        Vector3D vectorIntoI = I * (rot.transposed()*vectorIn);
        Vector3D errorVector = Vector3D::crossProduct(vectorIn, vectorOut);
        double errorMag = errorVector.length();
        assert(errorMag < 1e-12);
    }
}

void TestLinalg3D::runAll() {
    testMatrixProducts();
    testMatrixInverse();
    testQuaternionRotation();
    testRotationMatrix();
    testCombinedMatrixOperations();
}
