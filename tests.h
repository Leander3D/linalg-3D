#include "vector3d.h"
#include "matrix3x3d.h"
#include "quaternion.h"

// tests return nothing, they crash if they fail
namespace TestLinalg3D {
    // Random test data generators
    double URD(double low, double high); // Uniform random distribution
    Vector3D URD(double maxMagnitude);
    Vector3D randomDirection(); // length: 1
    Quaternion randomOrientation();
    Vector3D randomVector(double low, double high); // low, high per component
    Matrix3x3D randomMatrix(double low, double high); // low, high per component

    // make sure that matrix products are associative. (A*B)*C == A*(B*C)
    void testMatrixProducts();

    // matrix times its inverse must equal the identity matrix
    void testMatrixInverse();

    // rotate a quaternion by a known vector using rotateByAccurate().
    // Then retrieve this vector using getRotationVectorAccurate() and compare
    void testQuaternionRotation();

    // vector rotated by a quaternion must be the same as the vector rotated using the respective rotation matrix
    void testRotationMatrix();

    // a complex combination of matrix, vector and quaternion operations
    void testCombinedMatrixOperations();

    void runAll();
}