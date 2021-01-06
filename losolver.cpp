/*
    losolver.cpp (part of GNSS-Stylus)
    Copyright (C) 2020-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "losolver.h"
#include <iostream>
#include <QtDebug>

#if 0
inline QDebug operator<<(QDebug dbg, const Eigen::Matrix3d& m)
{
    dbg << QString::number(m(0,0),'f',3) << ", " << QString::number(m(0,1),'f',3) << ", " << QString::number(m(0,2),'f',3) << "\n" <<
           QString::number(m(1,0),'f',3) << ", " << QString::number(m(1,1),'f',3) << ", " << QString::number(m(1,2),'f',3) << "\n" <<
           QString::number(m(2,0),'f',3) << ", " << QString::number(m(2,1),'f',3) << ", " << QString::number(m(2,2),'f',3);
    return dbg.space();
}

inline QDebug operator<<(QDebug dbg, const Eigen::Transform<double, 3, Eigen::Affine> t)
{
    Eigen::Matrix4d m = t.matrix();

    dbg << QString::number(m(0,0),'f',3) << ", " << QString::number(m(0,1),'f',3) << ", " << QString::number(m(0,2),'f',3) << ", " << QString::number(m(0,3),'f',3) << "\n" <<
           QString::number(m(1,0),'f',3) << ", " << QString::number(m(1,1),'f',3) << ", " << QString::number(m(1,2),'f',3) << ", " << QString::number(m(1,3),'f',3) << "\n" <<
           QString::number(m(2,0),'f',3) << ", " << QString::number(m(2,1),'f',3) << ", " << QString::number(m(2,2),'f',3) << ", " << QString::number(m(2,3),'f',3) << "\n" <<
           QString::number(m(3,0),'f',3) << ", " << QString::number(m(3,1),'f',3) << ", " << QString::number(m(3,2),'f',3) << ", " << QString::number(m(3,3),'f',3);
    return dbg.space();
}
#endif


LOSolver::LOSolver()
{

}

void LOSolver::init(void)
{
    errorCode = ERROR_INVALID_REFERENCE_POINTS;
    refPointsValid = false;
}

bool LOSolver::setReferencePoints(const Eigen::Vector3d refPoints[3])
{
    this->refPoints[0] = refPoints[0];
    this->refPoints[1] = refPoints[1];
    this->refPoints[2] = refPoints[2];

    return calculateReferenceBasis();
}

bool LOSolver::calculateReferenceBasis(void)
{
    errorCode = ERROR_NONE;

    Eigen::Vector3d vecAtoB = refPoints[1] - refPoints[0];
    Eigen::Vector3d vecAtoC = refPoints[2] - refPoints[0];
    Eigen::Vector3d vecBtoC = refPoints[2] - refPoints[1];

    refDistAB = vecAtoB.norm();
    refDistAC = vecAtoC.norm();
    refDistBC = vecBtoC.norm();

    refCentroid = (refPoints[0] + refPoints[1] + refPoints[2]) / 3;
    Eigen::Vector3d vecZDirection = vecAtoB.cross(vecAtoC);

    if ((refDistAB == 0) ||
            (refDistAC == 0) ||
            (refDistBC == 0) ||
            (vecZDirection.norm() == 0))
    {
        // Some of the points are either identical or lie on the same line
        refPointsValid = false;
        errorCode = ERROR_INVALID_REFERENCE_POINTS;
        return false;
    }

    // As no point should affect the calculation more than others, basis vector X (and indirectly Y)
    // is calculated as follows (degrees used in this explanation):
    // "Imaginary" vectors from centroid, lying on the plane defined by points A, B and C and
    // separated by 120 degrees (1/3 of full round) are rotated around the centroid so that
    // the sum of the ("signed") angles between these and the vectors to real corners of the
    // triangle are zero.
    // "Imaginary" vector initially pointing to point A and rotated as explained above
    // is then used as a basis vector X.

    // Calculate reference basis vectors so that:
    // X points from centroid to the direction calculated as described above.
    // Y is in right angle with the unit vector X and lies on the plane defined by points A, B and C,
    // Z is in right angle with both unit vectors X and Y
    // (and therefore Z is perpendicular to the plane defined by points A, B and C)

    Eigen::Vector3d unitVecFromCentroidTowardsA = (refPoints[0] - refCentroid).normalized();
    Eigen::Vector3d unitVecFromCentroidTowardsB = (refPoints[1] - refCentroid).normalized();
    Eigen::Vector3d unitVecFromCentroidTowardsC = (refPoints[2] - refCentroid).normalized();

    double angleBetweenVectorsAndBFromCentroid = acos(unitVecFromCentroidTowardsB.dot(unitVecFromCentroidTowardsA)); // * 360. / (2 * M_PI);
    double angleBetweenVectorsAndCFromCentroid = acos(unitVecFromCentroidTowardsC.dot(unitVecFromCentroidTowardsA)); // * 360. / (2 * M_PI);

    // No "120 degree" separation is needed here as the different signs cause "cancellation".
    double angleError = angleBetweenVectorsAndBFromCentroid - angleBetweenVectorsAndCFromCentroid;

    // Basis vector Z is also the axis to turn the "imaginary" vectors from centroid around.
    Eigen::Vector3d unitVecZ = vecZDirection.normalized();

    // Turn the "imaginary" vector from the centoid around so that the sum of angle differences will be zero.
    Eigen::Vector3d unitVecX = Eigen::AngleAxisd(angleError / 3, unitVecZ) * unitVecFromCentroidTowardsA;

    // This results in right-handed system. Handedness only affects the debug
    // output from getTransformMatrix (if you make the same change to getTransformMatrix-function)
    Eigen::Vector3d unitVecY = vecZDirection.cross(unitVecX).normalized();
    // Left-handed version:
    // Eigen::Vector3d unitVecY = unitVecX.cross(vecZDirection).normalized();

    // This is constructed as transposed, but as it's orthogonal, it equals the inverse:
    refBasisInverse <<
                unitVecX(0), unitVecX(1), unitVecX(2),
                unitVecY(0), unitVecY(1), unitVecY(2),
                unitVecZ(0), unitVecZ(1), unitVecZ(2);

    refPointsValid = true;

#if 0
    // Test code. All angle "errors" added together should be close to zero.
    Eigen::Vector3d refDirA = unitVecX;
    double angleErrorA = acos(refDirA.dot(unitVecCentroidToA)) * (refDirA.cross(unitVecCentroidToA).dot(unitVecZ) < 0 ? 1 : -1);

    Eigen::Vector3d refDirB = Eigen::AngleAxisd(-2 * M_PI / 3, unitVecZ) * unitVecX;
    double angleErrorB = acos(refDirB.dot(unitVecCentroidToB)) * (refDirB.cross(unitVecCentroidToB).dot(unitVecZ) < 0 ? 1 : -1);

    Eigen::Vector3d refDirC = Eigen::AngleAxisd(2 * M_PI / 3, unitVecZ) * unitVecX;
    double angleErrorC = acos(refDirC.dot(unitVecCentroidToC)) * (refDirC.cross(unitVecCentroidToC).dot(unitVecZ) < 0 ? 1 : -1);

    double angleErrorTotal = angleErrorA + angleErrorB + angleErrorC;

    Q_ASSERT(fabs(angleErrorTotal) < 0.001);
#endif

    return true;
}

bool LOSolver::setPoints(const Eigen::Vector3d points[3])
{
    // Error checking is done when operating with the points
    errorCode = ERROR_NONE;

    this->points[0] = points[0];
    this->points[1] = points[1];
    this->points[2] = points[2];

    return true;
}

bool LOSolver::getTransformMatrix(Eigen::Transform<double, 3, Eigen::Affine>& transform,
                                  Eigen::Transform<double, 3, Eigen::Affine>* orientationTransform_Debug)
{
    if (!refPointsValid)
    {
        errorCode = ERROR_INVALID_REFERENCE_POINTS;
        return false;
    }
    else
    {
        errorCode = ERROR_NONE;
    }

    Eigen::Vector3d vecAtoB = points[1] - points[0];
    Eigen::Vector3d vecAtoC = points[2] - points[0];
    Eigen::Vector3d vecBtoC = points[2] - points[1];

    double distAB = vecAtoB.norm();
    double distAC = vecAtoC.norm();
    double distBC = vecBtoC.norm();

    // TODO: Add comparison of distances with the reference basis points

    Eigen::Vector3d centroid = (points[0] + points[1] + points[2]) / 3;
    Eigen::Vector3d vecZDirection = vecAtoB.cross(vecAtoC);

    if ((distAB == 0) ||
            (distAC == 0) ||
            (distBC == 0) ||
            (vecZDirection.norm() == 0))
    {
        // Some of the points are either identical or lie on the same line
        errorCode = ERROR_INVALID_POINTS;
        return false;
    }

    // See comments on the calculateReferenceBasis-function for an explanation on
    // how the basis is calculated here (the following few lines are almost identical).

    Eigen::Vector3d unitVecFromCentroidTowardsA = (points[0] - centroid).normalized();
    Eigen::Vector3d unitVecFromCentroidTowardsB = (points[1] - centroid).normalized();
    Eigen::Vector3d unitVecFromCentroidTowardsC = (points[2] - centroid).normalized();

    double angleBetweenVectorsAndBFromCentroid = acos(unitVecFromCentroidTowardsB.dot(unitVecFromCentroidTowardsA)); // * 360. / (2 * M_PI);
    double angleBetweenVectorsAndCFromCentroid = acos(unitVecFromCentroidTowardsC.dot(unitVecFromCentroidTowardsA)); // * 360. / (2 * M_PI);

    double angleError = angleBetweenVectorsAndBFromCentroid - angleBetweenVectorsAndCFromCentroid;

    Eigen::Vector3d unitVecZ = vecZDirection.normalized();

    Eigen::Vector3d unitVecX = Eigen::AngleAxisd(angleError / 3, unitVecZ) * unitVecFromCentroidTowardsA;

    // This results in right-handed system. Handedness only affects the debug
    // output from getTransformMatrix
    Eigen::Vector3d unitVecY = vecZDirection.cross(unitVecX).normalized();
    // Left-handed version:
    // Eigen::Vector3d unitVecY = unitVecX.cross(vecZDirection).normalized();

    Eigen::Matrix3d orientationBasis;

    orientationBasis <<
                        unitVecX(0), unitVecY(0), unitVecZ(0),
                        unitVecX(1), unitVecY(1), unitVecZ(1),
                        unitVecX(2), unitVecY(2), unitVecZ(2);

    Eigen::Matrix3d finalTransform = orientationBasis * refBasisInverse;

    // Origin can be now calculated using centroids and the newly calculated matrix
    Eigen::Vector3d origin = centroid - (finalTransform * refCentroid);

    transform.matrix() <<
                finalTransform(0,0), finalTransform(0,1), finalTransform(0,2), origin(0),
                finalTransform(1,0), finalTransform(1,1), finalTransform(1,2), origin(1),
                finalTransform(2,0), finalTransform(2,1), finalTransform(2,2), origin(2),
                0, 0, 0, 1;

    if (orientationTransform_Debug)
    {
        orientationTransform_Debug->matrix() <<
                unitVecX(0), unitVecY(0), unitVecZ(0), centroid(0),
                unitVecX(1), unitVecY(1), unitVecZ(1), centroid(1),
                unitVecX(2), unitVecY(2), unitVecZ(2), centroid(2),
                0, 0, 0, 1;
    }

    return true;
}

bool LOSolver::getYawPitchRollAngles(const Eigen::Transform<double, 3, Eigen::Affine>& transform,
                                  double& yaw, double& pitch, double& roll)
{
    return getYawPitchRollAngles(transform, yaw, pitch, roll, errorCode);
}

bool LOSolver::getYawPitchRollAngles(Eigen::Transform<double, 3, Eigen::Affine> transform,
                                  double& yaw, double& pitch, double& roll,
                                  ErrorCode& errorCode)
{
    Eigen::Matrix3d linearPart = transform.linear();

    pitch = -asin(linearPart(2, 0));

    // Calculate roll by using a "rotational plane" that is perpendicular
    // to the "forward"-vector and one axis (here planeVecX) is parallel to the "ground plane"
    // I tried to use Eigen's eulerAngles-function to no avail (commented out code below).
    // Maybe there's a way to use it that I didn't find(?)

    Eigen::Vector3d forwardVec(linearPart(0,0), linearPart(1,0), linearPart(2,0));
    const Eigen::Vector3d unitVecDown(0, 0, 1);

    // Vector parallel to the ground plane:
    Eigen::Vector3d planeVecX = -unitVecDown.cross(forwardVec);

    if (planeVecX.norm() == 0)
    {
        // pitch is directly up or down -> yaw and roll are on the same axis (="gimbal lock")
        // -> Calculate yaw based on the object's "down" vector.
        yaw = atan2(linearPart(1,2), linearPart(0,2));
        roll = 0;   // Roll is meaningless in this case
    }
    else
    {
        yaw = atan2(linearPart(1,0), linearPart(0,0));
        planeVecX.normalize();

        // Vector perpendicular to the vector parallel to the ground plane and "forward"-vector:
        Eigen::Vector3d planeVecY = -forwardVec.cross(planeVecX);
        planeVecY.normalize();
        Eigen::Vector3d objectDownVec(linearPart(0,2), linearPart(1,2), linearPart(2,2));

        roll = -atan2(-objectDownVec.dot(planeVecX), objectDownVec.dot(planeVecY));
    }

/* Tried to use Eigen's eulerAngles here but failed. Maybe there's a way?
    Eigen::Vector3d angles;
    angles = coordConv*transform.linear().eulerAngles(?, ?, ?);
    heading = angles(?);
    pitch = angles(?);
    roll = angles(?);
*/

    errorCode = ERROR_NONE;
    return true;
}

