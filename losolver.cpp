/*
    losolver.cpp (part of GNSS-Stylus)
    Copyright (C) 2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

    Eigen::Vector3d vecAtoBCMidpoint = (refPoints[1] + refPoints[2]) / 2 - refPoints[0];

    refDistAB = vecAtoB.norm();
    refDistAC = vecAtoC.norm();
    refDistBC = vecBtoC.norm();

    refCentroid = (refPoints[0] + refPoints[1] + refPoints[2]) / 3;
    Eigen::Vector3d refVecYDirection = vecAtoC.cross(vecAtoB);

    if ((refDistAB == 0) ||
            (refDistAC == 0) ||
            (refDistBC == 0) ||
            (refVecYDirection.norm() == 0))
    {
        // Some of the points are either identical or lie on the same line
        refPointsValid = false;
        errorCode = ERROR_INVALID_REFERENCE_POINTS;
        return false;
    }

    // Calculate reference basis vectors so that:
    // X points from A to the midpoint of vector BC,
    // Y is in right angle with the plane defined by points A, B and C,
    // Z is in right angle with both vectors AB and the new unit vector Y
    // (and therefore Z lies on the plane defined by points A, B and C)

    Eigen::Vector3d refUnitVecX = vecAtoBCMidpoint.normalized();
    Eigen::Vector3d refUnitVecY = refVecYDirection.normalized();

    // This results in right-handed system. Handedness only affects the debug
    // output from getTransformMatrix (if you make the same change to getTransformMatrix-function)
    Eigen::Vector3d refUnitVecZ = refUnitVecX.cross(refUnitVecY).normalized();
    // Left-handed version:
    // Eigen::Vector3d refUnitVecZ = refUnitVecY.cross(refUnitVecX).normalized();

    // This is constructed as transposed, but as it's orthogonal, it equals the inverse:
    refBasisInverse <<
                refUnitVecX(0), refUnitVecX(1), refUnitVecX(2),
                refUnitVecY(0), refUnitVecY(1), refUnitVecY(2),
                refUnitVecZ(0), refUnitVecZ(1), refUnitVecZ(2);

    refPointsValid = true;
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

    Eigen::Vector3d vecAtoBCMidpoint = (points[1] + points[2]) / 2 - points[0];

    double distAB = vecAtoB.norm();
    double distAC = vecAtoC.norm();
    double distBC = vecBtoC.norm();

    // TODO: Add comparison of distances with the reference basis points

    Eigen::Vector3d centroid = (points[0] + points[1] + points[2]) / 3;
    Eigen::Vector3d vecYDirection = vecAtoC.cross(vecAtoB);

    if ((distAB == 0) ||
            (distAC == 0) ||
            (distBC == 0) ||
            (vecYDirection.norm() == 0))
    {
        // Some of the points are either identical or lie on the same line
        errorCode = ERROR_INVALID_POINTS;
        return false;
    }

    // Calculate reference basis vectors (identically with the reference basis) so that:
    // X points from A to the midpoint of vector BC,
    // Y is in right angle with the plane defined by points A, B and C,
    // Z is in right angle with both vectors AB and the new unit vector Y
    // (and therefore Z lies on the plane defined by points A, B and C)

    Eigen::Vector3d orientationUnitVecX = vecAtoBCMidpoint.normalized();
    Eigen::Vector3d orientationUnitVecY = vecYDirection.normalized();

    // This results in right-handed system. Handedness only affects the debug
    // output (orientationTransform_Debug) (if you make the same change to calculateReferenceBasis-function)
    Eigen::Vector3d orientationUnitVecZ = orientationUnitVecX.cross(orientationUnitVecY).normalized();
    // Left-handed version:
    // Eigen::Vector3d orientationUnitVecZ = orientationUnitVecY.cross(orientationUnitVecX).normalized();

    Eigen::Matrix3d orientationBasis;

    orientationBasis <<
                        orientationUnitVecX(0), orientationUnitVecY(0), orientationUnitVecZ(0),
                        orientationUnitVecX(1), orientationUnitVecY(1), orientationUnitVecZ(1),
                        orientationUnitVecX(2), orientationUnitVecY(2), orientationUnitVecZ(2);

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
                orientationUnitVecX(0), orientationUnitVecY(0), orientationUnitVecZ(0), centroid(0),
                orientationUnitVecX(1), orientationUnitVecY(1), orientationUnitVecZ(1), centroid(1),
                orientationUnitVecX(2), orientationUnitVecY(2), orientationUnitVecZ(2), centroid(2),
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

