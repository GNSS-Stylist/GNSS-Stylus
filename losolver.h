/*
    losolver.h (part of GNSS-Stylus)
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
#ifndef LOSOLVER_H
#define LOSOLVER_H

#include "Eigen/Geometry"

class LOSolver
{
public:
    // Not needed anymore? EIGEN_MAKE_ALIGNED_OPERATOR_NEW // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html

    enum ErrorCode
    {
        ERROR_NONE = 0,

        ERROR_INVALID_REFERENCE_POINTS = 100,

        ERROR_INVALID_POINTS = 200,

        ERROR_NOT_KNOWN = 0xFF
    };

    LOSolver();
    void init(void);
    ErrorCode getLastError(void) { return errorCode; }

    bool getReferencePointsValidity(void) { return refPointsValid; }
    bool setReferencePoints(const Eigen::Vector3d refPoints[3]);
    bool setPoints(const Eigen::Vector3d points[3]);
    bool getTransformMatrix(Eigen::Transform<double, 3, Eigen::Affine>& transform,
                            Eigen::Transform<double, 3, Eigen::Affine>* orientationTransform_Debug = nullptr);

    // These expect axes convention to be NED (X=North, Y=East, Z=Down)
    bool getYawPitchRollAngles(const Eigen::Transform<double, 3, Eigen::Affine>& transform, double& yaw, double& pitch, double& roll);
    static bool getYawPitchRollAngles(Eigen::Transform<double, 3, Eigen::Affine> transform, double& yaw, double& pitch, double& roll, ErrorCode& errorCode);

private:
    ErrorCode errorCode = ERROR_INVALID_REFERENCE_POINTS;

    Eigen::Matrix3d refBasisInverse;

    bool refPointsValid = false;
    Eigen::Vector3d refPoints[3];
    Eigen::Vector3d refCentroid;

    Eigen::Vector3d points[3];

    // For speed-up when checking the validity of points:
    double refDistAB;
    double refDistAC;
    double refDistBC;

    bool calculateReferenceBasis(void);
};

#endif // LOSOLVER_H
