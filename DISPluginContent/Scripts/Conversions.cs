using OpenDis.Dis1998;
using System;
using GlmSharp;
using UnityEngine;

public class Conversions
{
    public static Vector3 UnwindRotation(Vector3 RotationDegrees)
    {
        Vector3 unwoundVector = new Vector3(RotationDegrees.x % 360, RotationDegrees.y % 360, RotationDegrees.z % 360);

        //Check each axis. If axis outside -180 to 180 degree range, take shorter path
        if(RotationDegrees.x <= -180 || RotationDegrees.x >= 180)
        {
            unwoundVector.x += (RotationDegrees.x <= -180) ? 360 : -360;
        }
        if (RotationDegrees.y <= -180 || RotationDegrees.y >= 180)
        {
            unwoundVector.y += (RotationDegrees.y <= -180) ? 360 : -360;
        }
        if (RotationDegrees.z <= -180 || RotationDegrees.z >= 180)
        {
            unwoundVector.z += (RotationDegrees.z <= -180) ? 360 : -360;
        }

        return unwoundVector;
    }

    public static dmat4 CreateSkewMatrix4x4(dvec3 nVector)
    {
        return new dmat4(new dvec4(0, nVector.z, -nVector.y, 0),
            new dvec4(-nVector.z, 0, nVector.x, 0),
            new dvec4(nVector.y, -nVector.x, 0, 0),
            dvec4.Zero
        );
    }

    public static dmat3 CreateSkewMatrix(dvec3 nVector)
    {
        return new dmat3(0, nVector.z, -nVector.y, -nVector.z, 0, nVector.x, nVector.y, -nVector.x, 0);
    }

    public static void CalculateLatLonHeightFromEcefXYZ(Vector3Double ecefLocation, out Vector3Double outLatLonHeightDegreesMeters)
    {
        const double earthSemiMajorRadiusMeters = 6378137;
        const double earthSemiMinorRadiusMeters = 6356752.3142;

        double earthSemiMajorRadiusMetersSquare = Math.Pow(earthSemiMajorRadiusMeters, 2);
        double earthSemiMinorRadiusMetersSquare = Math.Pow(earthSemiMinorRadiusMeters, 2);
        double distFromXToY = Math.Sqrt(Math.Pow(ecefLocation.X, 2) + Math.Pow(ecefLocation.Y, 2));

        double longitude = glm.Degrees(Math.Atan2(ecefLocation.Y, ecefLocation.X));
        double latitude = glm.Degrees(Math.Atan(earthSemiMajorRadiusMetersSquare / earthSemiMinorRadiusMetersSquare * (ecefLocation.Z / distFromXToY)));

        double cosLatitude = Math.Cos(glm.Radians(latitude));
        double sinLatitude = Math.Sin(glm.Radians(latitude));
        double height = (distFromXToY / cosLatitude) - (earthSemiMajorRadiusMetersSquare / Math.Sqrt((earthSemiMajorRadiusMetersSquare * Math.Pow(cosLatitude, 2)) + (earthSemiMinorRadiusMetersSquare * Math.Pow(sinLatitude, 2))));

        outLatLonHeightDegreesMeters = new Vector3Double
        {
            X = latitude,
            Y = longitude,
            Z = height
        };
    }

    public static void CalculateEcefXYZFromLatLonHeight(Vector3Double latLonHeightDegreesMeters, out Vector3Double ecefLocation)
    {
        const double earthSemiMajorRadiusMeters = 6378137;
        const double earthSemiMinorRadiusMeters = 6356752.3142;

        double cosLatitude = Math.Cos(glm.Radians(latLonHeightDegreesMeters.X));
        double sinLatitude = Math.Sin(glm.Radians(latLonHeightDegreesMeters.X));
        double cosLongitude = Math.Cos(glm.Radians(latLonHeightDegreesMeters.Y));
        double sinLongitude = Math.Sin(glm.Radians(latLonHeightDegreesMeters.Y));

        double XYBaseConversion = (earthSemiMajorRadiusMeters / (Math.Sqrt(Math.Pow(cosLatitude, 2) + ((Math.Pow(earthSemiMinorRadiusMeters, 2) / Math.Pow(earthSemiMajorRadiusMeters, 2)) * Math.Pow(sinLatitude, 2))))) + latLonHeightDegreesMeters.Z;
        double ZBaseConversion = (earthSemiMinorRadiusMeters / (((Math.Sqrt(Math.Pow(cosLatitude, 2) * (Math.Pow(earthSemiMajorRadiusMeters, 2) / Math.Pow(earthSemiMinorRadiusMeters, 2)) + Math.Pow(sinLatitude, 2)))))) + latLonHeightDegreesMeters.Z;

        ecefLocation = new Vector3Double
        {
            X = XYBaseConversion * cosLatitude * cosLongitude,
            Y = XYBaseConversion * cosLatitude * sinLongitude,
            Z = ZBaseConversion * sinLatitude
        };
    }

    public static void CreateRotationMatrix4x4(dvec3 AxisVector, double thetaRadians, out dmat4 outRotationMatrix)
    {
        double cosTheta = Math.Cos(thetaRadians);
        double sinTheta = Math.Sin(thetaRadians);

        outRotationMatrix = dmat4.Zero;
        dmat4 N = new dmat4(new dvec4(AxisVector.x, 0, 0, 0), new dvec4(AxisVector.y, 0, 0, 0), new dvec4(AxisVector.z, 0, 0, 0), dvec4.Zero);
        dmat4 NTransposeN = N.Transposed * N;

        dmat4 NCrossX = CreateSkewMatrix4x4(AxisVector);

        dmat4 ScaledTranspose = new dmat4(NTransposeN);
        ScaledTranspose *= (1 - cosTheta);
        dmat4 Identity = dmat4.Identity;
        Identity *= cosTheta;
        // Zero out the 4th row 4th column entry to represent the 3x3 matrix as a 4x4
        Identity.m33 = 0;
        dmat4 ScaledNCrossX = new dmat4(NCrossX);
        ScaledNCrossX *= sinTheta;

        outRotationMatrix += ScaledTranspose + Identity + ScaledNCrossX;
    }

    public static void CreateRotationMatrix(dvec3 AxisVector, double ThetaRadians, out dmat3 OutRotationMatrix)
    {
        double CosTheta = glm.Cos(ThetaRadians);
        double SinTheta = glm.Sin(ThetaRadians);

        dvec3 N = AxisVector;
        dmat3 NMat = new dmat3(N, dvec3.Zero, dvec3.Zero);

        dmat3 NTransposeN = NMat * NMat.Transposed;
        dmat3 NCrossN = CreateSkewMatrix(N);

        OutRotationMatrix = ((1 - CosTheta) * NTransposeN) + (CosTheta * dmat3.Identity) + (SinTheta * NCrossN);
    }

    public static void ApplyHeadingPitchRollToNorthEastDownVector(double headingDegrees, double pitchDegrees, double rollDegrees, dvec3 northVector, dvec3 eastVector, dvec3 downVector, out dvec3 outX, out dvec3 outY, out dvec3 outZ)
    {
        ApplyHeadingPitchToNorthEastDownVector(headingDegrees, pitchDegrees, northVector, eastVector, downVector, out dvec3 outNorthVector, out dvec3 outEastVector, out dvec3 outDownVector);
        ApplyRollToNorthEastDownVector(rollDegrees, outNorthVector, outEastVector, outDownVector, out outX, out outY, out outZ);
    }

    public static void ApplyHeadingPitchToNorthEastDownVector(double headingDegrees, double pitchDegrees, dvec3 northVector, dvec3 eastVector, dvec3 downVector, out dvec3 outX, out dvec3 outY, out dvec3 outZ)
    {
        RotateVectorAroundAxisByDegrees(northVector, headingDegrees, downVector, out outX);
        RotateVectorAroundAxisByDegrees(eastVector, headingDegrees, downVector, out outY);

        RotateVectorAroundAxisByDegrees(outX, pitchDegrees, outY, out outX);
        RotateVectorAroundAxisByDegrees(downVector, pitchDegrees, outY, out outZ);
    }

    public static void ApplyRollToNorthEastDownVector(double rollDegrees, dvec3 northVector, dvec3 eastVector, dvec3 downVector, out dvec3 outX, out dvec3 outY, out dvec3 outZ)
    {
        outX = northVector;
        RotateVectorAroundAxisByDegrees(eastVector, rollDegrees, northVector, out outY);
        RotateVectorAroundAxisByDegrees(downVector, rollDegrees, northVector, out outZ);
    }

    public static void RotateVectorAroundAxisByDegrees(dvec3 vectorToRotate, double thetaDegrees, dvec3 axisVector, out dvec3 outRotatedVector)
    {
        RotateVectorAroundAxisByRadians(vectorToRotate, glm.Radians(thetaDegrees), axisVector, out outRotatedVector);
    }

    public static void RotateVectorAroundAxisByRadians(dvec3 vectorToRotate, double thetaRadians, dvec3 axisVector, out dvec3 outRotatedVector)
    {
        dmat4 VectorMatrix = new dmat4(new dvec4(vectorToRotate.x, 0, 0, 0), new dvec4(vectorToRotate.y, 0, 0, 0), new dvec4(vectorToRotate.z, 0, 0, 0), dvec4.Zero);
        CreateRotationMatrix4x4(axisVector, thetaRadians, out dmat4 rotationMatrix);
        dmat4 ResMatrix = VectorMatrix * rotationMatrix.Transposed;
        outRotatedVector = new dvec3(ResMatrix.Row0);
    }

    public static void CalculateNorthEastDownVectorsFromLatLon(double latitudeDegrees, double longitudeDegrees, out dvec3 northVector, out dvec3 eastVector, out dvec3 downVector)
    {
        northVector = new dvec3(0, 0, 1);
        eastVector = new dvec3(0, 1, 0);
        downVector = new dvec3(-1, 0, 0);

        RotateVectorAroundAxisByDegrees(eastVector, longitudeDegrees, northVector, out eastVector);
        RotateVectorAroundAxisByDegrees(downVector, longitudeDegrees, northVector, out downVector);

        RotateVectorAroundAxisByDegrees(northVector, latitudeDegrees, -eastVector, out northVector);
        RotateVectorAroundAxisByDegrees(downVector, latitudeDegrees, -eastVector, out downVector);
    }

    public static void CalculateLatLongFromNorthEastDownVectors(dvec3 northVector, dvec3 eastVector, dvec3 downVector, out double latitudeDegrees, out double longitudeDegrees)
    {
        longitudeDegrees = glm.Degrees(Math.Acos(glm.Dot(new dvec3(0, 1, 0), eastVector) / eastVector.Length));
        latitudeDegrees = glm.Degrees(Math.Acos(glm.Dot(new dvec3(0, 0, 1), northVector) / northVector.Length));
    }

    public static void CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(double headingDegrees, double pitchDegrees, double rollDegrees, double latitudeDegrees, double longitudeDegrees, out double psiDegrees, out double thetaDegrees, out double phiDegrees)
    {
        CalculateNorthEastDownVectorsFromLatLon(latitudeDegrees, longitudeDegrees, out dvec3 northVector, out dvec3 eastVector, out dvec3 downVector);

        ApplyHeadingPitchRollToNorthEastDownVector(headingDegrees, pitchDegrees, rollDegrees, northVector, eastVector, downVector, out dvec3 X, out dvec3 Y, out _);

        dvec3 X0 = new dvec3(1, 0, 0);
        dvec3 Y0 = new dvec3(0, 1, 0);
        dvec3 Z0 = new dvec3(0, 0, 1);

        psiDegrees = glm.Degrees(Math.Atan2(glm.Dot(X, Y0), glm.Dot(X, X0)));
        thetaDegrees = glm.Degrees(Math.Atan2(-glm.Dot(X, Z0), Math.Sqrt(Math.Pow(glm.Dot(X, X0), 2) + Math.Pow(glm.Dot(X, Y0), 2))));

        northVector = X0;
        eastVector = Y0;
        downVector = Z0;

        ApplyHeadingPitchToNorthEastDownVector(psiDegrees, thetaDegrees, northVector, eastVector, downVector, out _, out dvec3 Y2, out dvec3 Z2);

        phiDegrees = glm.Degrees(Math.Atan2(glm.Dot(Y, Z2), glm.Dot(Y, Y2)));
    }

    public static void CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(double headingRadians, double pitchRadians, double rollRadians, double latitudeDegrees, double longitudeDegrees, out double psiRadians, out double thetaRadians, out double phiRadians)
    {
        CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(glm.Degrees(headingRadians), glm.Degrees(pitchRadians), glm.Degrees(rollRadians), latitudeDegrees, longitudeDegrees, out psiRadians, out thetaRadians, out phiRadians);
    }

    public static void CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(double headingDegrees, double pitchDegrees, double rollDegrees, double latitudeDegrees, double longitudeDegrees, out double psiRadians, out double thetaRadians, out double phiRadians)
    {
        CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(headingDegrees, pitchDegrees, rollDegrees, latitudeDegrees, longitudeDegrees, out double psiDegrees, out double thetaDegrees, out double phiDegrees);
        psiRadians = glm.Radians(psiDegrees);
        thetaRadians = glm.Radians(thetaDegrees);
        phiRadians = glm.Radians(phiDegrees);
    }

    public static void CalculatePsiThetaPhiDegreesFromHeadingPitchRollRadiansAtLatLon(double headingRadians, double pitchRadians, double rollRadians, double latitudeDegrees, double longitudeDegrees, out double psiDegrees, out double thetaDegrees, out double phiDegrees)
    {
        CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(glm.Degrees(headingRadians), glm.Degrees(pitchRadians), glm.Degrees(rollRadians), latitudeDegrees, longitudeDegrees, out psiDegrees, out thetaDegrees, out phiDegrees);
    }

    public static void CalculateHeadingPitchRollDegreesFromPsiThetaPhiDegreesAtLatLon(double psiDegrees, double thetaDegrees, double phiDegrees, double latitudeDegrees, double longitudeDegrees, out double headingDegrees, out double pitchDegrees, out double rollDegrees)
    {
        CalculateNorthEastDownVectorsFromLatLon(latitudeDegrees, longitudeDegrees, out dvec3 northVector, out dvec3 eastVector, out dvec3 downVector);

        dvec3 x0 = new dvec3(1, 0, 0);
        dvec3 y0 = new dvec3(0, 1, 0);
        dvec3 z0 = new dvec3(0, 0, 1);

        ApplyHeadingPitchRollToNorthEastDownVector(psiDegrees, thetaDegrees, phiDegrees, x0, y0, z0, out dvec3 x3, out dvec3 y3, out _);

        headingDegrees = glm.Degrees(Math.Atan2(glm.Dot(x3, eastVector), glm.Dot(x3, northVector)));
        pitchDegrees = glm.Degrees(Math.Atan2(-glm.Dot(x3, downVector), Math.Sqrt(Math.Pow(glm.Dot(x3, eastVector), 2) + Math.Pow(glm.Dot(x3, northVector), 2))));

        ApplyHeadingPitchToNorthEastDownVector(headingDegrees, pitchDegrees, northVector, eastVector, downVector, out _, out dvec3 y2, out dvec3 z2);
        rollDegrees = glm.Degrees(Math.Atan2(glm.Dot(y3, z2), glm.Dot(y3, y2)));
    }

    public static void CalculateHeadingPitchRollRadiansFromPsiThetaPhiDegreesAtLatLon(double psiDegrees, double thetaDegrees, double phiDegrees, double latitudeDegrees, double longitudeDegrees, out double headingRadians, out double pitchRadians, out double rollRadians)
    {
        CalculateHeadingPitchRollDegreesFromPsiThetaPhiDegreesAtLatLon(psiDegrees, thetaDegrees, phiDegrees, latitudeDegrees, longitudeDegrees, out double headingDegrees, out double pitchDegrees, out double rollDegrees);
        headingRadians = glm.Radians(headingDegrees);
        pitchRadians = glm.Radians(pitchDegrees);
        rollRadians = glm.Radians(rollDegrees);
    }

    public static void CalculateHeadingPitchRollDegreesFromPsiThetaPhiRadiansAtLatLon(double psiRadians, double thetaRadians, double phiRadians, double latitudeDegrees, double longitudeDegrees, out double headingDegrees, out double pitchDegrees, out double rollDegrees)
    {
        CalculateHeadingPitchRollDegreesFromPsiThetaPhiDegreesAtLatLon(glm.Degrees(psiRadians), glm.Degrees(thetaRadians), glm.Degrees(phiRadians), latitudeDegrees, longitudeDegrees, out headingDegrees, out pitchDegrees, out rollDegrees);
    }

    public static void CalculateHeadingPitchRollRadiansFromPsiThetaPhiRadiansAtLatLon(double psiRadians, double thetaRadians, double phiRadians, double latitudeDegrees, double longitudeDegrees, out double headingRadians, out double pitchRadians, out double rollRadians)
    {
        CalculateHeadingPitchRollRadiansFromPsiThetaPhiDegreesAtLatLon(glm.Degrees(psiRadians), glm.Degrees(thetaRadians), glm.Degrees(phiRadians), latitudeDegrees, longitudeDegrees, out headingRadians, out pitchRadians, out rollRadians);
    }

    public static void GetEastNorthUpVectorsFromNorthEastDownVectors(dvec3 northVector, dvec3 eastVector, dvec3 downVector, out dvec3 outEastVector, out dvec3 outNorthVector, out dvec3 outUpVector)
    {
        outEastVector = eastVector;
        outNorthVector = northVector;
        outUpVector = -downVector;
    }

    public static void GetNorthEastDownVectorsFromEastNorthUpVectors(dvec3 eastVector, dvec3 northVector, dvec3 upVector, out dvec3 outNorthVector, out dvec3 outEastVector, out dvec3 outDownVector)
    {
        outNorthVector = northVector;
        outEastVector = eastVector;
        outDownVector = -upVector;
    }
}