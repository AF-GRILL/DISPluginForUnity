using OpenDis.Dis1998;
using System;
using GlmSharp;
using UnityEngine;

namespace GRILLDIS
{
    public class Conversions
    {
        /// <summary>
        /// Takes in a rotation in degrees and finds the shortest route along each axis. (ex: [348, -181, 89] becomes [-12, 179, 89])
        /// </summary>
        /// <param name="RotationDegrees">Vector3 in degrees to unwind.</param>
        /// <returns>Vector3 containing shortest route rotations along each axis.</returns>
        public static Vector3 UnwindRotation(Vector3 RotationDegrees)
        {
            Vector3 unwoundVector = new Vector3(RotationDegrees.x % 360, RotationDegrees.y % 360, RotationDegrees.z % 360);

            //Check each axis. If axis outside -180 to 180 degree range, take shorter path
            if (unwoundVector.x <= -180 || unwoundVector.x >= 180)
            {
                unwoundVector.x += (RotationDegrees.x <= -180) ? 360 : -360;
            }
            if (unwoundVector.y <= -180 || unwoundVector.y >= 180)
            {
                unwoundVector.y += (RotationDegrees.y <= -180) ? 360 : -360;
            }
            if (unwoundVector.z <= -180 || unwoundVector.z >= 180)
            {
                unwoundVector.z += (RotationDegrees.z <= -180) ? 360 : -360;
            }

            return unwoundVector;
        }

        /// <summary>
        /// Creates a 4x4 n^x matrix used for creating a rotation matrix
        /// </summary>
        /// <param name="nVector">A 3x1 vector representing the axis of rotation</param>
        /// <returns></returns>
        public static dmat4 CreateSkewMatrix4x4(dvec3 nVector)
        {
            return new dmat4(new dvec4(0, nVector.z, -nVector.y, 0),
                             new dvec4(-nVector.z, 0, nVector.x, 0),
                             new dvec4(nVector.y, -nVector.x, 0, 0),
                             dvec4.Zero);

        }

        /// <summary>
        /// Creates a 3x3 n^x matrix used for creating a rotation matrix
        /// </summary>
        /// <param name="nVector">A 3x1 vector representing the axis of rotation</param>
        /// <returns></returns>
        public static dmat3 CreateSkewMatrix(dvec3 nVector)
        {
            return new dmat3(0, nVector.z, -nVector.y,
                             -nVector.z, 0, nVector.x,
                              nVector.y, -nVector.x, 0);
        }

        /// <summary>
        /// Converts DIS X, Y, Z coordinates (ECEF) to Latitude, Longitude, and Height (LLH) all in double (64-bit) precision
        /// </summary>
        /// <param name="ecefLocation">The ECEF location</param>
        /// <return>The converted latitude in degrees, longitude in degrees, and height in meters</return>
        public static FLatLonAlt CalculateLatLonHeightFromEcefXYZ(Vector3Double ecefLocation)
        {
            FLatLonAlt latLonAltLocation = new FLatLonAlt();

            double earthEquitorialRadiusMeters = 6378137;
            double earthPolarRadiusMeters = 6356752.3142;

            double earthEquitorialRadiusMetersSquared = Math.Pow(earthEquitorialRadiusMeters, 2);
            double earthPolarRadiusMetersSquared = Math.Pow(earthPolarRadiusMeters, 2);

            double eSquared = (earthEquitorialRadiusMetersSquared - earthPolarRadiusMetersSquared) / earthEquitorialRadiusMetersSquared;
            double ePrimeSquared = (earthEquitorialRadiusMetersSquared - earthPolarRadiusMetersSquared) / earthPolarRadiusMetersSquared;

            double p = Math.Sqrt(Math.Pow(ecefLocation.X, 2) + Math.Pow(ecefLocation.Y, 2));
            double F = 54 * earthPolarRadiusMetersSquared * Math.Pow(ecefLocation.Z, 2);
            double G = Math.Pow(p, 2) + (1 - eSquared) * Math.Pow(ecefLocation.Z, 2) - eSquared * (earthEquitorialRadiusMetersSquared - earthPolarRadiusMetersSquared);
            double c = (Math.Pow(eSquared, 2) * F * Math.Pow(p, 2)) / Math.Pow(G, 3);

            double s = Math.Pow(1 + c + Math.Sqrt(Math.Pow(c, 2) + 2 * c), 1f / 3f);
            double k = s + 1 + 1 / s;
            double P = F / (3 * Math.Pow(k, 2) * Math.Pow(G, 2));
            double Q = Math.Sqrt(1 + 2 * Math.Pow(eSquared, 2) * P);

            double rNot = (-P * eSquared * p) / (1 + Q) + Math.Sqrt(1f / 2f * earthEquitorialRadiusMetersSquared * (1 + 1 / Q) - (P * (1 - eSquared) * Math.Pow(ecefLocation.Z, 2)) / (Q * (1 + Q)) - 1f / 2f * P * Math.Pow(p, 2));
            double U = Math.Sqrt(Math.Pow(p - eSquared * rNot, 2) + Math.Pow(ecefLocation.Z, 2));
            double V = Math.Sqrt(Math.Pow(p - eSquared * rNot, 2) + (1 - eSquared) * Math.Pow(ecefLocation.Z, 2));
            double zNot = (earthPolarRadiusMetersSquared * ecefLocation.Z) / (earthEquitorialRadiusMeters * V);

            latLonAltLocation.Altitude = U * (1 - earthPolarRadiusMetersSquared / (earthEquitorialRadiusMeters * V));
            latLonAltLocation.Latitude = glm.Degrees(Math.Atan((ecefLocation.Z + ePrimeSquared * zNot) / p));
            latLonAltLocation.Longitude = glm.Degrees(Math.Atan2(ecefLocation.Y, ecefLocation.X));

            return latLonAltLocation;
        }

        /// <summary>
        /// Converts Latitude, Longitude, and Height (LLH) to DIS X, Y, Z coordinates (ECEF) all in double (64-bit) precision
        /// </summary>
        /// <param name="latLonHeightDegreesMeters">The latitude in degrees, longitude in degrees, and height in meters</param>
        /// <return>The converted ECEF location</return>
        public static Vector3Double CalculateEcefXYZFromLatLonHeight(FLatLonAlt latLonHeightDegreesMeters)
        {
            Vector3Double ecefLocation = new Vector3Double();

            double earthEquitorialRadiusMeters = 6378137;
            double earthPolarRadiusMeters = 6356752.3142;

            double earthEquitorialRadiusMetersSquared = Math.Pow(earthEquitorialRadiusMeters, 2);
            double earthPolarRadiusMetersSquared = Math.Pow(earthPolarRadiusMeters, 2);

            double eSquared = 1 - earthPolarRadiusMetersSquared / earthEquitorialRadiusMetersSquared;
            double f = 1 - earthPolarRadiusMeters / earthEquitorialRadiusMeters;

            double nLat = earthEquitorialRadiusMeters / Math.Sqrt(1 - eSquared * Math.Pow(Math.Sin(glm.Radians(latLonHeightDegreesMeters.Latitude)), 2));

            double latRadians = glm.Radians(latLonHeightDegreesMeters.Latitude);
            double lonRadians = glm.Radians(latLonHeightDegreesMeters.Longitude);

            ecefLocation.X = (nLat + latLonHeightDegreesMeters.Altitude) * Math.Cos(latRadians) * Math.Cos(lonRadians);
            ecefLocation.Y = (nLat + latLonHeightDegreesMeters.Altitude) * Math.Cos(latRadians) * Math.Sin(lonRadians);
            ecefLocation.Z = (Math.Pow(1 - f, 2) * nLat + latLonHeightDegreesMeters.Altitude) * Math.Sin(latRadians);

            return ecefLocation;
        }

        /// <summary>
        /// Creates a 4x4 rotation matrix around the given axis of rotation rotating by Theta degrees
        /// </summary>
        /// <param name="AxisVector">A 3x1 vector representing the axis of rotation</param>
        /// <param name="thetaRadians">The amount to rotate given in radians</param>
        /// <return>The 4x4 rotation matrix</return>
        public static dmat4 CreateRotationMatrix(Vector3 AxisVector, double thetaRadians)
        {
            double cosTheta = Math.Cos(thetaRadians);
            double sinTheta = Math.Sin(thetaRadians);

            dmat4 N = new dmat4(new dvec4(AxisVector.x, 0, 0, 0), new dvec4(AxisVector.y, 0, 0, 0), new dvec4(AxisVector.z, 0, 0, 0), dvec4.Zero);
            dmat4 NTransposeN = N.Transposed * N;

            dmat4 NCrossX = CreateSkewMatrix4x4(new dvec3(AxisVector.x, AxisVector.y, AxisVector.z));

            dmat4 ScaledTranspose = new dmat4(NTransposeN);
            ScaledTranspose *= (1 - cosTheta);
            dmat4 Identity = dmat4.Identity;
            Identity *= cosTheta;
            // Zero out the 4th row 4th column entry to represent the 3x3 matrix as a 4x4
            Identity.m33 = 0;
            dmat4 ScaledNCrossX = new dmat4(NCrossX);
            ScaledNCrossX *= sinTheta;

            return ScaledTranspose + Identity + ScaledNCrossX;
        }

        /// <summary>
        /// Creates a 3x3 rotation matrix around the given axis of rotation rotating by Theta degrees
        /// </summary>
        /// <param name="AxisVector">A 3x1 vector representing the axis of rotation</param>
        /// <param name="ThetaRadians">The amount to rotate given in radians</param>
        /// <return>The 3x3 rotation matrix</return>
        public static dmat3 CreateRotationMatrix(dvec3 AxisVector, double ThetaRadians)
        {
            double CosTheta = glm.Cos(ThetaRadians);
            double SinTheta = glm.Sin(ThetaRadians);

            dmat3 NTransposeN = glm.OuterProduct(AxisVector, AxisVector);
            dmat3 NCrossN = CreateSkewMatrix(AxisVector);

            return ((1 - CosTheta) * NTransposeN) + (CosTheta * dmat3.Identity) + (SinTheta * NCrossN);
        }

        /// <summary>
        /// Applies the given heading, pitch, and roll in degrees to the local East North Down vectors
        /// </summary>
        /// <param name="HeadingPitchRollDegrees">The degrees from North of the facing direction (heading), the radians rotated about the local X axis (pitch), and the radians rotated about the local Z axis (roll)</param>
        /// <param name="NorthEastDownVectors">The vectors pointing to the North, to the East, and toward the center of the Earth</param>
        /// <param name="outX">The x axis (forward) vector with the heading and pitch applied</param>
        /// <param name="outY">The y axis (right) vector with the heading and pitch applied</param>
        /// <param name="outZ">The z axis (down) vector with the heading and pitch applied</param>
        public static void ApplyHeadingPitchRollToNorthEastDownVector(FHeadingPitchRoll HeadingPitchRollDegrees, FNorthEastDown NorthEastDownVectors, out Vector3 outX, out Vector3 outY, out Vector3 outZ)
        {
            ApplyHeadingPitchToNorthEastDownVector(HeadingPitchRollDegrees.Heading, HeadingPitchRollDegrees.Pitch, NorthEastDownVectors, out Vector3 outNorthVector, out Vector3 outEastVector, out Vector3 outDownVector);
            ApplyRollToNorthEastDownVector(HeadingPitchRollDegrees.Roll, new FNorthEastDown(outNorthVector, outEastVector, outDownVector), out outX, out outY, out outZ);
        }

        /// <summary>
        /// Rotates the given East, North, and Up vectors by the given Heading and Pitch
        /// </summary>
        /// <param name="headingDegrees">The degrees from North of the facing direction (spin left and right)</param>
        /// <param name="pitchDegrees">The degrees rotated about the local X axis (front tip up and down)</param>
        /// <param name="NorthEastDownVectors">The vectors pointing to the North, to the East, and toward the center of the Earth</param>
        /// <param name="outX">The x axis (forward) vector with the heading and pitch applied</param>
        /// <param name="outY">The y axis (right) vector with the heading and pitch applied</param>
        /// <param name="outZ">The z axis (down) vector with the heading and pitch applied</param>
        public static void ApplyHeadingPitchToNorthEastDownVector(double headingDegrees, double pitchDegrees, FNorthEastDown NorthEastDownVectors, out Vector3 outX, out Vector3 outY, out Vector3 outZ)
        {
            outX = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.NorthVector, headingDegrees, NorthEastDownVectors.DownVector);
            outY = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.EastVector, headingDegrees, NorthEastDownVectors.DownVector);

            outX = RotateVectorAroundAxisByDegrees(outX, pitchDegrees, outY);
            outZ = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.DownVector, pitchDegrees, outY);
        }

        /// <summary>
        /// Rotates the given East, North, and Up vectors by the given Heading and Pitch
        /// </summary>
        /// <param name="rollDegrees">The degrees rotated about the local Z axis (tilt left and right)</param>
        /// <param name="NorthEastDownVectors">The vectors pointing to the North, to the East, and toward the center of the Earth</param>
        /// <param name="outX">The x axis (forward) vector with the heading and pitch applied</param>
        /// <param name="outY">The y axis (right) vector with the heading and pitch applied</param>
        /// <param name="outZ">The z axis (down) vector with the heading and pitch applied</param>
        public static void ApplyRollToNorthEastDownVector(double rollDegrees, FNorthEastDown NorthEastDownVectors, out Vector3 outX, out Vector3 outY, out Vector3 outZ)
        {
            outX = NorthEastDownVectors.NorthVector;
            outY = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.EastVector, rollDegrees, NorthEastDownVectors.NorthVector);
            outZ = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.DownVector, rollDegrees, NorthEastDownVectors.NorthVector);
        }

        /// <summary>
        /// Rotates the vector VectorToRotate around given axis AxisVector by Theta degrees
        /// </summary>
        /// <param name="vectorToRotate">The target vector to rotate</param>
        /// <param name="thetaDegrees">The desired amount to rotation in degrees</param>
        /// <param name="axisVector">The vector indicating the axis of rotation</param>
        /// <return>The resultant rotated vector</return>
        public static Vector3 RotateVectorAroundAxisByDegrees(Vector3 vectorToRotate, double thetaDegrees, Vector3 axisVector)
        {
            return RotateVectorAroundAxisByRadians(vectorToRotate, glm.Radians(thetaDegrees), axisVector);
        }

        public static dvec3 RotateVectorAroundAxisByDegrees(dvec3 vectorToRotate, double thetaDegrees, dvec3 axisVector)
        {
            return RotateVectorAroundAxisByRadians(vectorToRotate, glm.Radians(thetaDegrees), axisVector);
        }

        /// <summary>
        /// Rotates the vector VectorToRotate around given axis AxisVector by Theta radians
        /// </summary>
        /// <param name="vectorToRotate">The target vector to rotate</param>
        /// <param name="thetaRadians">The desired amount to rotation in radians</param>
        /// <param name="axisVector">The vector indicating the axis of rotation</param>
        /// <return>The resultant rotated vector</return>
        public static Vector3 RotateVectorAroundAxisByRadians(Vector3 vectorToRotate, double thetaRadians, Vector3 axisVector)
        {
            dmat4 VectorMatrix = new dmat4(new dvec4(vectorToRotate.x, 0, 0, 0), new dvec4(vectorToRotate.y, 0, 0, 0), new dvec4(vectorToRotate.z, 0, 0, 0), dvec4.Zero);
            dmat4 rotationMatrix = CreateRotationMatrix(axisVector, thetaRadians);
            dmat4 ResMatrix = VectorMatrix * rotationMatrix.Transposed;
            return new Vector3((float)ResMatrix.Row0.x, (float)ResMatrix.Row0.y, (float)ResMatrix.Row0.z);
        }

        public static dvec3 RotateVectorAroundAxisByRadians(dvec3 vectorToRotate, double thetaRadians, dvec3 axisVector)
        {
            dmat3 VectorMatrix = new dmat3(new dvec3(vectorToRotate.x, 0, 0), new dvec3(vectorToRotate.y, 0, 0), new dvec3(vectorToRotate.z, 0, 0));
            dmat3 rotationMatrix = CreateRotationMatrix(axisVector, thetaRadians);
            dmat3 ResMatrix = rotationMatrix * VectorMatrix;
            return new dvec3(ResMatrix.Column0);
        }

        /// <summary>
        /// Calculates the East, North, and Up vectors at given latitude and longitude.
        /// </summary>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The local vectors pointing North, pointing East, and toward the center of the Earth at the given latitude and longitude</return>
        public static FNorthEastDown CalculateNorthEastDownVectorsFromLatLon(double latitudeDegrees, double longitudeDegrees)
        {
            FNorthEastDown NorthEastDownVectors;
            NorthEastDownVectors.NorthVector = new Vector3(0, 0, 1);
            NorthEastDownVectors.EastVector = new Vector3(0, 1, 0);
            NorthEastDownVectors.DownVector = new Vector3(-1, 0, 0);

            NorthEastDownVectors.EastVector = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.EastVector, longitudeDegrees, NorthEastDownVectors.NorthVector);
            NorthEastDownVectors.DownVector = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.DownVector, longitudeDegrees, NorthEastDownVectors.NorthVector);

            NorthEastDownVectors.NorthVector = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.NorthVector, latitudeDegrees, -NorthEastDownVectors.EastVector);
            NorthEastDownVectors.DownVector = RotateVectorAroundAxisByDegrees(NorthEastDownVectors.DownVector, latitudeDegrees, -NorthEastDownVectors.EastVector);

            return NorthEastDownVectors;
        }

        /// <summary>
        /// Calculates the latitude and longitude at the given East, North, and Up vectors.
        /// </summary>
        /// <param name="NorthEastDownVectors">The vectors pointing to the North, to the East, and toward the center of the Earth</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        public static void CalculateLatLonFromNorthEastDownVectors(FNorthEastDown NorthEastDownVectors, out double latitudeDegrees, out double longitudeDegrees)
        {
            longitudeDegrees = glm.Degrees(Math.Acos(Vector3.Dot(new Vector3(0, 1, 0), NorthEastDownVectors.EastVector) / NorthEastDownVectors.EastVector.magnitude));
            latitudeDegrees = glm.Degrees(Math.Acos(Vector3.Dot(new Vector3(0, 0, 1), NorthEastDownVectors.NorthVector) / NorthEastDownVectors.NorthVector.magnitude));
        }

        /// <summary>
        /// Calculates the DIS orientation values Psi, Theta, and Phi in degrees with the given Heading, Pitch, and Roll in degrees at the given Latitude and Longitude.
        /// </summary>
        /// <param name="HeadingPitchRollDegrees">The degrees from North of the facing direction (heading), the radians rotated about the local X axis (pitch), and the radians rotated about the local Z axis (roll)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The rotation about the Y axis in degrees (Psi),the rotation about the X axis in degrees (Theta), the rotation about the Z axis in degrees (Phi)</return>
        public static FPsiThetaPhi CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(FHeadingPitchRoll HeadingPitchRollDegrees, double latitudeDegrees, double longitudeDegrees)
        {
            FNorthEastDown NorthEastDownVectors = CalculateNorthEastDownVectorsFromLatLon(latitudeDegrees, longitudeDegrees);

            ApplyHeadingPitchRollToNorthEastDownVector(HeadingPitchRollDegrees, NorthEastDownVectors, out Vector3 X, out Vector3 Y, out _);

            Vector3 X0 = new Vector3(1, 0, 0);
            Vector3 Y0 = new Vector3(0, 1, 0);
            Vector3 Z0 = new Vector3(0, 0, 1);

            FPsiThetaPhi psiThetaPhiDegrees;
            psiThetaPhiDegrees.Psi = (float)glm.Degrees(Math.Atan2(Vector3.Dot(X, Y0), Vector3.Dot(X, X0)));
            psiThetaPhiDegrees.Theta = (float)glm.Degrees(Math.Atan2(-Vector3.Dot(X, Z0), Math.Sqrt(Math.Pow(Vector3.Dot(X, X0), 2) + Math.Pow(Vector3.Dot(X, Y0), 2))));

            NorthEastDownVectors.NorthVector = X0;
            NorthEastDownVectors.EastVector = Y0;
            NorthEastDownVectors.DownVector = Z0;

            ApplyHeadingPitchToNorthEastDownVector(psiThetaPhiDegrees.Psi, psiThetaPhiDegrees.Theta, NorthEastDownVectors, out _, out Vector3 Y2, out Vector3 Z2);

            psiThetaPhiDegrees.Phi = (float)glm.Degrees(Math.Atan2(Vector3.Dot(Y, Z2), Vector3.Dot(Y, Y2)));

            return psiThetaPhiDegrees;
        }

        /// <summary>
        /// Calculates the DIS orientation values Psi, Theta, and Phi in radians with the given Heading, Pitch, and Roll in radians at the given Latitude and Longitude.
        /// </summary>
        /// <param name="HeadingPitchRollRadians">The radians from North of the facing direction (heading), the radians rotated about the local X axis (pitch), and the radians rotated about the local Z axis (roll)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The rotation about the Y axis in radians (Psi),the rotation about the X axis in radians (Theta), the rotation about the Z axis in radians (Phi)</return>
        public static FPsiThetaPhi CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(FHeadingPitchRoll HeadingPitchRollRadians, double latitudeDegrees, double longitudeDegrees)
        {
            FHeadingPitchRoll HeadingPitchRollDegrees;
            HeadingPitchRollDegrees.Heading = glm.Degrees(HeadingPitchRollRadians.Heading);
            HeadingPitchRollDegrees.Pitch = glm.Degrees(HeadingPitchRollRadians.Pitch);
            HeadingPitchRollDegrees.Roll = glm.Degrees(HeadingPitchRollRadians.Roll);

            return CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(HeadingPitchRollDegrees, latitudeDegrees, longitudeDegrees);
        }

        /// <summary>
        /// Calculates the DIS orientation values Psi, Theta, and Phi in radians with the given Heading, Pitch, and Roll in degrees at the given Latitude and Longitude.
        /// </summary>
        /// <param name="HeadingPitchRollDegrees">The degrees from North of the facing direction (heading), the radians rotated about the local X axis (pitch), and the radians rotated about the local Z axis (roll)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The rotation about the Y axis in radians (Psi),the rotation about the X axis in radians (Theta), the rotation about the Z axis in radians (Phi)</return>
        public static FPsiThetaPhi CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(FHeadingPitchRoll HeadingPitchRollDegrees, double latitudeDegrees, double longitudeDegrees)
        {
            FPsiThetaPhi psiThetaPhiRadians = CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(HeadingPitchRollDegrees, latitudeDegrees, longitudeDegrees);
            psiThetaPhiRadians.Psi = glm.Radians(psiThetaPhiRadians.Psi);
            psiThetaPhiRadians.Theta = glm.Radians(psiThetaPhiRadians.Theta);
            psiThetaPhiRadians.Phi = glm.Radians(psiThetaPhiRadians.Phi);

            return psiThetaPhiRadians;
        }

        /// <summary>
        /// Calculates the DIS orientation values Psi, Theta, and Phi in degrees with the given Heading, Pitch, and Roll in radians at the given Latitude and Longitude.
        /// </summary>
        /// <param name="HeadingPitchRollRadians">The radians from North of the facing direction (heading), the radians rotated about the local X axis (pitch), and the radians rotated about the local Z axis (roll)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The rotation about the Y axis in degrees (Psi),the rotation about the X axis in degrees (Theta), the rotation about the Z axis in degrees (Phi)</return>
        public static FPsiThetaPhi CalculatePsiThetaPhiDegreesFromHeadingPitchRollRadiansAtLatLon(FHeadingPitchRoll HeadingPitchRollRadians, double latitudeDegrees, double longitudeDegrees)
        {
            FHeadingPitchRoll headingPitchRollDegrees;
            headingPitchRollDegrees.Heading = glm.Degrees(HeadingPitchRollRadians.Heading);
            headingPitchRollDegrees.Pitch = glm.Degrees(HeadingPitchRollRadians.Pitch);
            headingPitchRollDegrees.Roll = glm.Degrees(HeadingPitchRollRadians.Roll);

            return CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees, latitudeDegrees, longitudeDegrees);
        }

        /// <summary>
        /// Calculates the Heading, Pitch, and Roll in degrees from the given DIS orientation values Psi, Theta, and Phi in degrees at the given Latitude and Longitude.
        /// </summary>
        /// <param name="psiThetaPhiDegrees">The rotation about the Y axis in degrees (Psi),the rotation about the X axis in degrees (Theta), the rotation about the Z axis in degrees (Phi)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The degrees from North of the facing direction (heading), the degrees rotated about the local X axis (pitch), and the degrees rotated about the local Z axis (roll)</return>
        public static FHeadingPitchRoll CalculateHeadingPitchRollDegreesFromPsiThetaPhiDegreesAtLatLon(FPsiThetaPhi psiThetaPhiDegrees, double latitudeDegrees, double longitudeDegrees)
        {
            FNorthEastDown NorthEastDownVectors = CalculateNorthEastDownVectorsFromLatLon(latitudeDegrees, longitudeDegrees);

            Vector3 x0 = new Vector3(1, 0, 0);
            Vector3 y0 = new Vector3(0, 1, 0);
            Vector3 z0 = new Vector3(0, 0, 1);

            FNorthEastDown StartingVectorsForRotation = new FNorthEastDown(x0, y0, z0);

            FHeadingPitchRoll headingPitchRollTemp = new FHeadingPitchRoll(psiThetaPhiDegrees.Psi, psiThetaPhiDegrees.Theta, psiThetaPhiDegrees.Phi);

            ApplyHeadingPitchRollToNorthEastDownVector(headingPitchRollTemp, StartingVectorsForRotation, out Vector3 x3, out Vector3 y3, out _);

            FHeadingPitchRoll headingPitchRollDegrees;
            headingPitchRollDegrees.Heading = (float)glm.Degrees(Math.Atan2(Vector3.Dot(x3, NorthEastDownVectors.EastVector), Vector3.Dot(x3, NorthEastDownVectors.NorthVector)));
            headingPitchRollDegrees.Pitch = (float)glm.Degrees(Math.Atan2(-Vector3.Dot(x3, NorthEastDownVectors.DownVector), Math.Sqrt(Math.Pow(Vector3.Dot(x3, NorthEastDownVectors.EastVector), 2) + Math.Pow(Vector3.Dot(x3, NorthEastDownVectors.NorthVector), 2))));

            ApplyHeadingPitchToNorthEastDownVector(headingPitchRollDegrees.Heading, headingPitchRollDegrees.Pitch, NorthEastDownVectors, out _, out Vector3 y2, out Vector3 z2);
            headingPitchRollDegrees.Roll = (float)glm.Degrees(Math.Atan2(Vector3.Dot(y3, z2), Vector3.Dot(y3, y2)));

            return headingPitchRollDegrees;
        }

        /// <summary>
        /// Calculates the Heading, Pitch, and Roll in radians from the given DIS orientation values Psi, Theta, and Phi in radians at the given Latitude and Longitude.
        /// </summary>
        /// <param name="psiThetaPhiDegrees">The rotation about the Y axis in degrees (Psi),the rotation about the X axis in degrees (Theta), the rotation about the Z axis in degrees (Phi)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The radians from North of the facing direction (heading), the degrees rotated about the local X axis (pitch), and the degrees rotated about the local Z axis (roll)</return>
        public static FHeadingPitchRoll CalculateHeadingPitchRollRadiansFromPsiThetaPhiDegreesAtLatLon(FPsiThetaPhi psiThetaPhiDegrees, double latitudeDegrees, double longitudeDegrees)
        {
            FHeadingPitchRoll headingPitchRollRadians = CalculateHeadingPitchRollDegreesFromPsiThetaPhiDegreesAtLatLon(psiThetaPhiDegrees, latitudeDegrees, longitudeDegrees);
            headingPitchRollRadians.Heading = glm.Radians(headingPitchRollRadians.Heading);
            headingPitchRollRadians.Pitch = glm.Radians(headingPitchRollRadians.Pitch);
            headingPitchRollRadians.Roll = glm.Radians(headingPitchRollRadians.Roll);

            return headingPitchRollRadians;
        }

        /// <summary>
        /// Calculates the Heading, Pitch, and Roll in degrees from the given DIS orientation values Psi, Theta, and Phi in radians at the given Latitude and Longitude.
        /// </summary>
        /// <param name="psiThetaPhiRadians">The rotation about the Y axis in radians (Psi),the rotation about the X axis in radians (Theta), the rotation about the Z axis in radians (Phi)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The degrees from North of the facing direction (heading), the degrees rotated about the local X axis (pitch), and the degrees rotated about the local Z axis (roll)</return>
        public static FHeadingPitchRoll CalculateHeadingPitchRollDegreesFromPsiThetaPhiRadiansAtLatLon(FPsiThetaPhi psiThetaPhiRadians, double latitudeDegrees, double longitudeDegrees)
        {
            FPsiThetaPhi psiThetaPhiDegrees;
            psiThetaPhiDegrees.Psi = glm.Degrees(psiThetaPhiRadians.Psi);
            psiThetaPhiDegrees.Theta = glm.Degrees(psiThetaPhiRadians.Theta);
            psiThetaPhiDegrees.Phi = glm.Degrees(psiThetaPhiRadians.Phi);

            return CalculateHeadingPitchRollDegreesFromPsiThetaPhiDegreesAtLatLon(psiThetaPhiDegrees, latitudeDegrees, longitudeDegrees);
        }

        /// <summary>
        /// Calculates the Heading, Pitch, and Roll in radians from the given DIS orientation values Psi, Theta, and Phi in degrees at the given Latitude and Longitude.
        /// </summary>
        /// <param name="psiThetaPhiRadians">The rotation about the Y axis in radians (Psi),the rotation about the X axis in radians (Theta), the rotation about the Z axis in radians (Phi)</param>
        /// <param name="latitudeDegrees">The target latitude given in degrees</param>
        /// <param name="longitudeDegrees">The target longitude given in degrees</param>
        /// <return>The radians from North of the facing direction (heading), the degrees rotated about the local X axis (pitch), and the degrees rotated about the local Z axis (roll)</return>
        public static FHeadingPitchRoll CalculateHeadingPitchRollRadiansFromPsiThetaPhiRadiansAtLatLon(FPsiThetaPhi psiThetaPhiRadians, double latitudeDegrees, double longitudeDegrees)
        {
            FPsiThetaPhi psiThetaPhiDegrees;
            psiThetaPhiDegrees.Psi = glm.Degrees(psiThetaPhiRadians.Psi);
            psiThetaPhiDegrees.Theta = glm.Degrees(psiThetaPhiRadians.Theta);
            psiThetaPhiDegrees.Phi = glm.Degrees(psiThetaPhiRadians.Phi);

            return CalculateHeadingPitchRollRadiansFromPsiThetaPhiDegreesAtLatLon(psiThetaPhiDegrees, latitudeDegrees, longitudeDegrees);
        }

        /// <summary>
        /// Get the Unity rotation from the given Heading, Pitch, Roll rotation in degrees.
        /// </summary>
        /// <param name="HeadingPitchRollDegrees">The Heading, Pitch, Roll rotation in degrees to get the Unity rotation from.</param>
        /// <param name="LatitudeDegrees">The target latitude given in degrees</param>
        /// <param name="LongitudeDegrees">The target longitude given in degrees</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing script reference.</param>
        /// <return>The Unity rotation of the given Heading, Pitch, Roll rotation</return>
        public static Vector3 GetUnityRotationFromHeadingPitchRollDegreesAtLatLon(FHeadingPitchRoll HeadingPitchRollDegrees, double LatitudeDegrees, double LongitudeDegrees, GeoreferenceSystem GeoReferencingSystem)
        {
            if (GeoReferencingSystem == null)
            {
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return Vector3.zero;
            }

            FHeadingPitchRoll headingPitchRollRadians;
            headingPitchRollRadians.Heading = glm.Radians(HeadingPitchRollDegrees.Heading);
            headingPitchRollRadians.Pitch = glm.Radians(HeadingPitchRollDegrees.Pitch);
            headingPitchRollRadians.Roll = glm.Radians(HeadingPitchRollDegrees.Roll);

            return GetUnityRotationFromHeadingPitchRollRadiansAtLatLon(headingPitchRollRadians, LatitudeDegrees, LongitudeDegrees, GeoReferencingSystem);
        }

        /// <summary>
        /// Get the Unity rotation from the given Heading, Pitch, Roll rotation in radians.
        /// </summary>
        /// <param name="HeadingPitchRollRadians">The Heading, Pitch, Roll rotation in radians to get the Unity rotation from.</param>
        /// <param name="LatitudeDegrees">The target latitude given in degrees</param>
        /// <param name="LongitudeDegrees">The target longitude given in degrees</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing script reference.</param>
        /// <return>The Unity rotation of the given Heading, Pitch, Roll rotation</return>
        public static Vector3 GetUnityRotationFromHeadingPitchRollRadiansAtLatLon(FHeadingPitchRoll HeadingPitchRollRadians, double LatitudeDegrees, double LongitudeDegrees, GeoreferenceSystem GeoReferencingSystem)
        {
            if (GeoReferencingSystem == null)
            {
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return Vector3.zero;
            }

            FPsiThetaPhi psiThetaPhiRadians = CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(HeadingPitchRollRadians, LatitudeDegrees, LongitudeDegrees);

            return GetUnityRotationFromPsiThetaPhiRadiansAtLatLon(psiThetaPhiRadians, LatitudeDegrees, LongitudeDegrees, GeoReferencingSystem);
        }

        /// <summary>
        /// Get the Unity rotation from the given Psi, Theta, Phi rotation in radians.
        /// </summary>
        /// <param name="PsiThetaPhiDegrees">The Psi, Theta, Phi rotation in degrees to get the Unity rotation from.</param>
        /// <param name="LatitudeDegrees">The target latitude given in degrees</param>
        /// <param name="LongitudeDegrees">The target longitude given in degrees</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing script reference.</param>
        /// <return>The Unity rotation of the given Psi, Theta, Phi rotation</return>
        public static Vector3 GetUnityRotationFromPsiThetaPhiDegreesAtLatLon(FPsiThetaPhi PsiThetaPhiDegrees, double LatitudeDegrees, double LongitudeDegrees, GeoreferenceSystem GeoReferencingSystem)
        {
            if (GeoReferencingSystem == null)
            {
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return Vector3.zero;
            }

            FPsiThetaPhi psiThetaPhiRadians;
            psiThetaPhiRadians.Psi = glm.Radians(PsiThetaPhiDegrees.Psi);
            psiThetaPhiRadians.Theta = glm.Radians(PsiThetaPhiDegrees.Theta);
            psiThetaPhiRadians.Phi = glm.Radians(PsiThetaPhiDegrees.Phi);

            return GetUnityRotationFromPsiThetaPhiRadiansAtLatLon(psiThetaPhiRadians, LatitudeDegrees, LongitudeDegrees, GeoReferencingSystem);
        }

        /// <summary>
        /// Get the Unity rotation from the given Psi, Theta, Phi rotation in radians.
        /// </summary>
        /// <param name="PsiThetaPhiRadians">The Psi, Theta, Phi rotation in radians to get the Unity rotation from.</param>
        /// <param name="LatitudeDegrees">The target latitude given in degrees</param>
        /// <param name="LongitudeDegrees">The target longitude given in degrees</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing script reference.</param>
        /// <returns>The Unity rotation of the given Psi, Theta, Phi rotation</returns>
        public static Vector3 GetUnityRotationFromPsiThetaPhiRadiansAtLatLon(FPsiThetaPhi PsiThetaPhiRadians, double LatitudeDegrees, double LongitudeDegrees, GeoreferenceSystem GeoReferencingSystem)
        {
            Vector3 unityRotation = Vector3.zero;

            if (GeoReferencingSystem == null)
            {
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return unityRotation;
            }

            FNorthEastDown NorthEastDownVectors = CalculateNorthEastDownVectorsFromLatLon(LatitudeDegrees, LongitudeDegrees);

            //Get NED of the world origin
            FNorthEastDown originNorthEastDown = GeoReferencingSystem.GetNEDVectorsAtEngineLocation(Vector3.zero);

            // Get the rotational difference between calculated NED and Unity origin NED
            float XAxisRotationAngle = Vector3.Angle(NorthEastDownVectors.EastVector, originNorthEastDown.EastVector);
            float YAxisRotationAngle = Vector3.Angle(NorthEastDownVectors.DownVector, originNorthEastDown.DownVector);
            float ZAxisRotationAngle = Vector3.Angle(NorthEastDownVectors.NorthVector, originNorthEastDown.NorthVector);

            FHeadingPitchRoll HeadingPitchRollDegrees = CalculateHeadingPitchRollDegreesFromPsiThetaPhiRadiansAtLatLon(PsiThetaPhiRadians, LatitudeDegrees, LongitudeDegrees);

            //Unity Roll and Pitch axes are backwards from DIS. Invert as needed
            unityRotation.z = -HeadingPitchRollDegrees.Roll + XAxisRotationAngle;
            unityRotation.x = -HeadingPitchRollDegrees.Pitch + YAxisRotationAngle;
            unityRotation.y = HeadingPitchRollDegrees.Heading + ZAxisRotationAngle;

            return unityRotation;
        }

        /// <summary>
        /// Gets Unity rotation from a DIS entity state PDU
        /// </summary>
        /// <param name="EntityStatePduIn">The DIS PDU indicating the current state of the DIS entity</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <returns>The rotation of the entity in Unity</returns>
        public static Vector3 GetUnityRotationFromEntityStatePdu(EntityStatePdu EntityStatePduIn, GeoreferenceSystem GeoReferencingSystem)
        {
            if (GeoReferencingSystem == null)
            {
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return Vector3.zero;
            }

            FPsiThetaPhi PsiThetaPhiRadians = new FPsiThetaPhi(EntityStatePduIn.EntityOrientation);

            FLatLonAlt lla = CalculateLatLonHeightFromEcefXYZ(EntityStatePduIn.EntityLocation);

            return GetUnityRotationFromPsiThetaPhiRadiansAtLatLon(PsiThetaPhiRadians, lla.Latitude, lla.Longitude, GeoReferencingSystem);
        }

        /// <summary>
        /// Gets the Unity X, Y, Z coordinates and rotation from a DIS entity state PDU. Location values returned change depending on if GeoReferencing Subsystem is set to Flat Earth or Round Earth.
        /// </summary>
        /// <param name="EntityStatePdu">The DIS PDU indicating the current state of the DIS entity</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <param name="UnityLocation">The location of the entity in Unity</param>
        /// <param name="UnityRotation">The rotation of the entity in Unity</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        public static void GetUnityLocationAndOrientationFromEntityStatePdu(EntityStatePdu EntityStatePdu, GeoreferenceSystem GeoReferencingSystem, out Vector3 UnityLocation, out Vector3 UnityRotation, Vector3 OriginRebasingOffset = default)
        {
            if (GeoReferencingSystem == null)
            {
                UnityLocation = new Vector3();
                UnityRotation = Vector3.zero;
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return;
            }

            UnityLocation = GeoReferencingSystem.ECEFToUnityRoundEarth(EntityStatePdu.EntityLocation, OriginRebasingOffset);
            UnityRotation = GetUnityRotationFromEntityStatePdu(EntityStatePdu, GeoReferencingSystem);
        }

        /// <summary>
        /// Gets the Heading, Pitch, and Roll in degrees of the given Unity rotation at the given Unity location
        /// </summary>
        /// <param name="UnityRotation">The Unity rotation in world space</param>
        /// <param name="UnityLocation">The Unity location in world space</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <returns>The heading from North, pitch, and roll in degrees</returns>
        public static FHeadingPitchRoll GetHeadingPitchRollFromUnityRotation(Vector3 UnityRotation, Vector3 UnityLocation, GeoreferenceSystem GeoReferencingSystem)
        {
            FHeadingPitchRoll headingPitchRoll = new FHeadingPitchRoll();

            if (GeoReferencingSystem == null)
            {
                Debug.LogError("Invalid GeoReference was passed to get Unity location from. Returning Unity location of (0, 0, 0).");
                return headingPitchRoll;
            }

            FNorthEastDown NorthEastDownVectors = GeoReferencingSystem.GetNEDVectorsAtEngineLocation(UnityLocation);

            //Get NED of the world origin
            FNorthEastDown OriginNorthEastDown = GeoReferencingSystem.GetNEDVectorsAtEngineLocation(Vector3.zero);

            // Get the rotational difference between calculated NED and Unity origin NED
            float XAxisRotationAngle = Vector3.Angle(NorthEastDownVectors.EastVector, OriginNorthEastDown.EastVector);
            float YAxisRotationAngle = Vector3.Angle(NorthEastDownVectors.DownVector, OriginNorthEastDown.DownVector);
            float ZAxisRotationAngle = Vector3.Angle(NorthEastDownVectors.NorthVector, OriginNorthEastDown.NorthVector);

            //Unity Roll and Pitch axes are backwards from DIS. Invert as needed
            headingPitchRoll.Roll = -UnityRotation.z - XAxisRotationAngle;
            headingPitchRoll.Pitch = -UnityRotation.x - YAxisRotationAngle;
            headingPitchRoll.Heading = UnityRotation.y - ZAxisRotationAngle;

            return headingPitchRoll;
        }

        /// <summary>
        /// Gets the Psi (Yaw), Theta (Pitch), and Phi (Roll) in degrees of the given Unity rotation at the given Unity location
        /// </summary>
        /// <param name="UnityRotation">The Unity rotation in world space</param>
        /// <param name="UnityLocation">The Unity location in world space</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <returns>The Psi (Yaw), Theta (Pitch), and Phi (Roll) in degrees</returns>
        public static FPsiThetaPhi GetPsiThetaPhiDegreesFromUnityRotation(Vector3 UnityRotation, Vector3 UnityLocation, GeoreferenceSystem GeoReferencingSystem)
        {
            FHeadingPitchRoll headingPitchRollDegrees = GetHeadingPitchRollFromUnityRotation(UnityRotation, UnityLocation, GeoReferencingSystem);
            FLatLonAlt lla = GeoReferencingSystem.UnityFlatearthToLatLonAlt(UnityLocation);

            return CalculatePsiThetaPhiDegreesFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees, lla.Latitude, lla.Longitude);
        }

        /// <summary>
        /// Gets the Psi (Yaw), Theta (Pitch), and Phi (Roll) in radians of the given Unity rotation at the given Unity location
        /// </summary>
        /// <param name="UnityRotation">The Unity rotation in world space</param>
        /// <param name="UnityLocation">The Unity location in world space</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <returns>The Psi (Yaw), Theta (Pitch), and Phi (Roll) in radians</returns>
        public static FPsiThetaPhi GetPsiThetaPhiRadiansFromUnityRotation(Vector3 UnityRotation, Vector3 UnityLocation, GeoreferenceSystem GeoReferencingSystem)
        {
            FHeadingPitchRoll headingPitchRollDegrees = GetHeadingPitchRollFromUnityRotation(UnityRotation, UnityLocation, GeoReferencingSystem);
            FLatLonAlt lla = GeoReferencingSystem.UnityFlatearthToLatLonAlt(UnityLocation);

            return CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees, lla.Latitude, lla.Longitude);
        }

        /// <summary>
        /// Gets the ECEF location and the Psi (Yaw), Theta (Pitch), and Phi (Roll) in degrees of the given Unity rotation at the given Unity location
        /// </summary>
        /// <param name="UnityRotation">The Unity rotation in world space</param>
        /// <param name="UnityLocation">The Unity location in world space</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <param name="EcefXYZ">The ECEF location of the given Unity location.</param>
        /// <param name="PsiThetaPhiDegrees">The Psi (Yaw), Theta (Pitch), and Phi (Roll) in degrees</param>
        public static void GetEcefXYZAndPsiThetaPhiDegreesFromUnity(Vector3 UnityRotation, Vector3 UnityLocation, GeoreferenceSystem GeoReferencingSystem, out Vector3Double EcefXYZ, out FPsiThetaPhi PsiThetaPhiDegrees)
        {
            EcefXYZ = GeoReferencingSystem.UnityFlatearthToECEF(UnityLocation);
            PsiThetaPhiDegrees = GetPsiThetaPhiDegreesFromUnityRotation(UnityRotation, UnityLocation, GeoReferencingSystem);
        }

        /// <summary>
        /// Gets the ECEF location and the Psi (Yaw), Theta (Pitch), and Phi (Roll) in radians of the given Unity rotation at the given Unity location
        /// </summary>
        /// <param name="UnityRotation">The Unity rotation in world space</param>
        /// <param name="UnityLocation">The Unity location in world space</param>
        /// <param name="GeoReferencingSystem">The GeoReferencing Subsystem reference.</param>
        /// <param name="EcefXYZ">The ECEF location of the given Unity location.</param>
        /// <param name="PsiThetaPhiRadians">The Psi (Yaw), Theta (Pitch), and Phi (Roll) in radians</param>
        public static void GetEcefXYZAndPsiThetaPhiRadiansFromUnity(Vector3 UnityRotation, Vector3 UnityLocation, GeoreferenceSystem GeoReferencingSystem, out Vector3Double EcefXYZ, out FPsiThetaPhi PsiThetaPhiRadians)
        {
            EcefXYZ = GeoReferencingSystem.UnityFlatearthToECEF(UnityLocation);
            PsiThetaPhiRadians = GetPsiThetaPhiRadiansFromUnityRotation(UnityRotation, UnityLocation, GeoReferencingSystem);
        }

        /// <summary>
        /// Get the East, North, and Up vectors from the North, East, and Down vector struct
        /// </summary>
        /// <param name="NorthEastDownVectors">The North, East, and Down vectors representing the current orientation</param>
        /// <returns>The resulting East, North, and Up vectors representing the current orientation</returns>
        public static FEastNorthUp GetEastNorthUpVectorsFromNorthEastDownVectors(FNorthEastDown NorthEastDownVectors)
        {
            return new FEastNorthUp(NorthEastDownVectors.EastVector, NorthEastDownVectors.NorthVector, -NorthEastDownVectors.DownVector);
        }

        /// <summary>
        /// Get the North, East, and Down vectors from the East, North, and Up vectors
        /// </summary>
        /// <param name="EastNorthUpVectors">The East, North, and Up vectors representing the current orientation</param>
        /// <returns>The resulting North, East, and Down vectors representing the current orientation</returns>
        public static FNorthEastDown GetNorthEastDownVectorsFromEastNorthUpVectors(FEastNorthUp EastNorthUpVectors)
        {
            return new FNorthEastDown(EastNorthUpVectors.NorthVector, EastNorthUpVectors.EastVector, -EastNorthUpVectors.UpVector);
        }
    }
}