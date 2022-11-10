using System.Collections;
using System.Collections.Generic;
using OpenDis.Dis1998;
using GlmSharp;
using System;
using UnityEngine;

public class DeadReckoningLibrary
{
    private static readonly double MIN_ROTATION_RATE = 0.2 * Math.PI / 180;  // minimum significant rate = 1deg/5sec
    private static readonly double ALLOWABLE_FLOATING_POINT_ERROR = 1e-7;    // the minimum offset a floating point number can have to be approximately equal

    public static bool DeadReckoning(EntityStatePdu EntityPduToDeadReckon, float DeltaTime, ref EntityStatePdu DeadReckonedPdu)
    {
        bool supported = true;

        if ((EntityPduToDeadReckon.EntityAppearance & (1 << 21)) != 0)
        {
            return false;
        }

        switch (EntityPduToDeadReckon.DeadReckoningParameters.DeadReckoningAlgorithm)
        {
            case 1: // Static
                {
                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        //Convert Local Rotator from Heading, Pitch, Roll to Psi, Theta, Phi
                        FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(EntityPduToDeadReckon.EntityLocation);

                        FPsiThetaPhi psiThetaPhiRadians = Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(new FHeadingPitchRoll(LocalRotation), llh.Latitude, llh.Longitude);
                        DeadReckonedPdu.EntityOrientation = new Orientation
                        {
                            Psi = psiThetaPhiRadians.Psi,
                            Theta = psiThetaPhiRadians.Theta,
                            Phi = psiThetaPhiRadians.Phi
                        };
                    }

                    break;
                }
            case 2: // Fixed Position World (FPW)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(0, 0, 0);

                    dvec3 deadReckonLocation = CalculateDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        //Convert Local Rotator from Heading, Pitch, Roll to Psi, Theta, Phi
                        FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(EntityPduToDeadReckon.EntityLocation);

                        FPsiThetaPhi psiThetaPhiRadians = Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(new FHeadingPitchRoll(LocalRotation), llh.Latitude, llh.Longitude);
                        DeadReckonedPdu.EntityOrientation = new Orientation
                        {
                            Psi = psiThetaPhiRadians.Psi,
                            Theta = psiThetaPhiRadians.Theta,
                            Phi = psiThetaPhiRadians.Phi
                        };
                    }

                    break;
                }
            case 3: // Rotation Position World (RPW)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(0, 0, 0);

                    dvec3 deadReckonLocation = CalculateDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalQuaternionAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Quaternion EntityRotationQuaternion))
                    {
                        dvec3 AngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                            EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                        DeadReckonedPdu.EntityOrientation = CalculateOrientationFromQuaternion(AngularVelocity, EntityRotationQuaternion, DeltaTime);
                    }
                    else
                    {
                        dvec3 EntityAngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y,
                            EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                        //NOTE: Roll=Phi, Pitch=Theta, Yaw=Psi
                        DeadReckonedPdu.EntityOrientation = CalculateDeadReckonedOrientation(EntityPduToDeadReckon.EntityOrientation, EntityAngularVelocity, DeltaTime);
                    }

                    break;
                }
            case 4: // Rotation Velocity World (RVW)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z);

                    dvec3 deadReckonLocation = CalculateDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalQuaternionAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Quaternion EntityRotationQuaternion))
                    {
                        dvec3 AngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                            EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                        DeadReckonedPdu.EntityOrientation = CalculateOrientationFromQuaternion(AngularVelocity, EntityRotationQuaternion, DeltaTime);
                    }
                    else
                    {
                        dvec3 EntityAngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y,
                            EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                        //NOTE: Roll=Phi, Pitch=Theta, Yaw=Psi
                        DeadReckonedPdu.EntityOrientation = CalculateDeadReckonedOrientation(EntityPduToDeadReckon.EntityOrientation, EntityAngularVelocity, DeltaTime);
                    }

                    break;
                }
            case 5: // Fixed Velocity World (FVW)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z);

                    dvec3 deadReckonLocation = CalculateDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        //Convert Local Rotator from Heading, Pitch, Roll to Psi, Theta, Phi
                        FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(EntityPduToDeadReckon.EntityLocation);

                        FPsiThetaPhi psiThetaPhiRadians = Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(new FHeadingPitchRoll(LocalRotation), llh.Latitude, llh.Longitude);
                        DeadReckonedPdu.EntityOrientation = new Orientation
                        {
                            Psi = psiThetaPhiRadians.Psi,
                            Theta = psiThetaPhiRadians.Theta,
                            Phi = psiThetaPhiRadians.Phi
                        };
                    }

                    break;
                }
            case 6: // Fixed Position Body (FPB)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z);
                    dvec3 AngularVelocity = new dvec3(0, 0, 0);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularVelocity, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        //Convert Local Rotator from Heading, Pitch, Roll to Psi, Theta, Phi
                        FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(EntityPduToDeadReckon.EntityLocation);

                        FPsiThetaPhi psiThetaPhiRadians = Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(new FHeadingPitchRoll(LocalRotation), llh.Latitude, llh.Longitude);
                        DeadReckonedPdu.EntityOrientation = new Orientation
                        {
                            Psi = psiThetaPhiRadians.Psi,
                            Theta = psiThetaPhiRadians.Theta,
                            Phi = psiThetaPhiRadians.Phi
                        };
                    }

                    break;
                }
            case 7: // Rotation Position Body (RPB)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z);
                    dvec3 AngularVelocity = new dvec3(0, 0, 0);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularVelocity, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalQuaternionAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Quaternion EntityRotationQuaternion))
                    {
                        AngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                            EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                        DeadReckonedPdu.EntityOrientation = CalculateOrientationFromQuaternion(AngularVelocity, EntityRotationQuaternion, DeltaTime);
                    }
                    else
                    {
                        //NOTE: Roll=Phi, Pitch=Theta, Yaw=Psi
                        DeadReckonedPdu.EntityOrientation = CalculateDeadReckonedOrientation(EntityPduToDeadReckon.EntityOrientation, AngularVelocity, DeltaTime);
                    }

                    break;
                }
            case 8: // Rotation Velocity Body (RVB)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z);
                    dvec3 AngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularVelocity, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalQuaternionAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Quaternion EntityRotationQuaternion))
                    {
                        DeadReckonedPdu.EntityOrientation = CalculateOrientationFromQuaternion(AngularVelocity, EntityRotationQuaternion, DeltaTime);
                    }
                    else
                    {
                        //NOTE: Roll=Phi, Pitch=Theta, Yaw=Psi
                        DeadReckonedPdu.EntityOrientation = CalculateDeadReckonedOrientation(EntityPduToDeadReckon.EntityOrientation, AngularVelocity, DeltaTime);
                    }

                    break;
                }
            case 9: // Fixed Velocity Body (FVB)
                {
                    // Form GLM vectors for position, velocity, and acceleration
                    dvec3 PositionVector = new dvec3(EntityPduToDeadReckon.EntityLocation.X, EntityPduToDeadReckon.EntityLocation.Y, EntityPduToDeadReckon.EntityLocation.Z);
                    dvec3 VelocityVector = new dvec3(EntityPduToDeadReckon.EntityLinearVelocity.X, EntityPduToDeadReckon.EntityLinearVelocity.Y, EntityPduToDeadReckon.EntityLinearVelocity.Z);
                    dvec3 AccelerationVector = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z);
                    dvec3 AngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularVelocity, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        //Convert Local Rotator from Heading, Pitch, Roll to Psi, Theta, Phi
                        FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(EntityPduToDeadReckon.EntityLocation);

                        FPsiThetaPhi psiThetaPhiRadians = Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollRadiansAtLatLon(new FHeadingPitchRoll(LocalRotation), llh.Latitude, llh.Longitude);
                        DeadReckonedPdu.EntityOrientation = new Orientation
                        {
                            Psi = psiThetaPhiRadians.Psi,
                            Theta = psiThetaPhiRadians.Theta,
                            Phi = psiThetaPhiRadians.Phi
                        };
                    }

                    break;
                }
            default:
                supported = false;
                break;
        }

        //Update the linear velocity according to the acceleration of the entity
        if (supported)
        {
            DeadReckonedPdu.EntityLinearVelocity = new Vector3Float
            {
                X = DeadReckonedPdu.EntityLinearVelocity.X + EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.X * Time.deltaTime,
                Y = DeadReckonedPdu.EntityLinearVelocity.Y + EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Y * Time.deltaTime,
                Z = DeadReckonedPdu.EntityLinearVelocity.Z + EntityPduToDeadReckon.DeadReckoningParameters.EntityLinearAcceleration.Z * Time.deltaTime
            };
        }

        return supported;
    }

    /// <summary>
    /// Converts a Quaternion into an OpenDIS Orientation (Psi, Theta, Phi).
    /// </summary>
    /// <param name="entityPDUToDeadReckon">The Entity PDU being dead reckoned</param>
    /// <param name="localQuaternion">The Quaternion to convert into an Orientation (Psi, Theta, Phi).</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The OpenDIS Orientation.</returns>
    public static Orientation CalculateOrientationFromQuaternion(dvec3 AngularVelocity, Quaternion localQuaternion, float deltaTime)
    {
        Quaternion deadReckoningQuat = localQuaternion * CreateDeadReckoningQuaternion(AngularVelocity, deltaTime);

        float theta = (float)Math.Asin(-2 * ((deadReckoningQuat.x * deadReckoningQuat.z) - (deadReckoningQuat.w * deadReckoningQuat.y)));
        if (theta == (Mathf.PI / 2))
        {
            theta = (float)1e-5;
        }

        return new Orientation
        {
            Psi = (float)Math.Atan2(2 * ((deadReckoningQuat.x * deadReckoningQuat.y) + (deadReckoningQuat.w * deadReckoningQuat.z)), Math.Pow(deadReckoningQuat.w, 2) + Math.Pow(deadReckoningQuat.x, 2) - Math.Pow(deadReckoningQuat.y, 2) - Math.Pow(deadReckoningQuat.z, 2)),
            Theta = theta,
            Phi = (float)Math.Atan2(2 * ((deadReckoningQuat.y * deadReckoningQuat.z) + (deadReckoningQuat.w * deadReckoningQuat.x)), Math.Pow(deadReckoningQuat.w, 2) - Math.Pow(deadReckoningQuat.x, 2) - Math.Pow(deadReckoningQuat.y, 2) + Math.Pow(deadReckoningQuat.z, 2))
        };
    }

    /// <summary>
    /// Calculates the new position vector using the given velocity, acceleration, and time increment given in body coordinates
    /// </summary>
    /// <param name="entityLocation">The initial position of the entity in body coordinates</param>
    /// <param name="entityLinearVelocity">The initial velocity in body coordinates</param>
    /// <param name="entityLinearAcceleration">The initial linear acceleration in body coordinates</param>
    /// <param name="entityAngularVelocity">The initial angular acceleration in body coordinates</param>
    /// <param name="entityOrientation">The orientation of the entity in radians</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The position vector in body coordinates.</returns>
    public static dvec3 CalculateBodyDeadReckonedPosition(dvec3 entityLocation, dvec3 entityLinearVelocity, dvec3 entityLinearAcceleration, dvec3 entityAngularVelocity, Orientation entityOrientation, float deltaTime)
    {
        dmat3 SkewMatrix = Conversions.CreateSkewMatrix(entityAngularVelocity);
        dvec3 BodyAccelerationVector = entityLinearAcceleration - (SkewMatrix * entityLinearVelocity);

        // Get the entity's current orientation matrix
        dmat3 OrientationMatrix = GetEntityOrientationMatrix(entityOrientation);
        dmat3 InverseInitialOrientationMatrix = OrientationMatrix.Inverse;

        dmat3 OmegaMatrix = new dmat3(entityAngularVelocity, new dvec3(0), new dvec3(0)) * new dmat3(entityAngularVelocity, new dvec3(0), new dvec3(0)).Transposed;
        double AngularVelocityMagnitude = entityAngularVelocity.Length;

        dmat3 R1 = new dmat3();
        dmat3 R2 = new dmat3();

        if (AngularVelocityMagnitude < MIN_ROTATION_RATE)
        {
            R1 = dmat3.Identity * deltaTime;
            R2 = dmat3.Identity * (Math.Pow(deltaTime, 2) / 2);
        }
        else
        {
            // Calculate R1
            R1 = (((AngularVelocityMagnitude * deltaTime - Math.Sin(AngularVelocityMagnitude * deltaTime)) / Math.Pow(AngularVelocityMagnitude, 3)) * OmegaMatrix) +
                ((Math.Sin(AngularVelocityMagnitude * deltaTime) / AngularVelocityMagnitude) * new dmat3(1, 0, 0, 0, 1, 0, 0, 0, 1)) +
                (((1 - Math.Cos(AngularVelocityMagnitude * deltaTime)) / Math.Pow(AngularVelocityMagnitude, 2)) * SkewMatrix);

            R2 = ((((0.5 * Math.Pow(AngularVelocityMagnitude, 2) * Math.Pow(deltaTime, 2)) - (Math.Cos(AngularVelocityMagnitude * deltaTime)) - (AngularVelocityMagnitude * deltaTime * Math.Sin(AngularVelocityMagnitude * deltaTime)) + (1)) / Math.Pow(AngularVelocityMagnitude, 4)) * SkewMatrix * SkewMatrix.Transposed) +
                ((((Math.Cos(AngularVelocityMagnitude * deltaTime)) + (AngularVelocityMagnitude * deltaTime * Math.Sin(AngularVelocityMagnitude * deltaTime)) - (1)) / (Math.Pow(AngularVelocityMagnitude, 2))) * new dmat3(1, 0, 0, 0, 1, 0, 0, 0, 1)) +
                ((((Math.Sin(AngularVelocityMagnitude * deltaTime)) - (AngularVelocityMagnitude * deltaTime * Math.Cos(AngularVelocityMagnitude * deltaTime))) / (Math.Pow(AngularVelocityMagnitude, 3))) * SkewMatrix);
        }

        return entityLocation + (InverseInitialOrientationMatrix * ((R1 * entityLinearVelocity) + (R2 * BodyAccelerationVector)));
    }

    public static Quaternion GetEntityOrientationQuaternion(double PsiRadians, double ThetaRadians, double PhiRadians)
    {
        //Abbreviations for the various angular functions
        double cy = Math.Cos(PsiRadians / 2);
        double sy = Math.Sin(PsiRadians / 2);
        double cp = Math.Cos(ThetaRadians / 2);
        double sp = Math.Sin(ThetaRadians / 2);
        double cr = Math.Cos(PhiRadians / 2);
        double sr = Math.Sin(PhiRadians / 2);

        Quaternion entityQuaternion = new Quaternion();
        entityQuaternion.w = (float)(cr * cp * cy + sr * sp * sy);
        entityQuaternion.x = (float)(sr * cp * cy - cr * sp * sy);
        entityQuaternion.y = (float)(cr * sp * cy + sr * cp * sy);
        entityQuaternion.z = (float)(cr * cp * sy - sr * sp * cy);

        return entityQuaternion;
    }

    /// <summary>
    /// Calculates the new orientation vector using the given velocity, acceleration, and time increment
    /// </summary>
    /// <param name="entityOrientation">The orientation of the entity in radians</param>
    /// <param name="entityAngularVelocity">The angular velocity vector in body coordinates</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The dead reckoning orientation matrix based off the given Entity Orientation and Entity Angular Velocity.</returns>
    public static Orientation CalculateDeadReckonedOrientation(Orientation entityOrientation, dvec3 entityAngularVelocity, float deltaTime)
    {
        // Get the entity's current orientation matrix
        dmat3 OrientationMatrix = GetEntityOrientationMatrix(entityOrientation);

        // Get the change in rotation for this time step
        dmat3 DeadReckoningMatrix = CreateDeadReckoningMatrix(entityAngularVelocity, deltaTime);

        // Calculate the new orientation matrix
        OrientationMatrix = DeadReckoningMatrix * OrientationMatrix;

        Orientation orientationOut = new Orientation();

        // Extract Euler angles from orientation matrix
        orientationOut.Theta = (float)Math.Asin(-OrientationMatrix.m20);

        // Special case for |Theta| = pi/2
        double CosTheta = 1e-5;
        if (Math.Abs(orientationOut.Theta) != (Mathf.PI / 2))
        {
            CosTheta = glm.Cos(orientationOut.Theta);
        }

        // Calculate values that will be used in ACos function and correct any floating point errors
        double psiValueToACos = OrientationMatrix.m00 / CosTheta;
        if (Math.Abs(psiValueToACos) > 1)
        {
            if (Math.Abs(psiValueToACos) - ALLOWABLE_FLOATING_POINT_ERROR <= 1)
            {
                psiValueToACos = 1 * Math.Sign(psiValueToACos);
            }
            else
            {
                Debug.LogWarning("Calculated Dead Reckoning Orientation has invalid Psi Value");
            }
        }

        double phiValueToACos = OrientationMatrix.m22 / CosTheta;
        if (Math.Abs(phiValueToACos) > 1)
        {
            if (Math.Abs(phiValueToACos) - ALLOWABLE_FLOATING_POINT_ERROR <= 1)
            {
                phiValueToACos = 1 * Math.Sign(phiValueToACos);
            }
            else
            {
                Debug.LogWarning("Calculated Dead Reckoning Orientation has invalid Phi Value");
            }
        }

        orientationOut.Psi = (float)((Math.Acos(OrientationMatrix.m00 / CosTheta) * (Math.Abs(OrientationMatrix.m10) / OrientationMatrix.m10)));
        orientationOut.Phi = (float)((Math.Acos(OrientationMatrix.m22 / CosTheta) * (Math.Abs(OrientationMatrix.m21) / OrientationMatrix.m21)));

        return orientationOut;
    }

    /// <summary>
    /// Calculates and returns the dead reckoning matrix used for calculating entity rotation
    /// </summary>
    /// <param name="entityAngularVelocity">The angular velocity vector in body coordinates</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculations</param>
    /// <returns>The dead reckoning matrix based off the given Entity Angular Velocity.</returns>
    public static dmat3 CreateDeadReckoningMatrix(dvec3 entityAngularVelocity, float deltaTime)
    {
        double AngularVelocityMagnitude = entityAngularVelocity.Length;
        if (AngularVelocityMagnitude == 0)
        {
            AngularVelocityMagnitude = 1e-5;
            entityAngularVelocity += new dvec3(1e-5);
        }

        dmat3 AngularVelocityMatrix = new dmat3(entityAngularVelocity, new dvec3(0), new dvec3(0));
        dmat3 AngularVelocity = AngularVelocityMatrix * AngularVelocityMatrix.Transposed;

        double CosOmega = Math.Cos(AngularVelocityMagnitude * deltaTime);
        double SinOmega = Math.Sin(AngularVelocityMagnitude * deltaTime);

        dmat3 DeadReckoningMatrix = (((1 - CosOmega) / Math.Pow(AngularVelocityMagnitude, 2)) * AngularVelocity) +
            (CosOmega * dmat3.Identity) -
            (SinOmega / AngularVelocityMagnitude * Conversions.CreateSkewMatrix(entityAngularVelocity));

        return DeadReckoningMatrix;
    }

    /// <summary>
    /// Calculates and returns the dead reckoning quaternion used for calculating entity rotation
    /// </summary>
    /// <param name="AngularVelocityVector">The angular velocity vector in body coordinates</param>
    /// <param name="DeltaTime">The time increment for dead reckoning calculations</param>
    /// <returns>The dead reckoning quaternion based off the given Entity Angular Velocity.</returns>
    public static Quaternion CreateDeadReckoningQuaternion(dvec3 AngularVelocityVector, double DeltaTime)
    {
        double AngularVelocityMagnitude = AngularVelocityVector.Length;
        if (AngularVelocityMagnitude == 0)
        {
            AngularVelocityMagnitude = 1e-5;
            AngularVelocityVector += new dvec3(1e-5);
        }

        double beta = AngularVelocityMagnitude * DeltaTime;
        dvec3 unitVector = AngularVelocityVector / AngularVelocityMagnitude;

        Quaternion deadReckoningQuaternion = new Quaternion();

        deadReckoningQuaternion.w = (float)Math.Cos(beta / 2);
        deadReckoningQuaternion.x = (float)(unitVector.x * Math.Sin(beta / 2));
        deadReckoningQuaternion.y = (float)(unitVector.y * Math.Sin(beta / 2));
        deadReckoningQuaternion.z = (float)(unitVector.z * Math.Sin(beta / 2));

        return deadReckoningQuaternion;
    }

    /// <summary>
    /// Calculates a DIS entity's orientation matrix from the provided Euler angles
    /// </summary>
    /// <param name="entityOrientation">The orientation of the entity in radians</param>
    /// <returns>The orienation matrix based off the given Entity Orientation.</returns>
    public static dmat3 GetEntityOrientationMatrix(Orientation entityOrientation)
    {
        //Trig orientations
        double CosPsi = Math.Cos(entityOrientation.Psi);
        double SinPsi = Math.Sin(entityOrientation.Psi);
        double CosTheta = Math.Cos(entityOrientation.Theta);
        double SinTheta = Math.Sin(entityOrientation.Theta);
        double CosPhi = Math.Cos(entityOrientation.Phi);
        double SinPhi = Math.Sin(entityOrientation.Phi);

        dmat3 HeadingRotationMatrix = new dmat3(CosPsi, -SinPsi, 0, SinPsi, CosPsi, 0, 0, 0, 1);
        dmat3 PitchRotationMatrix = new dmat3(CosTheta, 0, SinTheta, 0, 1, 0, -SinTheta, 0, CosTheta);
        dmat3 RollRotationMatrix = new dmat3(1, 0, 0, 0, CosPhi, -SinPhi, 0, SinPhi, CosPhi);

        return RollRotationMatrix * PitchRotationMatrix * HeadingRotationMatrix;
    }

    /// <summary>
    /// Gets the local entity orientation as a quaternion from the dead reckoning other parameters
    /// </summary>
    /// <param name="otherParameters">The 120 bits sent as part of the dead reckoning parameters marked as other parameters sent as an array of bytes</param>
    /// <param name="localRotation">The four-valued unit quaternion that represents the entity's orientation</param>
    /// <returns>Whether or not the data was loaded correctly. False if not loaded correctly.</returns>
    public static bool GetLocalQuaternionAngles(byte[] otherParameters, out Quaternion localRotation)
    {
        localRotation = new Quaternion();
        // Ensure the array is at least 15 bytes long
        if (otherParameters.Length < 15) { return false; }

        // Ensure the DR Parameter type is set to 2
        if (otherParameters[0] != 2) { return false; }

        //The next two bytes represent the 16 bit unsigned int approximation of q_0 (q_w in Unity terminology)
        //float Qu0 = (ushort)((otherParameters[1] << 8) + otherParameters[2]);

        // The quaternion x, y, and z components are the next three groups of four bytes respectively
        byte[] byteQuX = new byte[] { otherParameters[3], otherParameters[4], otherParameters[5], otherParameters[6] };
        byte[] byteQuY = new byte[] { otherParameters[7], otherParameters[8], otherParameters[9], otherParameters[10] };
        byte[] byteQuZ = new byte[] { otherParameters[11], otherParameters[12], otherParameters[13], otherParameters[14] };

        //Check indianness. DIS is Big Endian, so reverse the array order if in little endian
        if (BitConverter.IsLittleEndian)
        {
            Array.Reverse(byteQuX);
            Array.Reverse(byteQuY);
            Array.Reverse(byteQuZ);
        }

        //Convert bytes back to floats
        float QuX = BitConverter.ToSingle(byteQuX, 0);
        float QuY = BitConverter.ToSingle(byteQuY, 0);
        float QuZ = BitConverter.ToSingle(byteQuZ, 0);

        // Calculate the appropriate Qu0
        float Qu0 = (float)Math.Sqrt(1 - (Math.Pow(QuX, 2.0) + Math.Pow(QuY, 2.0) + Math.Pow(QuZ, 2.0)));

        // Set the values and return
        localRotation.Set(QuX, QuY, QuZ, Qu0);
        return true;
    }

    /// <summary>
    /// Calculates the new position vector using the given velocity, acceleration, and time increment
    /// </summary>
    /// <param name="entityLocation">The initial position of the entity in world coordinates</param>
    /// <param name="entityLinearVelocity">The initial velocity in world coordinates</param>
    /// <param name="entityAcceleration">The initial acceleration in world coordinates</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The calulated dead reckoned position.</returns>
    public static dvec3 CalculateDeadReckonedPosition(dvec3 entityLocation, dvec3 entityLinearVelocity, dvec3 entityAcceleration, float deltaTime)
    {
        return entityLocation + (entityLinearVelocity * deltaTime) + (0.5 * entityAcceleration * Math.Pow(deltaTime, 2));
    }

    /// <summary>
    /// Gets the local yaw, pitch, and roll from the other parameters structure. The yaw, pitch, and roll act on the entity's local North, East, Down vectors.
    /// </summary>
    /// <param name="otherParameters">The 120 bits sent as part of the dead reckoning parameters marked as other parameters sent as an array of bytes</param>
    /// <param name="localRotation">The local yaw, pitch, and roll of the entity. Yaw is the heading from true north, positive to the right, in radians. Pitch is the elevation angle above or below the local horizon, positive up, in radians. Roll is the bank angle from the local horizontal, positive tile to the right, in radians.</param>
    /// <returns>Whether or not the data was loaded correctly. False if not loaded correctly.</returns>
    public static bool GetLocalEulerAngles(byte[] otherParameters, out Orientation localRotation)
    {
        localRotation = null;
        // Ensure the Array is at least 15 bytes long
        if (otherParameters.Length < 15) { return false; }

        // Ensure the DR Parameter type is set to 1
        if (otherParameters[0] != 1) { return false; }

        // The next 2 bytes are padding and not necessary so skip indices 1 and 2
        // The heading, pitch, and roll are the next three groups of four bytes respectively
        byte[] byteHeading = new byte[] { otherParameters[3], otherParameters[4], otherParameters[5], otherParameters[6] };
        byte[] bytePitch = new byte[] { otherParameters[7], otherParameters[8], otherParameters[9], otherParameters[10] };
        byte[] byteRoll = new byte[] { otherParameters[11], otherParameters[12], otherParameters[13], otherParameters[14] };

        //Check indianness. DIS is Big Endian, so reverse the array order if in little endian
        if (BitConverter.IsLittleEndian)
        {
            Array.Reverse(byteHeading);
            Array.Reverse(bytePitch);
            Array.Reverse(byteRoll);
        }

        //Convert bytes back to floats
        float LocalYaw = BitConverter.ToSingle(byteHeading, 0);
        float LocalPitch = BitConverter.ToSingle(bytePitch, 0);
        float LocalRoll = BitConverter.ToSingle(byteRoll, 0);

        localRotation = new Orientation
        {
            Psi = LocalYaw,
            Theta = LocalPitch,
            Phi = LocalRoll
        };

        return true;
    }

    /// <summary>
    /// Forms the Other Parameters section utilized in Dead Reckoning Parameters.
    /// </summary>
    /// <param name="DeadReckoningAlgorithm">The dead reckoning algorithm being used.</param>
    /// <param name="EntityPsiThetaPhiRadians">The Psi, Theta, Phi orientation of the entity.</param>
    /// <param name="EntityECEFLocation">The ECEF location of the entity.</param>
    /// <returns>A byte array containing the other parameters values.</returns>
    public static byte[] FormOtherParameters(EDeadReckoningAlgorithm DeadReckoningAlgorithm, Orientation EntityPsiThetaPhiRadians, Vector3Double EntityECEFLocation)
    {
        byte[] otherParameters = new byte[15];

        byte[] charHeading = new byte[sizeof(float)];
        byte[] charPitch = new byte[sizeof(float)];
        byte[] charRoll = new byte[sizeof(float)];

        if (DeadReckoningAlgorithm == EDeadReckoningAlgorithm.Static || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.FPW || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.FVW
            || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.FPB || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.FVB)
        {
            otherParameters[0] = 1;
            //Padding: Unused
            otherParameters[1] = 0;
            otherParameters[2] = 0;

            FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(EntityECEFLocation);
            FHeadingPitchRoll headingPitchRollRadians = Conversions.CalculateHeadingPitchRollRadiansFromPsiThetaPhiRadiansAtLatLon(new FPsiThetaPhi(EntityPsiThetaPhiRadians), llh.Latitude, llh.Longitude);

            //Convert the floats to unsigned char arrays
            charHeading = BitConverter.GetBytes((float)headingPitchRollRadians.Heading);
            charPitch = BitConverter.GetBytes((float)headingPitchRollRadians.Pitch);
            charRoll = BitConverter.GetBytes((float)headingPitchRollRadians.Roll);

            //Check indianness. DIS is Big Endian, so reverse the array order if in little endian
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(charHeading);
                Array.Reverse(charPitch);
                Array.Reverse(charRoll);
            }
        }
        else if (DeadReckoningAlgorithm == EDeadReckoningAlgorithm.RPW || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.RVW || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.RPB || DeadReckoningAlgorithm == EDeadReckoningAlgorithm.RVB)
        {
            otherParameters[0] = 2;

            //Calculate the four quaternion terms
            float cosPsiHalved = Mathf.Cos(EntityPsiThetaPhiRadians.Psi / 2);
            float cosThetaHalved = Mathf.Cos(EntityPsiThetaPhiRadians.Theta / 2);
            float cosPhiHalved = Mathf.Cos(EntityPsiThetaPhiRadians.Phi / 2);
            float sinPsiHalved = Mathf.Sin(EntityPsiThetaPhiRadians.Psi / 2);
            float sinThetaHalved = Mathf.Sin(EntityPsiThetaPhiRadians.Theta / 2);
            float sinPhiHalved = Mathf.Sin(EntityPsiThetaPhiRadians.Phi / 2);

            float qu0 = cosPsiHalved * cosThetaHalved * cosPhiHalved + sinPsiHalved * sinThetaHalved * sinPhiHalved;
            float qux = cosPsiHalved * cosThetaHalved * sinPhiHalved - sinPsiHalved * sinThetaHalved * cosPhiHalved;
            float quy = cosPsiHalved * sinThetaHalved * cosPhiHalved + sinPsiHalved * cosThetaHalved * sinPhiHalved;
            float quz = sinPsiHalved * cosThetaHalved * cosPhiHalved - cosPsiHalved * sinThetaHalved * sinPhiHalved;

            float Qu0 = (float)Math.Sqrt(1 - (Math.Pow(qux, 2.0) + Math.Pow(quy, 2.0) + Math.Pow(quz, 2.0)));

            //If qu0 is negative, invert all terms
            if (qu0 < 0)
            {
                qu0 *= -1;
                qux *= -1;
                quy *= -1;
                quz *= -1;
            }
            //qu0 could be greater than 0 due to rounding error. Form finalQu0 based on this.
            byte[] finalQu0 = BitConverter.GetBytes(Convert.ToUInt16((qu0 >= 1) ? 65535 : (int)Math.Truncate(qu0 * 65536)));

            //Convert the floats to unsigned char arrays
            charHeading = BitConverter.GetBytes(qux);
            charPitch = BitConverter.GetBytes(quy);
            charRoll = BitConverter.GetBytes(quz);

            //Check indianness. DIS is Big Endian, so reverse the array order if in little endian
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(finalQu0);
                Array.Reverse(charHeading);
                Array.Reverse(charPitch);
                Array.Reverse(charRoll);
            }

            //Fill out Qu0 portion of Other Parameters
            otherParameters[1] = finalQu0[0];
            otherParameters[2] = finalQu0[1];
        }
        else
        {
            //Unknown Dead Reckoning algorithm being used
            return otherParameters;
        }

        Buffer.BlockCopy(charHeading, 0, otherParameters, 3, 4);
        Buffer.BlockCopy(charPitch, 0, otherParameters, 7, 4);
        Buffer.BlockCopy(charRoll, 0, otherParameters, 11, 4);

        return otherParameters;
    }
}
