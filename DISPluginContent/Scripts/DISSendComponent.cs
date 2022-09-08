using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenDis.Dis1998;
using System;
using GlmSharp;

public class DISSendComponent : MonoBehaviour
{
    public DISGameManager disGameManagerScript;

    /// <summary>
    /// The Entity Type of the associated entity.Specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.
    /// </summary>
    [Header("DIS Info")]
    [Tooltip("The Entity Type of the associated entity. Specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.")]
    public EntityTypeEditor CurrentEntityType;
    /// <summary>
    /// The Entity ID of the associated entity.Each Entity ID should be unique to an entity in the sim.
    /// </summary>
    [Tooltip("The Entity ID of the associated entity. Each Entity ID should be unique to an entity in the sim.")]
    public EntityIDEditor CurrentEntityID;
    /// <summary>
    /// The Force ID of the associated entity.Specifies the team or side the DIS entity is on.
    /// </summary>
    [Tooltip("The Force ID of the associated entity. Specifies the team or side the DIS entity is on.")]
    public EForceID EntityForceID;
    /// <summary>
    /// The Entity Marking of the associated entity.Designates a friendly name for the DIS entity.Max of 11 characters should be used.If more than 11 are used, it will be truncated.
    /// </summary>
    [Tooltip("The Entity Marking of the associated entity. Designates a friendly name for the DIS entity. Max of 11 characters should be used.If more than 11 are used, it will be truncated.")]
    public string EntityMarking;

    /// <summary>
    /// The time to live for the entity.Gets reset every time a new Entity State PDU is received by the sim.
    /// </summary>
    [Header("DIS Settings")]
    [Min(0.0f)]
    [Tooltip("The time to live for the entity. Gets reset every time a new Entity State PDU is received by the sim.")]
    public float DISHeartbeatSeconds = 5.0f;
    /// <summary>
    /// The sending mode that this entity should use for Entity State PDUs.
    /// </summary>
    [Tooltip("The sending mode that this entity should use for Entity State PDUs.")]
    public EEntityStateSendingMode EntityStatePDUSendingMode = EEntityStateSendingMode.None;
    /// <summary>
    /// The dead reckoning algorithm to use.Specifies the dynamic changes to the entities appearance attributes.
    /// </summary>
    [Min(0.0f)]
    [Tooltip("The dead reckoning algorithm to use. Specifies the dynamic changes to the entities appearance attributes.")]
    public int EntityAppearance = 0;
    /// <summary>
    /// The DIS Capabilities that the entity should have.Int representation of a collection of boolean fields which describe the capabilities of the entity.
    /// </summary>
    [Min(0.0f)]
    [Tooltip("The DIS Capabilities that the entity should have. Int representation of a collection of boolean fields which describe the capabilities of the entity.")]
    public int EntityCapabilities = 0;
    /// <summary>
    /// The dead reckoning algorithm to use.
    /// </summary>
    [Tooltip("The dead reckoning algorithm to use.")]
    public EDeadReckoningAlgorithm DeadReckoningAlgorithm = EDeadReckoningAlgorithm.Static;
    /// <summary>
    /// The position threshold to use for dead reckoning.If the dead reckoning position deviates more than this value away from the actual position in any axis, a new Entity State PDU will be sent.
    /// This value should be in meters.
    /// </summary>
    [Min(0.0f)]
    [Tooltip("The position threshold to use for dead reckoning. If the dead reckoning position deviates more than this value away from the actual position in any axis, a new Entity State PDU will be sent. This value should be in meters.")]
    public float DeadReckoningPositionThresholdMeters = 1;
    /// <summary>
    /// The orientation threshold to use for dead reckoning.If the dead reckoning orientation deviates more than this value away from the actual orientation, a new Entity State PDU will be sent.
    /// This value should be in degrees.
    /// </summary>
    [Min(0.0f)]
    [Tooltip("The orientation threshold to use for dead reckoning. If the dead reckoning orientation deviates more than this value away from the actual orientation, a new Entity State PDU will be sent. This value should be in degrees.")]
    public float DeadReckoningOrientationThresholdDegrees = 3;

    /// <summary>
    /// The most recent Entity State PDU that has been received.
    /// </summary>
    [HideInInspector]
    public EntityStatePdu MostRecentEntityStatePDU = new EntityStatePdu();
    /// <summary>
    /// The most recent Dead Reckoned Entity State PDU that has been calculated.
    /// </summary>
    [HideInInspector]
    public EntityStatePdu MostRecentDeadReckoningPDU = new EntityStatePdu();

    private float DeltaTimeSinceLastPDU = 0;
    private EntityStatePdu PreviousEntityStatePDU;

    private float TimeOfLastVelocityCalculation;
    private Vector3 PreviousUnityLocation;
    private Vector3 PreviousUnityRotation;
    private Vector3 PreviousUnityLinearVelocity;
    private PDUSender pduSenderScript;

    void Start()
    {
        //Setup Previous/Current Location and Rotation variables.
        PreviousUnityLocation = transform.position;
        PreviousUnityRotation = transform.eulerAngles;

        //Form Entity State PDU packets
        MostRecentEntityStatePDU = FormEntityStatePDU();
        MostRecentDeadReckoningPDU = MostRecentEntityStatePDU;
        PreviousEntityStatePDU = MostRecentEntityStatePDU;

        pduSenderScript = (disGameManagerScript == null) ? FindObjectOfType<DISGameManager>()?.GetComponent<PDUSender>() : disGameManagerScript.GetComponent<PDUSender>();

        //Begin play with Entity State PDU
        if (pduSenderScript == null)
        {
            Debug.LogError("Invalid DISGameManager. Please make sure one is in the world.");
        }
        else if (EntityStatePDUSendingMode != EEntityStateSendingMode.None)
        {
            pduSenderScript = disGameManagerScript.GetComponent<PDUSender>();
            pduSenderScript.SendPdu(MostRecentEntityStatePDU);
        }
    }

    // Called every frame
    void Update()
    {
        DeltaTimeSinceLastPDU += Time.deltaTime;

        if (EntityStatePDUSendingMode != EEntityStateSendingMode.None)
        {
            SendEntityStatePDU();
        }

        //Update previous velocity, rotation, and location regardless of if an Entity State PDU was sent out.
        //Multiply by 100 to convert velocity to m/s
        PreviousUnityLinearVelocity = (transform.position - PreviousUnityLocation) / (Time.deltaTime * 100);
        PreviousUnityLocation = transform.position;
        PreviousUnityRotation = transform.eulerAngles;
        TimeOfLastVelocityCalculation = Time.timeSinceLevelLoad;
    }

    void OnDestroy()
    {
        //Emit a final EntityStatePDU detailing that the entity has been deactivated
        EntityStatePdu finalESPDU = new EntityStatePdu();

        finalESPDU.EntityID = CurrentEntityID.toEntityID();
        finalESPDU.EntityType = CurrentEntityType.toEntityType();
        finalESPDU.ForceId = (byte)EntityForceID;
        finalESPDU.Marking = PDUUtil.getStringAsMarking(EntityMarking);
        finalESPDU.EntityAppearance |= 1 << 23;

        if (disGameManagerScript)
        {
            finalESPDU.ExerciseID = Convert.ToByte(disGameManagerScript.ExerciseID);
        }
        else
        {
            finalESPDU.ExerciseID = Convert.ToByte(FindObjectOfType<DISGameManager>()?.ExerciseID);
        }

        EmitAppropriatePDU(finalESPDU);
    }

    void SetEntityCapabilities(int NewEntityCapabilities)
    {
        //If the new entity capabilities differ, send out a new ESPDU
        if (EntityStatePDUSendingMode == EEntityStateSendingMode.EntityStatePDU && NewEntityCapabilities != EntityCapabilities && NewEntityCapabilities >= 0)
        {
            EntityCapabilities = NewEntityCapabilities;

            MostRecentEntityStatePDU = FormEntityStatePDU();
            MostRecentDeadReckoningPDU = MostRecentEntityStatePDU;
            PreviousEntityStatePDU = MostRecentEntityStatePDU;

            if (pduSenderScript != null)
            {
                pduSenderScript.SendPdu(MostRecentEntityStatePDU);
            }
        }
    }

    void SetEntityAppearance(int NewEntityAppearance)
    {
        //If the new appearance differs, send out a new ESPDU
        if (NewEntityAppearance != EntityAppearance && NewEntityAppearance >= 0)
        {
            EntityAppearance = NewEntityAppearance;

            MostRecentEntityStatePDU = FormEntityStatePDU();
            MostRecentDeadReckoningPDU = MostRecentEntityStatePDU;
            PreviousEntityStatePDU = MostRecentEntityStatePDU;

            EmitAppropriatePDU(MostRecentEntityStatePDU);
        }
    }

    void SetDeadReckoningAlgorithm(EDeadReckoningAlgorithm NewDeadReckoningAlgorithm)
    {
        //If the dead reckoning algorithm differs and is in the appropriate range, send out a new ESPDU
        if (EntityStatePDUSendingMode == EEntityStateSendingMode.EntityStatePDU && NewDeadReckoningAlgorithm != DeadReckoningAlgorithm)
        {
            DeadReckoningAlgorithm = NewDeadReckoningAlgorithm;

            MostRecentEntityStatePDU = FormEntityStatePDU();
            MostRecentDeadReckoningPDU = MostRecentEntityStatePDU;
            PreviousEntityStatePDU = MostRecentEntityStatePDU;

            if (pduSenderScript != null)
            {
                pduSenderScript.SendPdu(MostRecentEntityStatePDU);
            }
        }
    }

    EntityStatePdu FormEntityStatePDU()
    {
        EntityStatePdu newEntityStatePDU = new EntityStatePdu();

        newEntityStatePDU.EntityID = CurrentEntityID.toEntityID();
        newEntityStatePDU.EntityType = CurrentEntityType.toEntityType();
        newEntityStatePDU.ForceId = (byte)EntityForceID;
        newEntityStatePDU.Marking = PDUUtil.getStringAsMarking(EntityMarking);
        newEntityStatePDU.Capabilities = EntityCapabilities;
        newEntityStatePDU.EntityAppearance = EntityAppearance;

        newEntityStatePDU.DeadReckoningParameters.DeadReckoningAlgorithm = (byte)DeadReckoningAlgorithm;
        float timeSinceLastCalc = Time.timeSinceLevelLoad - TimeOfLastVelocityCalculation;

        if (disGameManagerScript)
        {
            newEntityStatePDU.ExerciseID = Convert.ToByte(disGameManagerScript.ExerciseID);
        }
        else
        {
            newEntityStatePDU.ExerciseID = Convert.ToByte(FindObjectOfType<DISGameManager>()?.ExerciseID);
        }

        // TODO: Implement Unity to/from geospatial conversions
        newEntityStatePDU.EntityLocation = new Vector3Double
        {
            X = -2187316.72454938,
            Y = -4558561.94075072,
            Z = 3882725.39901095
        };
        newEntityStatePDU.EntityOrientation = new Orientation
        {
            Psi = -2.90558f,
            Theta = 0.500781f,
            Phi = 2.2565f
        };

        //Set all geospatial values
        //if (IsValid(GeoReferencingSystem))
        //{
        //    //Calculate the position of the entity in ECEF
        //    Vector3Double ecefLocation;
        //    Conversions.CalculateEcefXYZFromUnrealLocation(transform.position, GeoReferencingSystem, ecefLocation);

        //    newEntityStatePDU.EntityLocation = new Vector3Double
        //    {
        //        X = ecefLocation.X,
        //        Y = ecefLocation.Y,
        //        Z = ecefLocation.Z
        //    };

        //    //Calculate the orientation of the entity in Psi, Theta, Phi
        //    Vector3Double latLonHeightMeters;
        //    Vector3Double headingPitchRollDegrees;
        //    Conversions.CalculateLatLonHeightFromUnrealLocation(transform.position, GeoReferencingSystem, latLonHeightMeters);
        //    Conversions.GetHeadingPitchRollFromUnrealRotation(transform.eulerAngles, transform.position, GeoReferencingSystem, headingPitchRollDegrees);
        //    Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees.X, headingPitchRollDegrees.Y, headingPitchRollDegrees.Z, latLonHeightMeters.X, latLonHeightMeters.Y, out double psi, out double theta, out double phi);

        //    newEntityStatePDU.EntityOrientation = new Orientation
        //    {
        //        Psi = (float)psi,
        //        Theta = (float)theta,
        //        Phi = (float)phi
        //    };
        //}
        //else
        //{
        //    Debug.LogWarning("Invalid GeoReference. Please make sure one is in the world."));
        //}

        //Set all Dead Reckoning Parameters
        if (timeSinceLastCalc > 0)
        {
            //Calculate the velocities and acceleration of the entity in m/s
            Vector3 curLoc = transform.position;
            //Multiply by 100 to convert velocity to m/s
            Vector3 curUnityLinearVelocity = (curLoc - PreviousUnityLocation) / (timeSinceLastCalc * 100);

            Vector3 curRot = transform.eulerAngles;
            Vector3 rotDiff = DeadReckoningLibrary.CalculateDirectionalRotationDifference(PreviousUnityRotation.y, PreviousUnityRotation.x, PreviousUnityRotation.z, curRot.y, curRot.x, curRot.z);

            Vector3Float angularVelocity = new Vector3Float
            {
                X = glm.Radians(rotDiff.x) / timeSinceLastCalc,
                Y = glm.Radians(rotDiff.y) / timeSinceLastCalc,
                Z = glm.Radians(rotDiff.z) / timeSinceLastCalc
            };
            newEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity = angularVelocity;

            Vector3Float linearAcceleration = new Vector3Float();
            //Apply the appropriate acceleration based on Dead Reckoning algorithm being used
            if ((byte)DeadReckoningAlgorithm < 6)
            {
                //Convert linear velocity vectors to be in ECEF coordinates --- UE origin may not be Earth center and may lie rotated on Earth
                Vector3 curLinearVelocity = Conversions.ConvertUnityVectorToECEFVector(curUnityLinearVelocity, curLoc);
                newEntityStatePDU.EntityLinearVelocity = new Vector3Float
                {
                    X = curLinearVelocity.x,
                    Y = curLinearVelocity.y,
                    Z = curLinearVelocity.z
                };

                Vector3 prevECEFLinearVelocity = Conversions.ConvertUnityVectorToECEFVector(PreviousUnityLinearVelocity, PreviousUnityLocation);
                linearAcceleration = new Vector3Float
                {
                    X = (newEntityStatePDU.EntityLinearVelocity.X - prevECEFLinearVelocity.x) / timeSinceLastCalc,
                    Y = (newEntityStatePDU.EntityLinearVelocity.Y - prevECEFLinearVelocity.y) / timeSinceLastCalc,
                    Z = (newEntityStatePDU.EntityLinearVelocity.Z - prevECEFLinearVelocity.z) / timeSinceLastCalc
                };
            }
            else
            {
                //Convert linear velocity vectors to be in body space --- Use inverse UE rotations to convert vectors into appropriate DIS body space
                Vector3 LinearVelocity = Vector3.Scale(Quaternion.Euler(curRot) * curUnityLinearVelocity, new Vector3(1, 1, -1));
                newEntityStatePDU.EntityLinearVelocity = new Vector3Float
                {
                    X = LinearVelocity.x,
                    Y = LinearVelocity.y,
                    Z = LinearVelocity.z
                };
                Vector3 prevVelBodySpace = Vector3.Scale(Quaternion.Euler(PreviousUnityRotation) * PreviousUnityLinearVelocity, new Vector3(1, 1, -1));

                //Calculate the centripetal acceleration in body space
                dvec3 dvecAngularVelocity = new dvec3(angularVelocity.X, angularVelocity.Y, angularVelocity.Z);
                dmat3 SkewMatrix = Conversions.CreateSkewMatrix(dvecAngularVelocity);
                dvec3 dvecBodyVelocityVector = new dvec3(newEntityStatePDU.EntityLinearVelocity.X, newEntityStatePDU.EntityLinearVelocity.Y, newEntityStatePDU.EntityLinearVelocity.Z);
                dvec3 dvecCentripetalAcceleration = (SkewMatrix * dvecBodyVelocityVector);

                linearAcceleration = new Vector3Float
                {
                    X = ((newEntityStatePDU.EntityLinearVelocity.X - prevVelBodySpace.x) / timeSinceLastCalc) + (float)dvecCentripetalAcceleration.x,
                    Y = ((newEntityStatePDU.EntityLinearVelocity.Y - prevVelBodySpace.y) / timeSinceLastCalc) + (float)dvecCentripetalAcceleration.y,
                    Z = ((newEntityStatePDU.EntityLinearVelocity.Z - prevVelBodySpace.z) / timeSinceLastCalc) + (float)dvecCentripetalAcceleration.z
                };
            }

            newEntityStatePDU.DeadReckoningParameters.EntityLinearAcceleration = linearAcceleration;
        }

        newEntityStatePDU.DeadReckoningParameters.OtherParameters = DeadReckoningLibrary.FormOtherParameters(DeadReckoningAlgorithm, newEntityStatePDU.EntityOrientation, newEntityStatePDU.EntityLocation);

        return newEntityStatePDU;
    }

    bool CheckDeadReckoningThreshold()
    {
        bool outsideThreshold = false;

        if (DeadReckoningLibrary.DeadReckoning(MostRecentEntityStatePDU, DeltaTimeSinceLastPDU, ref MostRecentDeadReckoningPDU))
        {
            //Get the actual position of the entity
            Vector3Double ecefLocation = new Vector3Double();

            // TODO: Implement Unity to/from geospatial conversions
            //Conversions.CalculateEcefXYZFromUnrealLocation(transform.position, GeoReferencingSystem, ecefLocation);

            //Get the position difference along each axis. Values should be in ECEF.
            bool xPosOutsideThreshold = Math.Abs(ecefLocation.X - MostRecentDeadReckoningPDU.EntityLocation.X) > DeadReckoningPositionThresholdMeters;
            bool yPosOutsideThreshold = Math.Abs(ecefLocation.Y - MostRecentDeadReckoningPDU.EntityLocation.Y) > DeadReckoningPositionThresholdMeters;
            bool zPosOutsideThreshold = Math.Abs(ecefLocation.Z - MostRecentDeadReckoningPDU.EntityLocation.Z) > DeadReckoningPositionThresholdMeters;

            //Check if the position difference is beyond the position threshold in any axis
            if (xPosOutsideThreshold || yPosOutsideThreshold || zPosOutsideThreshold || CheckOrientationQuaternionThreshold())
            {
                outsideThreshold = true;
            }
        }

        return outsideThreshold;
    }

    bool CheckOrientationQuaternionThreshold()
    {
        bool outsideThreshold = false;

        dvec3 AngularVelocityVector = new dvec3(PreviousEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity.X,
            PreviousEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity.Y, PreviousEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity.Z);

        //Get the entity orientation quaternion
        Quaternion entityOrientationQuaternion = DeadReckoningLibrary.GetEntityOrientationQuaternion(PreviousEntityStatePDU.EntityOrientation.Psi, PreviousEntityStatePDU.EntityOrientation.Theta, PreviousEntityStatePDU.EntityOrientation.Phi);
        //Get the entity dead reckoning quaternion
        Quaternion deadReckoningQuaternion = DeadReckoningLibrary.CreateDeadReckoningQuaternion(AngularVelocityVector, DeltaTimeSinceLastPDU);
        //Calculate the new orientation quaternion
        Quaternion DR_OrientationQuaternion = entityOrientationQuaternion * deadReckoningQuaternion;

        Quaternion actualOrientationQuaternion = new Quaternion();

        // TODO: Implement Unity to/from geospatial conversions
        //if (IsValid(GeoReferencingSystem))
        //{
        //    //Calculate the orientation of the entity in Psi, Theta, Phi
        //    FLatLonHeightFloat latLonHeightMeters;
        //    FHeadingPitchRoll headingPitchRollDegrees;
        //    FPsiThetaPhi psiThetaPhiRadians;

        //    Conversions.CalculateLatLonHeightFromUnrealLocation(transform.position, GeoReferencingSystem, latLonHeightMeters);
        //    Conversions.GetHeadingPitchRollFromUnrealRotation(transform.eulerAngles, transform.position, GeoReferencingSystem, headingPitchRollDegrees);
        //    Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees, latLonHeightMeters.Latitude, latLonHeightMeters.Longitude, psiThetaPhiRadians);
        //    // Get the entity's current orientation quaternion
        //    actualOrientationQuaternion = DeadReckoningLibrary.GetEntityOrientationQuaternion(psiThetaPhiRadians.Psi, psiThetaPhiRadians.Theta, psiThetaPhiRadians.Phi);
        //}
        //else
        //{
        //    Debug.LogWarning("Invalid GeoReference. Please make sure one is in the world."));
        //    return false;
        //}

        float quaternionDotProduct = Quaternion.Dot(actualOrientationQuaternion, DR_OrientationQuaternion);

        double OrientationQuaternionThresholdEpsilon = 1 - Math.Cos(glm.Radians(DeadReckoningOrientationThresholdDegrees / 2));

        //Check if outside of threshold -- 1 is chosen so that left hand side will be 0 if rotations do not differ
        outsideThreshold = (1 - quaternionDotProduct) > OrientationQuaternionThresholdEpsilon;

        return outsideThreshold;
    }

    bool CheckOrientationMatrixThreshold()
    {
        bool outsideThreshold = false;

        dvec3 AngularVelocityVector = new dvec3(PreviousEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity.X,
            PreviousEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity.Y, PreviousEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity.Z);

        // Get the entity's current orientation matrix
        dmat3 OrientationMatrix = DeadReckoningLibrary.GetEntityOrientationMatrix(PreviousEntityStatePDU.EntityOrientation);
        // Get the change in rotation for this time step
        dmat3 DeadReckoningMatrix = DeadReckoningLibrary.CreateDeadReckoningMatrix(AngularVelocityVector, DeltaTimeSinceLastPDU);
        // Calculate the new orientation matrix
        dmat3 DR_OrientationMatrix = DeadReckoningMatrix * OrientationMatrix;

        dmat3 ActualOrientationMatrix = new dmat3();

        // TODO: Implement Unity to/from geospatial conversions
        //if (IsValid(GeoReferencingSystem))
        //{
        //    //Calculate the orientation of the entity in Psi, Theta, Phi
        //    FLatLonHeightFloat latLonHeightMeters;
        //    FHeadingPitchRoll headingPitchRollDegrees;
        //    FPsiThetaPhi psiThetaPhiRadians;

        //    Conversions.CalculateLatLonHeightFromUnrealLocation(transform.position, GeoReferencingSystem, latLonHeightMeters);
        //    Conversions.GetHeadingPitchRollFromUnrealRotation(transform.eulerAngles, transform.position, GeoReferencingSystem, headingPitchRollDegrees);
        //    Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees, latLonHeightMeters.Latitude, latLonHeightMeters.Longitude, psiThetaPhiRadians);
        //    // Get the entity's current orientation matrix
        //    ActualOrientationMatrix = DeadReckoningLibrary.GetEntityOrientationMatrix(psiThetaPhiRadians.Psi, psiThetaPhiRadians.Theta, psiThetaPhiRadians.Phi);
        //}
        //else
        //{
        //    Debug.LogWarning("Invalid GeoReference. Please make sure one is in the world."));
        //    return false;
        //}

        //Calculate the rotational difference matrix and its trace
        dmat3 rotDiffMatrix = DR_OrientationMatrix.Transposed * ActualOrientationMatrix;
        float rotDiffTrace = (float)(rotDiffMatrix.m00 + rotDiffMatrix.m11 + rotDiffMatrix.m22);

        double OrientationMatrixThresholdDelta = 2 - 2 * Math.Cos(glm.Radians(DeadReckoningOrientationThresholdDegrees));

        //Check if outside of threshold -- 3 is chosen so that left hand side will be 0 if rotations do not differ (that is if the rotDiffMatrix was an identity matrix)
        outsideThreshold = (3 - rotDiffTrace) > OrientationMatrixThresholdDelta;

        return outsideThreshold;
    }

    public virtual bool SendEntityStatePDU()
    {
        bool sentUpdate = false;

        //Verify a new Entity State or Entity State Update PDU should be sent
        if ((EntityStatePDUSendingMode == EEntityStateSendingMode.EntityStatePDU || EntityStatePDUSendingMode == EEntityStateSendingMode.EntityStateUpdatePDU) && (DeltaTimeSinceLastPDU > DISHeartbeatSeconds || CheckDeadReckoningThreshold()))
        {
            MostRecentEntityStatePDU = FormEntityStatePDU();
            MostRecentDeadReckoningPDU = MostRecentEntityStatePDU;
            PreviousEntityStatePDU = MostRecentEntityStatePDU;

            EmitAppropriatePDU(MostRecentEntityStatePDU);

            sentUpdate = true;
            DeltaTimeSinceLastPDU = 0;
        }

        return sentUpdate;
    }

    bool EmitAppropriatePDU(EntityStatePdu pduToSend)
    {
        bool successful = false;
        //Send out the appropriate PDU
        if (pduSenderScript != null && EntityStatePDUSendingMode == EEntityStateSendingMode.EntityStatePDU)
        {
            pduSenderScript.SendPdu(pduToSend);
        }
        else if (pduSenderScript != null && EntityStatePDUSendingMode == EEntityStateSendingMode.EntityStateUpdatePDU)
        {
            //Convert PDUToSend to an Entity State Update PDU
            pduSenderScript.SendPdu(pduToSend);
        }

        return successful;
    }
}
