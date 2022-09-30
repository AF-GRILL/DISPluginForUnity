using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenDis.Dis1998;
using System;
using GlmSharp;

public class DISSendComponent : MonoBehaviour
{
    /// <summary>
    /// The DIS Game Manager Game Object that exists in the world.
    /// </summary>
    public GameObject DISGameManager;

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
    /// The rate at which the timer that calculates the current linear velocity, linear acceleration, and angular acceleration of the entity gets executed.
	/// The values calculated by this timer get utilized when forming an Entity State PDU.
    /// </summary>
    [Min(0.01f)]
    [Tooltip("The rate at which the timer that calculates the current linear velocity, linear acceleration, and angular acceleration of the entity gets executed. The values calculated by this timer get utilized when forming an Entity State PDU.")]
    public float EntityStateCalculationRate = 0.1f;
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

    private float TimeOfLastParametersCalculation;
    private Vector3 LastCalculatedUnityLocation;
    private Quaternion LastCalculatedUnityRotation;
    private Vector3 LastCalculatedUnityLinearVelocity;
    private Vector3 LastCalculatedECEFLinearVelocity;
    private Vector3 LastCalculatedECEFLinearAcceleration;
    private Vector3 LastCalculatedBodyLinearVelocity;
    private Vector3 LastCalculatedBodyLinearAcceleration;
    private Vector3 LastCalculatedAngularVelocity;

    private PDUSender pduSenderScript;
    private DISGameManager disGameManagerScript;
    private GeoreferenceSystem georeferenceScript;

    void Start()
    {
        //Setup Previous/Current Location and Rotation variables.
        LastCalculatedUnityLocation = transform.position;
        LastCalculatedUnityRotation = transform.rotation;

        TimeOfLastParametersCalculation = Time.realtimeSinceStartup;

        //Form Entity State PDU packets
        MostRecentEntityStatePDU = FormEntityStatePDU();
        MostRecentDeadReckoningPDU = MostRecentEntityStatePDU;
        PreviousEntityStatePDU = MostRecentEntityStatePDU;

        //If the DIS Game Manager was not set, attempt to find it
        if (DISGameManager == null)
        {
            DISGameManager = FindObjectOfType<DISGameManager>()?.gameObject;
        }
        //Check if it is set and initialize scripts from it
        if (DISGameManager)
        {
            pduSenderScript = DISGameManager.GetComponent<PDUSender>();
            disGameManagerScript = DISGameManager.GetComponent<DISGameManager>();
            georeferenceScript = DISGameManager.GetComponent<GeoreferenceSystem>();

            //Begin play with Entity State PDU
            if (pduSenderScript == null)
            {
                Debug.LogError("DISGameManager is missing a PDU Sender script. Please make sure it has one attached.");
            }
            else if (EntityStatePDUSendingMode != EEntityStateSendingMode.None)
            {
                pduSenderScript = DISGameManager.GetComponent<PDUSender>();
                pduSenderScript.SendPdu(MostRecentEntityStatePDU);
            }
        }
        else
        {
            Debug.LogError("Invalid DISGameManager. Please make sure one is in the world.");
        }

        InvokeRepeating("UpdateEntityStateCalculations", EntityStateCalculationRate, EntityStateCalculationRate);
    }

    void UpdateEntityStateCalculations()
    {
        float deltaTime = Time.realtimeSinceStartup - TimeOfLastParametersCalculation;

        //Update previous velocity, rotation, and location regardless of if an Entity State PDU was sent out.		
        LastCalculatedAngularVelocity = CalculateAngularVelocity();

        CalculateECEFLinearVelocityAndAcceleration(out LastCalculatedECEFLinearVelocity, out LastCalculatedECEFLinearAcceleration);
        CalculateBodyLinearVelocityAndAcceleration(LastCalculatedAngularVelocity, out LastCalculatedBodyLinearVelocity, out LastCalculatedBodyLinearAcceleration);

        if (deltaTime > 0)
        {
            //Divide location offset by 100 to convert to meters
            LastCalculatedUnityLinearVelocity = (transform.position - LastCalculatedUnityLocation) / (deltaTime * 100);
        }

        LastCalculatedUnityLocation = transform.position;
        LastCalculatedUnityRotation = transform.rotation;

        TimeOfLastParametersCalculation = Time.realtimeSinceStartup;
    }

    // Called every frame
    void Update()
    {
        DeltaTimeSinceLastPDU += Time.deltaTime;

        if (EntityStatePDUSendingMode != EEntityStateSendingMode.None)
        {
            SendEntityStatePDU();
        }
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

    public void SetEntityCapabilities(int NewEntityCapabilities)
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

    public void SetEntityAppearance(int NewEntityAppearance)
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

    public void SetDeadReckoningAlgorithm(EDeadReckoningAlgorithm NewDeadReckoningAlgorithm)
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

    public EntityStatePdu FormEntityStatePDU()
    {
        EntityStatePdu newEntityStatePDU = new EntityStatePdu();

        newEntityStatePDU.EntityID = CurrentEntityID.toEntityID();
        newEntityStatePDU.EntityType = CurrentEntityType.toEntityType();
        newEntityStatePDU.ForceId = (byte)EntityForceID;
        newEntityStatePDU.Marking = PDUUtil.getStringAsMarking(EntityMarking);
        newEntityStatePDU.Capabilities = EntityCapabilities;
        newEntityStatePDU.EntityAppearance = EntityAppearance;

        newEntityStatePDU.DeadReckoningParameters.DeadReckoningAlgorithm = (byte)DeadReckoningAlgorithm;

        if (disGameManagerScript)
        {
            newEntityStatePDU.ExerciseID = Convert.ToByte(disGameManagerScript.ExerciseID);
        }
        else
        {
            newEntityStatePDU.ExerciseID = Convert.ToByte(FindObjectOfType<DISGameManager>()?.ExerciseID);
        }

        //Set all geospatial values
        if (georeferenceScript)
        {
            //Calculate the position of the entity in ECEF
            newEntityStatePDU.EntityLocation = georeferenceScript.UnityToECEF(transform.position);

            // TODO: Implement Unity to/from geospatial conversions
            //Calculate the orientation of the entity in Psi, Theta, Phi
            //Vector3Double latLonHeightMeters;
            //Vector3Double headingPitchRollDegrees;
            //Conversions.CalculateLatLonHeightFromUnrealLocation(transform.position, GeoReferencingSystem, latLonHeightMeters);
            //Conversions.GetHeadingPitchRollFromUnrealRotation(transform.eulerAngles, transform.position, GeoReferencingSystem, headingPitchRollDegrees);
            //Conversions.CalculatePsiThetaPhiRadiansFromHeadingPitchRollDegreesAtLatLon(headingPitchRollDegrees.X, headingPitchRollDegrees.Y, headingPitchRollDegrees.Z, latLonHeightMeters.X, latLonHeightMeters.Y, out double psi, out double theta, out double phi);

            //newEntityStatePDU.EntityOrientation = new Orientation
            //{
            //    Psi = (float)psi,
            //    Theta = (float)theta,
            //    Phi = (float)phi
            //};
        }
        else
        {
            Debug.LogWarning("Invalid GeoReference Script. Please make sure one is attached to the DIS Game Manager.");
        }

        //Set all Dead Reckoning Parameters
        Vector3 angularVelocity = CalculateAngularVelocity();
        newEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity = new Vector3Float
        {
            X = angularVelocity.x,
            Y = angularVelocity.y,
            Z = angularVelocity.z
        };

        //Verify close to enough time has passed to need values calculated again
        float deltaTime = Time.realtimeSinceStartup - TimeOfLastParametersCalculation;
        if ((deltaTime - EntityStateCalculationRate) > 0)
        {
            UpdateEntityStateCalculations();
        }

        //Set the angular velocity of the entity
        newEntityStatePDU.DeadReckoningParameters.EntityAngularVelocity = new Vector3Float
        { 
            X = LastCalculatedAngularVelocity.x,
            Y = LastCalculatedAngularVelocity.y,
            Z = LastCalculatedAngularVelocity.z,
        };

        //Apply the appropriate linear velocity and acceleration based on Dead Reckoning algorithm being used
        if ((byte)DeadReckoningAlgorithm == 1 || (byte)DeadReckoningAlgorithm == 2 || (byte)DeadReckoningAlgorithm == 3 || (byte)DeadReckoningAlgorithm == 4 || (byte)DeadReckoningAlgorithm == 5)
        {
            newEntityStatePDU.EntityLinearVelocity = new Vector3Float
            {
                X = LastCalculatedECEFLinearVelocity.x,
                Y = LastCalculatedECEFLinearVelocity.y,
                Z = LastCalculatedECEFLinearVelocity.z
            };
            newEntityStatePDU.DeadReckoningParameters.EntityLinearAcceleration = new Vector3Float
            {
                X = LastCalculatedECEFLinearAcceleration.x,
                Y = LastCalculatedECEFLinearAcceleration.y,
                Z = LastCalculatedECEFLinearAcceleration.z
            };
        }
        else if ((byte)DeadReckoningAlgorithm == 6 || (byte)DeadReckoningAlgorithm == 7 || (byte)DeadReckoningAlgorithm == 8 || (byte)DeadReckoningAlgorithm == 9)
        {
            newEntityStatePDU.EntityLinearVelocity = new Vector3Float
            {
                X = LastCalculatedBodyLinearVelocity.x,
                Y = LastCalculatedBodyLinearVelocity.y,
                Z = LastCalculatedBodyLinearVelocity.z
            };
            newEntityStatePDU.DeadReckoningParameters.EntityLinearAcceleration = new Vector3Float
            {
                X = LastCalculatedBodyLinearAcceleration.x,
                Y = LastCalculatedBodyLinearAcceleration.y,
                Z = LastCalculatedBodyLinearAcceleration.z
            };
        }

        newEntityStatePDU.DeadReckoningParameters.OtherParameters = DeadReckoningLibrary.FormOtherParameters(DeadReckoningAlgorithm, newEntityStatePDU.EntityOrientation, newEntityStatePDU.EntityLocation);

        return newEntityStatePDU;
    }

    public bool CheckDeadReckoningThreshold()
    {
        bool outsideThreshold = false;

        if (DeadReckoningLibrary.DeadReckoning(MostRecentEntityStatePDU, DeltaTimeSinceLastPDU, ref MostRecentDeadReckoningPDU))
        {
            //Get the actual position of the entity
            Vector3Double ecefLocation = georeferenceScript.UnityToECEF(transform.position);

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

    public bool CheckOrientationQuaternionThreshold()
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

    public bool CheckOrientationMatrixThreshold()
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

    public bool SendEntityStatePDU()
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

    public void CalculateECEFLinearVelocityAndAcceleration(out Vector3 ECEFLinearVelocity, out Vector3 ECEFLinearAcceleration)
    {
        float timeSinceLastCalc = Time.realtimeSinceStartup - TimeOfLastParametersCalculation;

        //If delta time is greater than zero, calculate new values. Otherwise use previous calculations
        if (timeSinceLastCalc > 0)
        {
            Vector3 curLoc = transform.position;
            //Divide location offset by 100 to convert to meters
            Vector3 curUnrealLinearVelocity = (curLoc - LastCalculatedUnityLocation) / (timeSinceLastCalc * 100);

            //Convert linear velocity vectors to be in ECEF coordinates --- UE origin may not be Earth center and may lie rotated on Earth
            Vector3Double ecefLinearVelocityDouble = Conversions.ConvertUnityVectorToECEFVector(curUnrealLinearVelocity, georeferenceScript, curLoc);
            ECEFLinearVelocity = new Vector3((float)ecefLinearVelocityDouble.X, (float)ecefLinearVelocityDouble.Y, (float)ecefLinearVelocityDouble.Z);
            Vector3Double prevECEFLinearVelocity = Conversions.ConvertUnityVectorToECEFVector(LastCalculatedUnityLinearVelocity, georeferenceScript, LastCalculatedUnityLocation);

            ECEFLinearAcceleration = new Vector3
            {
                x = (float)(ECEFLinearVelocity.x - prevECEFLinearVelocity.X) / timeSinceLastCalc,
                y = (float)(ECEFLinearVelocity.y - prevECEFLinearVelocity.Y) / timeSinceLastCalc,
                z = (float)(ECEFLinearVelocity.z - prevECEFLinearVelocity.Z) / timeSinceLastCalc
            };
        }
        else
        {
            //Convert linear velocity vectors to be in ECEF coordinates --- UE origin may not be Earth center and may lie rotated on Earth
            ECEFLinearVelocity = LastCalculatedECEFLinearVelocity;
            ECEFLinearAcceleration = LastCalculatedECEFLinearAcceleration;
        }
    }

    public void CalculateBodyLinearVelocityAndAcceleration(Vector3 AngularVelocity, out Vector3 BodyLinearVelocity, out Vector3 BodyLinearAcceleration)
    {
        float timeSinceLastCalc = Time.realtimeSinceStartup - TimeOfLastParametersCalculation;

        //If delta time greater than zero, calculate new values. Otherwise use previous calculations
        if (timeSinceLastCalc > 0)
        {
            Vector3 curLoc = transform.position;
            Vector3 curUnityLinearVelocity = (curLoc - LastCalculatedUnityLocation) / (timeSinceLastCalc * 100);

            //Convert linear velocity vectors to be in body space --- Use inverse UE rotations to convert vectors into appropriate DIS body space
            BodyLinearVelocity = Vector3.Scale(transform.rotation * curUnityLinearVelocity, new Vector3(1, 1, -1));
            Vector3 prevVelBodySpace = Vector3.Scale(LastCalculatedUnityRotation * LastCalculatedUnityLinearVelocity, new Vector3(1, 1, -1));
            BodyLinearAcceleration = (BodyLinearVelocity - prevVelBodySpace) / timeSinceLastCalc;

            //Calculate the centripetal acceleration in body space
            dvec3 dvecAngularVelocity = new dvec3(AngularVelocity.x, AngularVelocity.y, AngularVelocity.z);
            dmat3 SkewMatrix = Conversions.CreateSkewMatrix(dvecAngularVelocity);
            dvec3 dvecBodyVelocityVector = new dvec3(BodyLinearVelocity.x, BodyLinearVelocity.y, BodyLinearVelocity.z);
            dvec3 dvecCentripetalAcceleration = (SkewMatrix * dvecBodyVelocityVector);

            BodyLinearAcceleration = new Vector3
            {
                x = BodyLinearAcceleration.x + (float)dvecCentripetalAcceleration.x,
                y = BodyLinearAcceleration.y + (float)dvecCentripetalAcceleration.y,
                z = BodyLinearAcceleration.z + (float)dvecCentripetalAcceleration.z
            };
        }
        else
        {
            //Convert linear velocity vectors to be in body space --- Use inverse UE rotations to convert vectors into appropriate DIS body space
            BodyLinearVelocity = LastCalculatedBodyLinearVelocity;
            BodyLinearAcceleration = LastCalculatedBodyLinearAcceleration;
        }
    }

    public Vector3 CalculateAngularVelocity()
    {
        Vector3 angularVelocity = LastCalculatedAngularVelocity;
        float timeSinceLastCalc = Time.realtimeSinceStartup - TimeOfLastParametersCalculation;

        if (timeSinceLastCalc > 0)
        {
            //Convert the rotators to quaternions
            Quaternion oldQuat = LastCalculatedUnityRotation;
            Quaternion newQuat = transform.rotation;
            oldQuat.Normalize();
            newQuat.Normalize();

            //Get the rotational difference between the quaternions -- Gives back direction of rotation too
            Quaternion rotDiff = Quaternion.Inverse(oldQuat) * newQuat;

            //If negative, flip it
            if (rotDiff.w < 0)
            {
                rotDiff = Quaternion.Inverse(rotDiff);
                rotDiff.w *= -1;
            }

            Vector3 rotationAxis;
            float rotationAngle;
            rotDiff.ToAngleAxis(out rotationAngle, out rotationAxis);
            rotationAngle = glm.Radians(rotationAngle);
            
            angularVelocity = (rotationAngle * rotationAxis) / timeSinceLastCalc;
            //Swap axes as needed and invert X and Y axis
            angularVelocity = new Vector3(-angularVelocity.z, -angularVelocity.x, angularVelocity.y);
        }

        return angularVelocity;
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
