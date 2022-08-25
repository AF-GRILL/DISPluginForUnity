using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenDis.Dis1998;
using System;
using UnityEngine.Events;
using GlmSharp;

[Serializable] public class DeadReckoningUpdate : UnityEvent<EntityStatePdu> { }
[Serializable] public class EntityStatePDUProcessed : UnityEvent<EntityStatePdu> { }
[Serializable] public class EntityStateUpdatePDUProcessed : UnityEvent<EntityStateUpdatePdu> { }
[Serializable] public class FirePDUProcessed : UnityEvent<FirePdu> { }
[Serializable] public class DetonationPDUProcessed : UnityEvent<DetonationPdu> { }
[Serializable] public class RemoveEntityPDUProcessed : UnityEvent<RemoveEntityPdu> { }
[Serializable] public class StopFreezePDUProcessed : UnityEvent<StopFreezePdu> { }
[Serializable] public class StartResumePDUProcessed : UnityEvent<StartResumePdu> { }

public class DISComponent : MonoBehaviour
{
    /// <summary>
    /// The time to live for the entity. Gets reset every time a new Entity State PDU is received by the sim.
    /// </summary>
    [Header("DIS Info")]
    [Range(0.0f, 3600.0f)]
    [Tooltip("The time to live for the entity. Gets reset every time a new Entity State PDU is received by the sim.")]
    public float DISTimeoutSeconds = 30.0f;
    /// <summary>
    /// The Entity Type of the associated entity. Specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.
    /// </summary>
    [Tooltip("The Entity Type of the associated entity. Specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.")]
    public EntityTypeEditor CurrentEntityType;
    /// <summary>
    /// The Entity ID of the associated entity. Each Entity ID should be unique to an entity in the sim.
    /// </summary>
    [Tooltip("The Entity ID of the associated entity. Each Entity ID should be unique to an entity in the sim.")]
    public EntityIDEditor CurrentEntityID;
    /// <summary>
    /// The Force ID of the associated entity. Specifies the team or side the DIS entity is on.
    /// </summary>
    [Tooltip("The Force ID of the associated entity. Specifies the team or side the DIS entity is on.")]
    public EForceID EntityForceID;
    /// <summary>
    /// The Entity Marking of the associated entity. Designates a friendly name for the DIS entity. Max of 11 characters should be used.If more than 11 are used, it will be truncated.
    /// </summary>
    [Tooltip("The Entity Marking of the associated entity. Designates a friendly name for the DIS entity. Max of 11 characters should be used.If more than 11 are used, it will be truncated.")]
    public string EntityMarking;

    /// <summary>
    /// Determines what all DIS info should be culled. Allows for updates to happen less frequently for entities that aren't currently important. The distance that culling should happen is handled by the DISCullingDistance variable.
    /// </summary>
    [Header("DIS Settings")]
    [Tooltip("Determines what all DIS info should be culled. Allows for updates to happen less frequently for entities that aren't currently important. The distance that culling should happen is handled by the DISCullingDistance variable.")]
    public EDISCullingMode DISCullingMode = EDISCullingMode.None;
    /// <summary>
    /// The distance in Unity units from the camera that culling should begin happening.
    /// </summary>
    [Tooltip("The distance in Unity units from the camera that culling should begin happening.")]
    public float DISCullingDistance = 0.0f;
    /// <summary>
    /// Whether or not dead reckoning should be performed for this entity.
    /// </summary>
    [Tooltip("Whether or not dead reckoning should be performed for this entity.")]
    public bool PerformDeadReckoning = true;
    /// <summary>
    /// Whether or not dead reckoning should be locally smoothed for this entity.
    /// </summary>
    [Tooltip("Whether or not dead reckoning should be locally smoothed for this entity.")]
    public bool PerformDeadReckoningSmoothing = true;
    /// <summary>
    /// Number of seconds to smooth between dead reckoned information and packet information if dead reckoning is enabled.
    /// </summary>
    [Tooltip("Number of seconds to smooth between dead reckoned information and packet information if dead reckoning is enabled")]
    public float DeadReckoningSmoothingPeriodSeconds = 0.5f;
    /// <summary>
    /// Determines how this entity should be ground clamped.
    /// </summary>
    [Tooltip("Determines how this entity should be ground clamped.")]
    public EGroundClampingMode PerformGroundClamping = EGroundClampingMode.GroundClampWithDISOptions;
    /// <summary>
    /// The collision channel to use for ground clamping.
    /// </summary>
    [Tooltip("The collision channel to use for ground clamping.")]
    public LayerMask GroundClampingCollisionLayer = 1;

    /// <summary>
    /// Called after a dead reckoning update is performed by the component. Passes out an Entity State PDU with updated dead reckoning variables as a parameter.
    /// </summary>
    [Space(20)]
    public DeadReckoningUpdate OnDeadReckoningUpdate = new DeadReckoningUpdate();
    /// <summary>
    /// Called after an Entity State PDU is processed by the component. The component updates associated variables prior to broadcasting this event. Passes the Entity State PDU that was received as a parameter.
    /// </summary>
    public EntityStatePDUProcessed OnEntityStatePDUProcessed = new EntityStatePDUProcessed();
    /// <summary>
    /// Called after an Entity State Update PDU is processed by the component. The component updates associated variables prior to broadcasting this event. Passes the Entity State Update PDU that was received as a parameter.
    /// </summary>
    public EntityStateUpdatePDUProcessed OnEntityStateUpdatePDUProcessed = new EntityStateUpdatePDUProcessed();
    /// <summary>
    /// Called after a Fire PDU is processed by the component. Passes the Fire PDU that was received as a parameter.
    /// </summary>
    public FirePDUProcessed OnFirePDUProcessed = new FirePDUProcessed();
    /// <summary>
    /// Called after a Detonation PDU is processed by the component. Passes the Detonation PDU that was received as a parameter.
    /// </summary>
    public DetonationPDUProcessed OnDetonationPDUProcessed = new DetonationPDUProcessed();
    /// <summary>
    /// Called after a Remove Entity PDU is processed by the component. Passes the Remove Entity PDU that was received as a parameter.
    /// </summary>
    public RemoveEntityPDUProcessed OnRemoveEntityPDUProcessed = new RemoveEntityPDUProcessed();
    /// <summary>
    /// Called after a Stop/Freeze PDU is processed by the component. Passes the Stop / Freeze PDU that was received as a parameter.
    /// </summary>
    public StopFreezePDUProcessed OnStopFreezePDUProcessed = new StopFreezePDUProcessed();
    /// <summary>
    /// Called after a Start/Resume PDU is processed by the component. Passes the Start / Resume PDU that was received as a parameter.
    /// </summary>
    public StartResumePDUProcessed OnStartResumePDUProcessed = new StartResumePDUProcessed();

    /// <summary>
    /// The timestamp that the most recent Entity State PDU was received at by the DISComponent.
    /// </summary>
    [HideInInspector]
    public DateTime LatestEntityStatePDUTimestamp;
    /// <summary>
    /// Whether or not the associated entity was spawned by the network or not.
    /// </summary>
    [HideInInspector]
    public bool SpawnedFromNetwork = false;
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

    
    public DISGameManager disGameManagerScript;

    private int NumberEntityStatePDUsReceived = 0;
    private Coroutine DISHeartbeatCoroutine;
    private float DeltaTimeSinceLastPDU = 0.0f;
    private EntityStatePdu PreviousDeadReckonedPDU = new EntityStatePdu();
    /// <summary>
    /// The difference in ECEF Location from the previous PDU to the current PDU.
    /// </summary>
    private Vector3Double EntityECEFLocationDifference = new Vector3Double();
    /// <summary>
    /// The difference in Orientation from the previous PDU to the current PDU.
    /// </summary>
    private Vector3 EntityRotationDifference;

    void HandleDISHeartbeat()
    {
        //Stop the coroutine if one is running
        if (DISHeartbeatCoroutine != null)
        {
            StopCoroutine(DISHeartbeatCoroutine);
        }
        //Start the coroutine
        DISHeartbeatCoroutine = StartCoroutine(DelayedDestroy(DISTimeoutSeconds));
    }

    IEnumerator DelayedDestroy(float timeToDelay)
    {
        //Delay for amount of time and then destroy
        yield return new WaitForSeconds(timeToDelay);
        Destroy(gameObject);
    }

    // Update is called once per frame
    void Update()
    {
        DoDeadReckoning(Time.deltaTime);
    }

    /// <summary>
    /// Handles processing for Entity State PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewEntityStatePDU">The Entity State PDU to process.</param>
    public void HandleEntityStatePDU(EntityStatePdu NewEntityStatePDU)
    {
        //Check if the entity has been deactivated -- Entity is deactivated if the 23rd bit of the Entity Appearance value is set
        if ((NewEntityStatePDU.EntityAppearance & (1 << 23)) != 0)
        {
            Debug.Log(NewEntityStatePDU.EntityID.ToString() + " Entity Appearance is set to deactivated, deleting entity...");
            Destroy(gameObject);
            return;
        }

        UpdateCommonEntityStateInfo(NewEntityStatePDU);

        CurrentEntityType.fromEntityType(NewEntityStatePDU.EntityType);
        EntityForceID = (EForceID)NewEntityStatePDU.ForceId;
        EntityMarking = PDUUtil.getMarkingAsString(NewEntityStatePDU);

        OnEntityStatePDUProcessed.Invoke(NewEntityStatePDU);

        if (!PerformDeadReckoning)
        {
            GroundClamping();
        }
    }

    /// <summary>
    /// Handles processing for Entity State Update PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewEntityStateUpdatePDU">The Entity State Update PDU to process.</param>
    public void HandleEntityStateUpdatePDU(EntityStateUpdatePdu NewEntityStateUpdatePDU)
    {
        //Check if the entity has been deactivated -- Entity is deactivated if the 23rd bit of the Entity Appearance value is set
        if ((NewEntityStateUpdatePDU.EntityAppearance & (1 << 23)) != 0)
        {
            Debug.Log(NewEntityStateUpdatePDU.EntityID.ToString() + " Entity Appearance is set to deactivated, deleting entity...");
            Destroy(gameObject);
            return;
        }

        //Update all similar variables between the Entity State and the Entity State Update
        MostRecentEntityStatePDU.ProtocolVersion = NewEntityStateUpdatePDU.ProtocolVersion;
        MostRecentEntityStatePDU.ExerciseID = NewEntityStateUpdatePDU.ExerciseID;
        MostRecentEntityStatePDU.ProtocolFamily = NewEntityStateUpdatePDU.ProtocolFamily;
        MostRecentEntityStatePDU.Timestamp = NewEntityStateUpdatePDU.Timestamp;
        MostRecentEntityStatePDU.Length = NewEntityStateUpdatePDU.Length;
        MostRecentEntityStatePDU.Padding = NewEntityStateUpdatePDU.Padding;

        MostRecentEntityStatePDU.EntityID = NewEntityStateUpdatePDU.EntityID;
        MostRecentEntityStatePDU.EntityLocation = NewEntityStateUpdatePDU.EntityLocation;
        MostRecentEntityStatePDU.EntityOrientation = NewEntityStateUpdatePDU.EntityOrientation;
        MostRecentEntityStatePDU.EntityLinearVelocity = NewEntityStateUpdatePDU.EntityLinearVelocity;
        MostRecentEntityStatePDU.EntityAppearance = NewEntityStateUpdatePDU.EntityAppearance;

        MostRecentEntityStatePDU.ArticulationParameters.Clear();
        MostRecentEntityStatePDU.ArticulationParameters.AddRange(NewEntityStateUpdatePDU.ArticulationParameters);

        UpdateCommonEntityStateInfo(MostRecentEntityStatePDU);

        OnEntityStateUpdatePDUProcessed.Invoke(NewEntityStateUpdatePDU);

        if (!PerformDeadReckoning)
        {
            GroundClamping();
        }
    }

    /// <summary>
    /// Handles updating common variables that both the Entity State and Entity State Update PDU have
    /// </summary>
    /// <param name="NewEntityStatePDU">The Entity State or Entity State Update PDU to update variables from.</param>
    void UpdateCommonEntityStateInfo(EntityStatePdu NewEntityStatePDU)
    {
        LatestEntityStatePDUTimestamp = DateTime.Now;
        MostRecentEntityStatePDU = NewEntityStatePDU;
        PreviousDeadReckonedPDU = MostRecentDeadReckoningPDU;
        MostRecentDeadReckoningPDU = PDUUtil.DeepCopyEntityStatePDU(MostRecentEntityStatePDU);
        DeltaTimeSinceLastPDU = 0.0f;

        //Get difference in ECEF between most recent dead reckoning location and last known Dead Reckoning location
        EntityECEFLocationDifference.X = MostRecentEntityStatePDU.EntityLocation.X - PreviousDeadReckonedPDU.EntityLocation.X;
        EntityECEFLocationDifference.Y = MostRecentEntityStatePDU.EntityLocation.Y - PreviousDeadReckonedPDU.EntityLocation.Y;
        EntityECEFLocationDifference.Z = MostRecentEntityStatePDU.EntityLocation.Z - PreviousDeadReckonedPDU.EntityLocation.Z;

        EntityRotationDifference.x = MostRecentEntityStatePDU.EntityOrientation.Psi - PreviousDeadReckonedPDU.EntityOrientation.Psi;
        EntityRotationDifference.y = MostRecentEntityStatePDU.EntityOrientation.Theta - PreviousDeadReckonedPDU.EntityOrientation.Theta;
        EntityRotationDifference.z = MostRecentEntityStatePDU.EntityOrientation.Phi - PreviousDeadReckonedPDU.EntityOrientation.Phi;

        CurrentEntityID.fromEntityID(NewEntityStatePDU.EntityID);

        HandleDISHeartbeat();

        NumberEntityStatePDUsReceived++;
    }

    /// <summary>
    /// Handles processing for Fire PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewFirePDU">The Fire PDU to process.</param>
    public void HandleFirePDU(FirePdu NewFirePDU)
    {
        OnFirePDUProcessed.Invoke(NewFirePDU);
    }

    /// <summary>
    /// Handles processing for Detonation PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewDetonationPDU">The Detonation PDU to process.</param>
    public void HandleDetonationPDU(DetonationPdu NewDetonationPDU)
    {
        OnDetonationPDUProcessed.Invoke(NewDetonationPDU);
    }

    /// <summary>
    /// Handles processing for Remove Entity PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewRemoveEntityPDU">The Remove Entity PDU to process.</param>
    public void HandleRemoveEntityPDU(RemoveEntityPdu NewRemoveEntityPDU)
    {
        OnRemoveEntityPDUProcessed.Invoke(NewRemoveEntityPDU);
    }

    /// <summary>
    /// Handles processing for Stop Freeze PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewStopFreezePDU">The Stop Freeze PDU to process.</param>
    public void HandleStopFreezePDU(StopFreezePdu NewStopFreezePDU)
    {
        OnStopFreezePDUProcessed.Invoke(NewStopFreezePDU);
    }

    /// <summary>
    /// Handles processing for Start Resume PDUs on the DIS Component.
    /// </summary>
    /// <param name="NewStartResumePDU">The Start Resume PDU to process.</param>
    public void HandleStartResumePDU(StartResumePdu NewStartResumePDU)
    {
        OnStartResumePDUProcessed.Invoke(NewStartResumePDU);
    }

    void DoDeadReckoning(float DeltaTime)
    {
        DeltaTimeSinceLastPDU += DeltaTime;

        if (PerformDeadReckoning && SpawnedFromNetwork)
        {
            //Check if Dead Reckoning updates should be culled or not.
            if (DISCullingMode == EDISCullingMode.CullDeadReckoning || DISCullingMode == EDISCullingMode.CullAll)
            {
                //If so, get the player camera and cull beyond specified distance.
                Camera mainCamera = Camera.main;
                if (mainCamera)
                {
                    Vector3 userCameraLocation = mainCamera.transform.position;
                    float distanceToUser = Vector3.Distance(transform.position, userCameraLocation);

                    if (distanceToUser > DISCullingDistance)
                    {
                        //In case users are relying on Dead Reckoning for their entity movement, just send them the most recent Dead Reckoned PDU again
                        OnDeadReckoningUpdate.Invoke(MostRecentDeadReckoningPDU);
                        return;
                    }
                }
            }

            if (DeadReckoning(MostRecentEntityStatePDU, DeltaTimeSinceLastPDU, ref MostRecentDeadReckoningPDU))
            {
                //If more than one PDU has been received and we're still in the smoothing period, then smooth
                if (PerformDeadReckoningSmoothing && NumberEntityStatePDUsReceived > 1 && DeltaTimeSinceLastPDU <= DeadReckoningSmoothingPeriodSeconds)
                {
                    //Only update temp Dead Reckoning PDU. We don't want this smoothing to change the actual Dead Reckoning calculations
                    MostRecentDeadReckoningPDU = SmoothDeadReckoning(MostRecentDeadReckoningPDU);
                }

                OnDeadReckoningUpdate.Invoke(MostRecentDeadReckoningPDU);
            }

            //Perform ground clamping last
            GroundClamping();
        }
    }

    bool DeadReckoning(EntityStatePdu EntityPduToDeadReckon, float DeltaTime, ref EntityStatePdu DeadReckonedPdu)
    {
        if (!PerformDeadReckoning || !SpawnedFromNetwork)
        {
            PerformDeadReckoning = false;
            return false;
        }

        bool supported = true;

        switch (EntityPduToDeadReckon.DeadReckoningParameters.DeadReckoningAlgorithm)
        {
            case 1: // Static
                {
                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
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
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
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

                    if (GetLocalQuaternionAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Quaternion LocalQuaternion))
                    {
                        DeadReckonedPdu.EntityOrientation = CalculateOrientationFromQuaternion(LocalQuaternion, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity, DeltaTime);
                    }
                    else
                    {
                        dvec3 EntityAngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

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

                    if (GetLocalQuaternionAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Quaternion LocalQuaternion))
                    {
                        DeadReckonedPdu.EntityOrientation = CalculateOrientationFromQuaternion(LocalQuaternion, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity, DeltaTime);
                    }
                    else
                    {
                        dvec3 EntityAngularVelocity = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y,
                            EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

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
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
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
                    dvec3 AngularAcceleration = new dvec3(0, 0, 0);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularAcceleration, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
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
                    dvec3 AngularAcceleration = new dvec3(0, 0, 0);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularAcceleration, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
                    }
                    else
                    {
                        DeadReckonedPdu.EntityOrientation = CalculateDeadReckonedOrientation(EntityPduToDeadReckon.EntityOrientation, AngularAcceleration, DeltaTime);
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
                    dvec3 AngularAcceleration = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularAcceleration, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
                    }
                    else
                    {
                        DeadReckonedPdu.EntityOrientation = CalculateDeadReckonedOrientation(EntityPduToDeadReckon.EntityOrientation, AngularAcceleration, DeltaTime);
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
                    dvec3 AngularAcceleration = new dvec3(EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.X,
                        EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Y, EntityPduToDeadReckon.DeadReckoningParameters.EntityAngularVelocity.Z);

                    dvec3 deadReckonLocation = CalculateBodyDeadReckonedPosition(PositionVector, VelocityVector, AccelerationVector, AngularAcceleration, EntityPduToDeadReckon.EntityOrientation, DeltaTime);

                    DeadReckonedPdu.EntityLocation = new Vector3Double
                    {
                        X = deadReckonLocation.x,
                        Y = deadReckonLocation.y,
                        Z = deadReckonLocation.z
                    };

                    if (GetLocalEulerAngles(EntityPduToDeadReckon.DeadReckoningParameters.OtherParameters, out Orientation LocalRotation))
                    {
                        DeadReckonedPdu.EntityOrientation = LocalRotation;
                    }

                    break;
                }
            default:
                supported = false;
                break;
        }

        return supported;
    }

    /// <summary>
    /// Converts a Quaternion into an OpenDIS Orientation (Psi, Theta, Phi).
    /// </summary>
    /// <param name="localQuaternion">The Quaternion to convert into an Orientation (Psi, Theta, Phi).</param>
    /// <param name="entityAngularVelocity">The angular velocity vector in body coordinates</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The OpenDIS Orientation.</returns>
    private Orientation CalculateOrientationFromQuaternion(Quaternion localQuaternion, Vector3Float entityAngularVelocity, float deltaTime)
    {
        double entityAngularVelocityMagnitude = entityAngularVelocity.CalculateLength();
        double beta = entityAngularVelocityMagnitude * deltaTime;
        Vector3Double unitVector = new Vector3Double
        {
            X = entityAngularVelocity.X / entityAngularVelocityMagnitude,
            Y = entityAngularVelocity.Y / entityAngularVelocityMagnitude,
            Z = entityAngularVelocity.Z / entityAngularVelocityMagnitude
        };
        Quaternion deadReckoningQuat = localQuaternion * new Quaternion((float)Math.Cos(beta / 2), (float)(unitVector.X * Math.Sin(beta / 2)), (float)(unitVector.Y * Math.Sin(beta / 2)), (float)(unitVector.Z * Math.Sin(beta / 2)));
        return new Orientation
        {
            Psi = (float)Math.Atan2(2 * ((deadReckoningQuat.x * deadReckoningQuat.y) + (deadReckoningQuat.w * deadReckoningQuat.z)), Math.Pow(deadReckoningQuat.w, 2) + Math.Pow(deadReckoningQuat.x, 2) - Math.Pow(deadReckoningQuat.y, 2) - Math.Pow(deadReckoningQuat.z, 2)),
            Theta = (float)Math.Asin(-2 * ((deadReckoningQuat.x * deadReckoningQuat.z) - (deadReckoningQuat.w * deadReckoningQuat.y))),
            Phi = (float)Math.Atan2(2 * ((deadReckoningQuat.y * deadReckoningQuat.z) + (deadReckoningQuat.w * deadReckoningQuat.x)), Math.Pow(deadReckoningQuat.w, 2) - Math.Pow(deadReckoningQuat.x, 2) - Math.Pow(deadReckoningQuat.y, 2) + Math.Pow(deadReckoningQuat.z, 2))
        };
    }

    /// <summary>
    /// Calculates the new position vector using the given velocity, acceleration, and time increment given in body coordinates
    /// </summary>
    /// <param name="entityLocation">The initial position of the entity in body coordinates</param>
    /// <param name="entityLinearVelocity">The initial velocity in body coordinates</param>
    /// <param name="entityLinearAcceleration">The initial linear acceleration in body coordinates</param>
    /// <param name="entityAngularAcceleration">The initial angular acceleration in body coordinates</param>
    /// <param name="entityOrientation">The orientation of the entity in radians</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The position vector in body coordinates.</returns>
    private dvec3 CalculateBodyDeadReckonedPosition(dvec3 entityLocation, dvec3 entityLinearVelocity, dvec3 entityLinearAcceleration, dvec3 entityAngularAcceleration, Orientation entityOrientation, float deltaTime)
    {
        dmat3 SkewMatrix = Conversions.CreateSkewMatrix(entityAngularAcceleration);
        dvec3 BodyAccelerationVector = entityLinearAcceleration - (SkewMatrix * entityLinearVelocity);

        // Get the entity's current orientation matrix
        dmat3 OrientationMatrix = GetEntityOrientationMatrix(entityOrientation);
        dmat3 InverseInitialOrientationMatrix = OrientationMatrix.Inverse;

        // Calculate R1
        dmat3 OmegaMatrix = new dmat3(entityAngularAcceleration, new dvec3(0), new dvec3(0)) * new dmat3(entityAngularAcceleration, new dvec3(0), new dvec3(0)).Transposed;
        double AccelerationMagnitude = entityAngularAcceleration.Length;
        dmat3 R1 = (((AccelerationMagnitude * deltaTime - Math.Sin(AccelerationMagnitude * deltaTime)) / Math.Pow(AccelerationMagnitude, 3)) * OmegaMatrix) +
            ((Math.Sin(AccelerationMagnitude * deltaTime) / AccelerationMagnitude) * new dmat3(1, 0, 0, 0, 1, 0, 0, 0, 1)) +
            (((1 - Math.Cos(AccelerationMagnitude * deltaTime)) / Math.Pow(AccelerationMagnitude, 2)) * SkewMatrix);

        dmat3 R2 = ((((0.5 * Math.Pow(AccelerationMagnitude, 2) * Math.Pow(deltaTime, 2)) - (Math.Cos(AccelerationMagnitude * deltaTime)) - (AccelerationMagnitude * deltaTime * Math.Sin(AccelerationMagnitude * deltaTime)) + (1)) / Math.Pow(AccelerationMagnitude, 4)) * SkewMatrix * SkewMatrix.Transposed) +
            ((((Math.Cos(AccelerationMagnitude * deltaTime)) + (AccelerationMagnitude * deltaTime * Math.Sin(AccelerationMagnitude * deltaTime)) - (1)) / (Math.Pow(AccelerationMagnitude, 2))) * new dmat3(1, 0, 0, 0, 1, 0, 0, 0, 1)) +
            ((((Math.Sin(AccelerationMagnitude * deltaTime)) - (AccelerationMagnitude * deltaTime * Math.Cos(AccelerationMagnitude * deltaTime))) / (Math.Pow(AccelerationMagnitude, 3))) * SkewMatrix);

        return entityLocation + (InverseInitialOrientationMatrix * ((R1 * entityLinearVelocity) + (R2 * BodyAccelerationVector)));
    }

    /// <summary>
    /// Calculates the new orientation vector using the given velocity, acceleration, and time increment
    /// </summary>
    /// <param name="entityOrientation">The orientation of the entity in radians</param>
    /// <param name="entityAngularVelocity">The angular velocity vector in body coordinates</param>
    /// <param name="deltaTime">The time increment for dead reckoning calculation</param>
    /// <returns>The dead reckoning orientation matrix based off the given Entity Orientation and Entity Angular Velocity.</returns>
    private Orientation CalculateDeadReckonedOrientation(Orientation entityOrientation, dvec3 entityAngularVelocity, float deltaTime)
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
        if (Math.Abs(orientationOut.Theta) != Math.PI / 2)
        {
            CosTheta = glm.Cos(orientationOut.Theta);
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
    private dmat3 CreateDeadReckoningMatrix(dvec3 entityAngularVelocity, float deltaTime)
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
            (CosOmega * new dmat3(1, 0, 0, 0, 1, 0, 0, 0, 1)) -
            (SinOmega / AngularVelocityMagnitude * Conversions.CreateSkewMatrix(entityAngularVelocity));

        return DeadReckoningMatrix;
    }

    /// <summary>
    /// Calculates a DIS entity's orientation matrix from the provided Euler angles
    /// </summary>
    /// <param name="entityOrientation">The orientation of the entity in radians</param>
    /// <returns>The orienation matrix based off the given Entity Orientation.</returns>
    private dmat3 GetEntityOrientationMatrix(Orientation entityOrientation)
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
    private bool GetLocalQuaternionAngles(byte[] otherParameters, out Quaternion localRotation)
    {
        localRotation = new Quaternion();
        // Ensure the array is at least 15 bytes long
        if (otherParameters.Length < 15) { return false; }

        // Ensure the DR Parameter type is set to 2
        if (otherParameters[0] != 2) { return false; }

        //The next two bytes represent the 16 bit unsigned int approximation of q_0
        ushort Qu0 = (ushort)((otherParameters[1] << 8) + otherParameters[2]);

        // The x, y, and z components of the quaternion are the next three groups of four bytes
        float QuX = (otherParameters[3] << 24) + (otherParameters[4] << 16) + (otherParameters[5] << 8) + (otherParameters[6]);
        float QuY = (otherParameters[7] << 24) + (otherParameters[8] << 16) + (otherParameters[9] << 8) + (otherParameters[10]);
        float QuZ = (otherParameters[11] << 24) + (otherParameters[12] << 16) + (otherParameters[13] << 8) + (otherParameters[14]);

        // Calculate the appropriate Qu0
        Qu0 = (ushort)(Math.Sqrt(1 - (Math.Pow(QuX, 2.0) + Math.Pow(QuY, 2.0) + Math.Pow(QuZ, 2.0))));

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
    private dvec3 CalculateDeadReckonedPosition(dvec3 entityLocation, dvec3 entityLinearVelocity, dvec3 entityAcceleration, float deltaTime)
    {
        return entityLocation + (entityLinearVelocity * deltaTime) + (0.5 * entityAcceleration * Math.Pow(deltaTime, 2));
    }

    /// <summary>
    /// Gets the local yaw, pitch, and roll from the other parameters structure. The yaw, pitch, and roll act on the entity's local North, East, Down vectors.
    /// </summary>
    /// <param name="otherParameters">The 120 bits sent as part of the dead reckoning parameters marked as other parameters sent as an array of bytes</param>
    /// <param name="localRotation">The local yaw, pitch, and roll of the entity. Yaw is the heading from true north, positive to the right, in radians. Pitch is the elevation angle above or below the local horizon, positive up, in radians. Roll is the bank angle from the local horizontal, positive tile to the right, in radians.</param>
    /// <returns>Whether or not the data was loaded correctly. False if not loaded correctly.</returns>
    private bool GetLocalEulerAngles(byte[] otherParameters, out Orientation localRotation)
    {
        localRotation = null;
        // Ensure the Array is at least 15 bytes long
        if (otherParameters.Length < 15) { return false; }

        // Ensure the DR Parameter type is set to 1
        if (otherParameters[0] != 1) { return false; }

        // The next 2 bytes are padding and not necessary so skip indices 1 and 2
        // Concatenate the next three groups of four bytes
        float LocalYaw = (otherParameters[3] << 24) + (otherParameters[4] << 16) + (otherParameters[5] << 8) + (otherParameters[6]);
        float LocalPitch = (otherParameters[7] << 24) + (otherParameters[8] << 16) + (otherParameters[9] << 8) + (otherParameters[10]);
        float LocalRoll = (otherParameters[11] << 24) + (otherParameters[12] << 16) + (otherParameters[13] << 8) + (otherParameters[14]);

        // TODO: Change to psi/theta/phi instead of yaw/pitch/roll
        localRotation = new Orientation
        {
            Psi = LocalYaw,
            Theta = LocalPitch,
            Phi = LocalRoll
        };

        return true;
    }

    EntityStatePdu SmoothDeadReckoning(EntityStatePdu DeadReckonPDUToSmooth)
    {
        EntityStatePdu SmoothedDeadReckonPDU = DeadReckonPDUToSmooth;

        float alpha = MapRangeClamped(DeltaTimeSinceLastPDU, 0.0f, DeadReckoningSmoothingPeriodSeconds, 0.0f, 1.0f);

        //Lerp location for smoothing
        SmoothedDeadReckonPDU.EntityLocation.X -= Lerp(EntityECEFLocationDifference.X, 0.0f, alpha);
        SmoothedDeadReckonPDU.EntityLocation.Y -= Lerp(EntityECEFLocationDifference.Y, 0.0f, alpha);
        SmoothedDeadReckonPDU.EntityLocation.Z -= Lerp(EntityECEFLocationDifference.Z, 0.0f, alpha);

        Vector3 lerpRot = Vector3.Lerp(EntityRotationDifference, Vector3.zero, alpha);

        SmoothedDeadReckonPDU.EntityOrientation.Psi -= lerpRot.x;
        SmoothedDeadReckonPDU.EntityOrientation.Theta -= lerpRot.y;
        SmoothedDeadReckonPDU.EntityOrientation.Phi -= lerpRot.z;

        return SmoothedDeadReckonPDU;
    }

    double Lerp(double a, double b, float t)
    {
        return a + (b - a) * Mathf.Clamp01(t);
    }

    float MapRangeClamped(float Value, float InRangeA, float InRangeB, float OutRangeA, float OutRangeB)
    {
        float valuePercentInStartRange = 0.0f;

        float divisor = InRangeB - InRangeA;

        if (Mathf.Approximately(divisor, 0.0f))
        {
            return (Value >= InRangeB) ? 1.0f : 0.0f;
        }
        else
        {
            valuePercentInStartRange = Mathf.Clamp01((Value - InRangeA) / divisor);
        }

        return Mathf.Lerp(OutRangeA, OutRangeB, valuePercentInStartRange);
    }

    ////////////////////////////////////////////////////////////////////////
    ///////////////////    Begin Overridable Functions    //////////////////
    ////////////////////////////////////////////////////////////////////////

    //Override the below functions in a new class that inherits from the DISComponent if custom behavior is desired

    /// <summary>
    /// Clamps an entity to the ground.
    /// </summary>
    public virtual void GroundClamping()
    {
        //Verify that ground clamping is enabled, the entity is owned by another sim, is of the ground domain, and that it is not a munition
        if (SpawnedFromNetwork && (PerformGroundClamping == EGroundClampingMode.AlwaysGroundClamp || (PerformGroundClamping == EGroundClampingMode.GroundClampWithDISOptions && CurrentEntityType.domain == 1 && CurrentEntityType.entityKind != 2)))
        {
            Vector3Double llh;
            Conversions.CalculateLatLonHeightFromEcefXYZ(MostRecentDeadReckoningPDU.EntityLocation, out llh);

            dvec3 northVector;
            dvec3 eastVector;
            dvec3 downVector;
            Conversions.CalculateNorthEastDownVectorsFromLatLon(llh.X, llh.Y, out northVector, out eastVector, out downVector);

            Vector3 clampDirection = new Vector3((float)downVector.x, (float)downVector.y, (float)downVector.z);

            // TODO: Update the entityLocation to go off of the most recent dead reckoning update location in Unity coordinates

            Vector3 entityLocation = gameObject.transform.position;
            Vector3 raycastEndLocation = (clampDirection * 100000) + entityLocation;
            Vector3 raycastStartLocation = (clampDirection * -100000) + entityLocation;

            float raycastDistance = Vector3.Distance(raycastStartLocation, raycastEndLocation);

            RaycastHit hit;
            // Does the ray intersect any objects excluding the player layer
            if (Physics.Raycast(raycastStartLocation, clampDirection, out hit, raycastDistance, GroundClampingCollisionLayer))
            {
                Vector3 clampLocation = hit.point;
                Quaternion clampRotation = Quaternion.FromToRotation(Vector3.forward, hit.normal);

                transform.position = clampLocation;
                transform.rotation = clampRotation;
            }
        }
    }

    private void OnDestroy()
    {
        if (disGameManagerScript)
        {
            disGameManagerScript.RemoveDISEntityFromMap(CurrentEntityID.toEntityID());
        }
        else
        {
            FindObjectOfType<DISGameManager>().RemoveDISEntityFromMap(CurrentEntityID.toEntityID());
        }
    }
}
