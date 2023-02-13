using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenDis.Dis1998;
using System;
using UnityEngine.Events;
using GlmSharp;

namespace GRILLDIS
{
    [Serializable] public class DeadReckoningUpdate : UnityEvent<EntityStatePdu> { }
    [Serializable] public class EntityStatePDUProcessed : UnityEvent<EntityStatePdu> { }
    [Serializable] public class EntityStateUpdatePDUProcessed : UnityEvent<EntityStateUpdatePdu> { }
    [Serializable] public class FirePDUProcessed : UnityEvent<FirePdu> { }
    [Serializable] public class DetonationPDUProcessed : UnityEvent<DetonationPdu> { }
    [Serializable] public class RemoveEntityPDUProcessed : UnityEvent<RemoveEntityPdu> { }
    [Serializable] public class StopFreezePDUProcessed : UnityEvent<StopFreezePdu> { }
    [Serializable] public class StartResumePDUProcessed : UnityEvent<StartResumePdu> { }

    public class DISReceiveComponent : MonoBehaviour
    {
        /// <summary>
        /// The time to live for the entity. Gets reset every time a new Entity State PDU is received by the sim.
        /// </summary>
        [Header("DIS Settings")]
        [Range(0.0f, 3600.0f)]
        [Tooltip("The time to live for the entity. Gets reset every time a new Entity State PDU is received by the sim.")]
        public float DISTimeoutSeconds = 30.0f;
        /// <summary>
        /// Determines what all DIS info should be culled. Allows for updates to happen less frequently for entities that aren't currently important. The distance that culling should happen is handled by the DISCullingDistance variable.
        /// </summary>
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
        /// The Entity Type of the associated entity. Specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.
        /// </summary>
        [HideInInspector]
        public EntityTypeEditor CurrentEntityType;
        /// <summary>
        /// The Entity ID of the associated entity. Each Entity ID should be unique to an entity in the sim.
        /// </summary>
        [HideInInspector]
        public EntityIDEditor CurrentEntityID;
        /// <summary>
        /// The Force ID of the associated entity. Specifies the team or side the DIS entity is on.
        /// </summary>
        [HideInInspector]
        public EForceID EntityForceID;
        /// <summary>
        /// The Entity Marking of the associated entity. Designates a friendly name for the DIS entity. Max of 11 characters should be used.If more than 11 are used, it will be truncated.
        /// </summary>
        [HideInInspector]
        public string EntityMarking;
        /// <summary>
        /// The timestamp that the most recent Entity State PDU was received at by the DISReceiveComponent.
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


        [HideInInspector]
        public DISGameManager disGameManagerScript;
        [HideInInspector]
        public GeoreferenceSystem georeferenceScript;

        private int NumberEntityStatePDUsReceived = 0;
        private Coroutine DISHeartbeatCoroutine;
        private float DeltaTimeSinceLastPDU = 0.0f;

        /// <summary>
        /// The difference in ECEF Location from the previous PDU to the current PDU.
        /// </summary>
        private Vector3Double EntityECEFLocationDifference = new Vector3Double();
        /// <summary>
        /// The difference in Orientation from the previous PDU to the current PDU.
        /// </summary>
        private Vector3 EntityRotationDifference;

        private void Start()
        {
            if (georeferenceScript == null)
            {
                georeferenceScript = FindObjectOfType<GeoreferenceSystem>();
            }
        }

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
            disGameManagerScript.e_DestroyDISEntity.Invoke(this.gameObject, EDestroyCode.TimeOut);
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
                disGameManagerScript.e_DestroyDISEntity.Invoke(this.gameObject, EDestroyCode.Disabled);
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
                disGameManagerScript.e_DestroyDISEntity.Invoke(this.gameObject, EDestroyCode.Disabled);
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
            DeltaTimeSinceLastPDU = 0.0f;

            //Get difference in ECEF between most recent dead reckoning location and last known Dead Reckoning location
            EntityECEFLocationDifference.X = NewEntityStatePDU.EntityLocation.X - MostRecentDeadReckoningPDU.EntityLocation.X;
            EntityECEFLocationDifference.Y = NewEntityStatePDU.EntityLocation.Y - MostRecentDeadReckoningPDU.EntityLocation.Y;
            EntityECEFLocationDifference.Z = NewEntityStatePDU.EntityLocation.Z - MostRecentDeadReckoningPDU.EntityLocation.Z;

            Vector3 rotDiffDegrees = new Vector3(glm.Degrees(NewEntityStatePDU.EntityOrientation.Psi) - glm.Degrees(MostRecentDeadReckoningPDU.EntityOrientation.Psi),
                glm.Degrees(NewEntityStatePDU.EntityOrientation.Theta) - glm.Degrees(MostRecentDeadReckoningPDU.EntityOrientation.Theta),
                glm.Degrees(NewEntityStatePDU.EntityOrientation.Phi) - glm.Degrees(MostRecentDeadReckoningPDU.EntityOrientation.Phi));

            rotDiffDegrees = Conversions.UnwindRotation(rotDiffDegrees);
            EntityRotationDifference = new Vector3(glm.Radians(rotDiffDegrees.x), glm.Radians(rotDiffDegrees.y), glm.Radians(rotDiffDegrees.z));

            MostRecentEntityStatePDU = NewEntityStatePDU;
            MostRecentDeadReckoningPDU = PDUUtil.DeepCopyEntityStatePDU(MostRecentEntityStatePDU);

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

                if (DeadReckoningLibrary.DeadReckoning(MostRecentEntityStatePDU, DeltaTimeSinceLastPDU, ref MostRecentDeadReckoningPDU))
                {
                    //If more than one PDU has been received and we're still in the smoothing period, then smooth
                    if (PerformDeadReckoningSmoothing && NumberEntityStatePDUsReceived > 1 && DeltaTimeSinceLastPDU <= DeadReckoningSmoothingPeriodSeconds)
                    {
                        MostRecentDeadReckoningPDU = SmoothDeadReckoning(MostRecentDeadReckoningPDU);
                    }

                    OnDeadReckoningUpdate.Invoke(MostRecentDeadReckoningPDU);
                }

                //Perform ground clamping last
                GroundClamping();
            }
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

        //Override the below functions in a new class that inherits from the DISReceiveComponent if custom behavior is desired

        /// <summary>
        /// Clamps an entity to the ground.
        /// </summary>
        public virtual void GroundClamping()
        {
            //Verify that ground clamping is enabled, the entity is owned by another sim, is of the ground domain, and that it is not a munition
            if (SpawnedFromNetwork && (PerformGroundClamping == EGroundClampingMode.AlwaysGroundClamp || (PerformGroundClamping == EGroundClampingMode.GroundClampWithDISOptions && CurrentEntityType.domain == 1 && CurrentEntityType.entityKind != 2)))
            {
                FLatLonAlt llh = Conversions.CalculateLatLonHeightFromEcefXYZ(MostRecentDeadReckoningPDU.EntityLocation);

                FNorthEastDown northEastDownVectors = Conversions.CalculateNorthEastDownVectorsFromLatLon(llh.Latitude, llh.Longitude);

                Vector3 clampDirection = northEastDownVectors.DownVector;

                Vector3 entityLocation = gameObject.transform.position;
                if (georeferenceScript)
                {
                    //Get the location the object is supposed to be at according to the most recent dead reckoning update.
                    Vector3Double mostRecentDRPosition = georeferenceScript.ECEFToUnityFlatearth(MostRecentDeadReckoningPDU.EntityLocation);
                    entityLocation = new Vector3((float)mostRecentDRPosition.X, (float)mostRecentDRPosition.Y, (float)mostRecentDRPosition.Z);
                }

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
                FindObjectOfType<DISGameManager>()?.RemoveDISEntityFromMap(CurrentEntityID.toEntityID());
            }
        }
    }

}