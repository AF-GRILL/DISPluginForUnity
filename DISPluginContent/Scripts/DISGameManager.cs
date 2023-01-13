using OpenDis.Dis1998;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Events;

namespace GRILLDIS
{
    public class DISGameManager : MonoBehaviour
    {
        /// <summary>
        /// The Exercise ID of the DIS sim. Valid Exercise IDs range from 0 to 255.
        /// </summary>
        [Header("DIS Simulation Settings")]
        [Range(0, 255)]
        [Tooltip("The Exercise ID of the DIS sim. Valid Exercise IDs range from 0 to 255.")]
        public int ExerciseID = 0;
        /// <summary>
        /// The Site ID of this application instance. Valid Site IDs range from 0 to 65535.
        /// </summary>
        [Range(0, 65535)]
        [Tooltip("The Site ID of this application instance. Valid Site IDs range from 0 to 65535.")]
        public int SiteID = 0;
        /// <summary>
        /// The Application ID of this application instance. Valid Application IDs range from 0 to 65535.
        /// </summary>
        [Range(0, 65535)]
        [Tooltip("The Application ID of this application instance. Valid Application IDs range from 0 to 65535.")]
        public int ApplicationID = 0;

        /// <summary>
        /// The mapping between DIS Entity IDs and corresponding entity actors.
        /// </summary>
        [Tooltip("The mappings between DIS Entity IDs and corresponding entity actors.")]
        public DISEnumerationMappings DISEnumerationMapping;

        /// <summary>
        /// The parent container that spawned DIS entities should be placed in.
        /// </summary>
        [Tooltip("The parent container that spawned DIS entities should be placed in.")]
        public GameObject DISEntityParentContainer;

        [Header("")]
        public UnityEvent<GameObject, EntityStatePdu> e_CreateDISEntity;
        public UnityEvent<GameObject> e_DestroyDISEntity;
        public UnityEvent<EntityStatePdu> e_NoDISEntity;

        private Dictionary<UInt64, GameObject> entityIDDictionary;
        private Dictionary<UInt64, GameObject> entityTypeDictionary;
        private GeoreferenceSystem georeferenceScript;
        //Add event for handling no mapping for entity

        public DISGameManager()
        {
            entityIDDictionary = new Dictionary<UInt64, GameObject>();
            entityTypeDictionary = new Dictionary<UInt64, GameObject>();
        }

        private void Awake()
        {
            InitializeEntityTypeMappings();

            georeferenceScript = GetComponent<GeoreferenceSystem>();
        }

        private void InitializeEntityTypeMappings()
        {
            foreach (EntityTypeMapping entityTypeMapping in DISEnumerationMapping.EntityTypeMappings)
            {
                foreach (EntityTypeEditor entityTypeEditor in entityTypeMapping.entityTypes)
                {
                    EntityType entityType = entityTypeEditor.toEntityType();
                    GameObject gameObject = entityTypeMapping.gameObject;

                    UInt64 entityTypeU64 = PDUUtil.EntityTypeToUInt64(entityType);

                    //Check if mapping already exists and update to new value if it does
                    try
                    {
                        entityTypeDictionary.Add(entityTypeU64, gameObject);
                    }
                    catch (ArgumentException)
                    {
                        Debug.LogWarning("A DIS Enumeration already exists for " + entityTypeEditor.ToString() + " and is linked to " + entityTypeDictionary[entityTypeU64].name + ". This enumeration will now point to: " + gameObject.name);
                        entityTypeDictionary[entityTypeU64] = gameObject;
                    }
                }
            }
        }

        /// <summary>
        /// Spawn a new entity from the given Entity State PDU that was received on the network. Gets an existing Entity if one exists with the associated Entity ID.
        /// </summary>
        /// <param name="entityStatePdu">The Entity State PDU to spawn or retreive a game object from.</param>
        /// <param name="DISEntityParentContainerIn">The parent container that a new spawned entity should be placed in.</param>
        /// <returns></returns>
        public GameObject SpawnOrGetGameObjectFromEntityStatePDU(EntityStatePdu entityStatePdu)
        {
            EntityID entityID = entityStatePdu.EntityID;
            EntityType entityType = entityStatePdu.EntityType;
            UInt64 entityIDU64 = PDUUtil.EntityIDToUInt64(entityID);
            string markingString = PDUUtil.getMarkingAsString(entityStatePdu);
            string markingSAE = PDUUtil.getEntityStatePDUMarkingSAE(entityStatePdu);
            GameObject entityGameObject;

            if (entityIDDictionary.TryGetValue(entityIDU64, out entityGameObject))
            {
                return entityGameObject;
            }
            //This is the first time we have seen this entity state PDU
            else
            {
                UInt64 entityTypeU64 = PDUUtil.EntityTypeToUInt64(entityType);

                if ((entityStatePdu.EntityAppearance & (1 << 23)) != 0)
                {
                    Debug.Log("Received Entity State PDU with a Deactivated Entity Appearance for an entity that is not in the level. Ignoring the PDU. Entity marking: " + markingSAE);
                    return null;
                }

                //Check if this entity has a mapping specified
                if (entityTypeDictionary.TryGetValue(entityTypeU64, out entityGameObject))
                {
                    string entityTypeString = PDUUtil.getEntityTypeAsString(entityStatePdu);

                    if (entityGameObject)
                    {
                        Vector3 spawnPosition = Vector3.zero;
                        Quaternion spawnRotation = Quaternion.Euler(0, 0, 0);
                        if (georeferenceScript)
                        {
                            Conversions.GetUnityLocationAndOrientationFromEntityStatePdu(entityStatePdu, georeferenceScript, out Vector3Double unityLoc, out Vector3 unityRot);
                            spawnPosition = new Vector3((float)unityLoc.X, (float)unityLoc.Y, (float)unityLoc.Z);
                            spawnRotation = Quaternion.Euler(unityRot);
                        }

                        GameObject newGameObject;
                        if (DISEntityParentContainer)
                        {
                            newGameObject = Instantiate(entityGameObject, spawnPosition, spawnRotation, DISEntityParentContainer.transform);
                            e_CreateDISEntity.Invoke(newGameObject, entityStatePdu);
                        }
                        else
                        {
                            newGameObject = Instantiate(entityGameObject, spawnPosition, spawnRotation);
                            e_CreateDISEntity.Invoke(newGameObject, entityStatePdu);
                        }

                        //Update that this Game Object was spawned by the network
                        DISReceiveComponent objectDISReceiveComponent = newGameObject.GetComponent<DISReceiveComponent>();
                        if (objectDISReceiveComponent)
                        {
                            objectDISReceiveComponent.SpawnedFromNetwork = true;
                            objectDISReceiveComponent.disGameManagerScript = this;
                            objectDISReceiveComponent.georeferenceScript = this.gameObject.GetComponent<GeoreferenceSystem>();
                        }

                        AddDISEntityToMap(entityID, newGameObject);

                        return newGameObject;
                    }
                    else
                    {
                        Debug.LogWarning("Mapping points to a null GameObject for " + markingString + " (" + entityTypeString + ")");

                        return null;
                    }
                }
                else
                {
                    Debug.LogWarning("No mapping found for " + markingSAE);

                    return null;
                }
            }
        }

        /// <summary>
        /// Get the DIS Component associated with the given Entity ID
        /// </summary>
        /// <param name="EntityIDIn">Entity ID to get the DIS Component of.</param>
        /// <returns>The associated DIS Component of the given Entity ID. Null if an entity with the given ID was not found.</returns>
        public DISReceiveComponent GetAssociatedDISReceiveComponent(EntityID EntityIDIn)
        {
            UInt64 entityIDU64 = PDUUtil.EntityIDToUInt64(EntityIDIn);
            GameObject gameObject;
            DISReceiveComponent DISReceiveComponentScript = null;

            if (entityIDDictionary.TryGetValue(entityIDU64, out gameObject))
            {
                DISReceiveComponentScript = gameObject.GetComponent<DISReceiveComponent>();
            }

            return DISReceiveComponentScript;
        }

        /// <summary>
        /// Adds a new Entity ID to the Entity ID map.
        /// </summary>
        /// <param name="entityIDToAdd">The Entity ID to add to the map.</param>
        /// <param name="gameObjectToMapTo">The Game Object the Entity ID will point to in the map.</param>
        /// <returns>Whether or not the addition to the map was successful.</returns>
        public bool AddDISEntityToMap(EntityID entityIDToAdd, GameObject gameObjectToMapTo)
        {
            bool successful = false;

            if (!gameObjectToMapTo)
            {
                Debug.LogWarning("Given Game Object to add to DIS Entity ID map was null. Skipping adding it...");
                return successful;
            }

            UInt64 entityIDU64 = PDUUtil.EntityIDToUInt64(entityIDToAdd);
            EntityIDEditor entityIDEditor = new EntityIDEditor();
            entityIDEditor.fromEntityID(entityIDToAdd);

            try
            {
                entityIDDictionary.Add(entityIDU64, gameObjectToMapTo);
            }
            catch (ArgumentException)
            {
                Debug.LogWarning("A DIS Entity ID already exists for " + entityIDEditor.ToString() + " and is linked to " + entityIDDictionary[entityIDU64].name + ". This entity ID will now point to: " + gameObjectToMapTo.name);
                entityIDDictionary[entityIDU64] = gameObject;
            }

            successful = true;
            return successful;
        }

        public bool RemoveDISEntityFromMap(EntityID EntityIDToRemove)
        {
            UInt64 entityIDU64 = PDUUtil.EntityIDToUInt64(EntityIDToRemove);
            return entityIDDictionary.Remove(entityIDU64);
        }

    }
}