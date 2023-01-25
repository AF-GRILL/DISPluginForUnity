using GRILLDIS;
using System.Collections.Generic;
using UnityEngine;

namespace GRILLDIS
{
    [CreateAssetMenu(fileName = "Data", menuName = "GRILL DIS/DIS Enumeration Mapping", order = 1)]
    public class DISEnumerationMappings : ScriptableObject
    {
        /// <summary>
        /// The mapping between DIS Entity IDs and corresponding entity actors.
        /// </summary>
        [SerializeField]
        [Tooltip("The mapping between DIS Entity IDs and corresponding entity actors.")]
        public List<EntityTypeMapping> EntityTypeMappings = new List<EntityTypeMapping>();
    }
}