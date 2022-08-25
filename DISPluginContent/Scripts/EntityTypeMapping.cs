using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class EntityTypeMapping
{
    public string friendlyName;

    public List<EntityTypeEditor> entityTypes;
    
    public GameObject gameObject;
}
