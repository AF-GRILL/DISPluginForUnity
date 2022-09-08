using OpenDis.Dis1998;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class GeoreferenceSystem : MonoBehaviour
{

    /// <summary>
    /// The Latitudinal equivalent for 0 on Unity's Z-Axis.
    /// </summary>
    [Header("DIS Georeference Settings")]
    [Tooltip("The Latitudinal equivalent for 0 on Unity's Z-Axis.")]
    public double OriginLat = 0;
    /// <summary>
    /// The Longitudinal equivalent for 0 on Unity's X-Axis.
    /// </summary>
    [Tooltip("The Longitudinal equivalent for 0 on Unity's X-Axis.")]
    public double OriginLon = 0;
    /// <summary>
    /// The Altitudinal equivalent for 0 on Unity's Y-Axis.
    /// </summary>
    [Tooltip("The Altitudinal equivalent for 0 on Unity's Y-Axis.")]
    public double OriginAlt = 0;

    private Vector3Double Origin;

    // Start is called before the first frame update
    void Awake()
    {
        Vector3Double b = new Vector3Double { 
        X = OriginLat,
        Y = OriginLon,
        Z = OriginAlt
        };

        Conversions.CalculateEcefXYZFromLatLonHeight(b, out Origin);
    }

    public Vector3Double GetOrigin()
    {
        return Origin;
    }

    //Methods to move to Conversions
    public static void EcefToUnity (GeoreferenceSystem georeferenceSystem, Vector3Double Ecef, out Vector3Double UnityChoords)
    {
        Vector3Double Origin = new Vector3Double {
            X = georeferenceSystem.GetOrigin().X,
            Y = georeferenceSystem.GetOrigin().Y,
            Z = georeferenceSystem.GetOrigin().Z,
        };
        Vector3Double UnityChoordsOne = new Vector3Double {
            X = Ecef.X - Origin.X,
            Y = Ecef.Y - Origin.Y,
            Z = Ecef.Z - Origin.Z,
        };
        UnityChoords = UnityChoordsOne;
    }

    public static void LatLonHeightToUnity (GeoreferenceSystem georeferenceSystem, Vector3Double LatLonHeight, out Vector3Double UnityChoords)
    {
        Vector3Double ecef;
        Conversions.CalculateEcefXYZFromLatLonHeight(LatLonHeight, out ecef);
        EcefToUnity(georeferenceSystem, ecef, out UnityChoords);
    }
}
