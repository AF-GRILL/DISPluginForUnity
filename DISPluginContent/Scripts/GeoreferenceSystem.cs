using OpenDis.Dis1998;
using UnityEngine;
using GlmSharp;
using Esri.HPFramework;
using Esri.ArcGISMapsSDK.Utils.Math;

public class GeoreferenceSystem : MonoBehaviour
{
    const double EARTH_MAJOR_AXIS = 6378137;
    const double EARTH_MINOR_AXIS = 6356752.3142;
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
    private dvec4 Offset;

    private dmat4 WorldFrameToECEFFrame; // Matrix to transform a vector from EU to ECEF CRS
    private dmat4 ECEFFrameToWorldFrame; // Matrix to transform a vector from ECEF to UE CRS - Inverse of the previous one kept in cache. 

    private dmat4 WorldFrameToUnityFrame;
    private dmat4 UnityFrameToWorldFrame;

    // Start is called before the first frame update
    void Awake()
    {
        SetupVars();

        #region Debugging
        
        Debug.Log("Origin (ECEF): " + Origin.X + ", " + Origin.Y + ", " + Origin.Z);
        Debug.Log("Offset (Unity): " + Offset.xyz);

        Vector3Double output;
        Conversions.CalculateEcefXYZFromLatLonHeight(new Vector3Double { X = OriginLat, Y = OriginLon, Z = OriginAlt + 1 }, out output);
        Debug.Log("Up:   " + (GetUnityCoordinatesFromECEF(output) - Offset).xyz);

        Conversions.CalculateEcefXYZFromLatLonHeight(new Vector3Double { X = OriginLat + 1, Y = OriginLon, Z = OriginAlt }, out output);
        Debug.Log("Lat:   " + (GetUnityCoordinatesFromECEF(output) - Offset).xyz);

        Conversions.CalculateEcefXYZFromLatLonHeight(new Vector3Double { X = OriginLat, Y = OriginLon + 1, Z = OriginAlt }, out output);
        Debug.Log("Lon:   " + (GetUnityCoordinatesFromECEF(output) - Offset).xyz);
        
        #endregion Debugging
    }

    #region PublicFunctions

    public dvec3 LatLonAltToUnity(Vector3Double LatLonAlt)
    {
        Vector3Double ECEF;
        Conversions.CalculateEcefXYZFromLatLonHeight(LatLonAlt, out ECEF);
        return (GetUnityCoordinatesFromECEF(ECEF) - Offset).xyz;
    }

    public dvec3 ECEFToUnity(Vector3Double ECEF)
    {
        return (GetUnityCoordinatesFromECEF(ECEF) - Offset).xyz;
    }

    public Vector3Double UnityToLatLonAlt(dvec3 Unity)
    {
        return GetLatLonAltFromUnity(Unity);

    }

    public Vector3Double UnityToECEF(dvec3 Unity)
    {
        return GetECEFFromUnity(Unity);
    }
    public Vector3Double GetOrigin() { return Origin; }

    #endregion PublicFunctions

    #region PrivateFunctions
    private void SetupVars()
    {
        Vector3Double b = new Vector3Double
        {
            X = OriginLat,
            Y = OriginLon,
            Z = OriginAlt
        };

        Conversions.CalculateEcefXYZFromLatLonHeight(b, out Origin);
        GetWorldFrameToECEFFrame(Origin);
        GetWorldFrameToUnityFrame();
        Offset = GetUnityCoordinatesFromECEF(Origin);

    }

    private void GetWorldFrameToECEFFrame(Vector3Double ECEF)
    {
        if (Abs(ECEF.X) < 0.000000001 && Abs(ECEF.Y) < 0.000000001)
        {
            Debug.Log("Along Pole");
            //Origin at Center
            if (Abs(ECEF.Z) < Mathf.Epsilon)
            {
                //Works
                WorldFrameToECEFFrame =
                    new dmat4(0, -1, 0, ECEF.X,
                              1, 0, 0, ECEF.Y,
                              0, 0, 1, ECEF.Z,
                              0, 0, 0, 1);

                //Origin at N/S Pole
            } else
            {
                //Works
                double Sign = Mathf.Sign((float)ECEF.Z);
                WorldFrameToECEFFrame =
                    new dmat4(0, -1 * Sign, 0, ECEF.X,
                              1, 0, 0, ECEF.Y,
                              0, 0, 1 * Sign, ECEF.Z,
                              0, 0, 0, 1);
            }
        }
        else
        {
            dvec3 Up = new dvec3(ECEF.X / (EARTH_MAJOR_AXIS * EARTH_MAJOR_AXIS), ECEF.Y / (EARTH_MAJOR_AXIS * EARTH_MAJOR_AXIS), ECEF.Z / (EARTH_MINOR_AXIS * EARTH_MINOR_AXIS)).Normalized;

            dvec3 East = new dvec3(-ECEF.Y, ECEF.X, 0).Normalized;

            dvec3 North = dvec3.Cross(Up, East);


            dmat4 mat = new dmat4
                (East.x, East.y, East.z, 0,
                 North.x, North.y, North.z, 0,
                    Up.x, Up.y, Up.z, 0,
                  ECEF.X, ECEF.Y, ECEF.Z, 1);


            dmat4 matTest = new dmat4
                (East.x, North.x, Up.x, ECEF.X,
                 East.y, North.y, Up.y, ECEF.Y,
                 East.z, North.z, Up.z, ECEF.Z,
                      0, 0, 0, 1);


            WorldFrameToECEFFrame = matTest;
        }
        ECEFFrameToWorldFrame = WorldFrameToECEFFrame.Inverse;
    }

    private void GetWorldFrameToUnityFrame()
    {
        dmat4 WorldFrameToUEFrame =
            new dmat4(1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);

        WorldFrameToUnityFrame = WorldFrameToUEFrame *
            new dmat4(0, 0, 1, 0,
                      1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 0, 1);

        UnityFrameToWorldFrame = WorldFrameToUnityFrame.Inverse;
    }

    private dvec4 GetUnityCoordinatesFromECEF(Vector3Double ECEF)
    {
        dvec4 multiplier = new dvec4((float)ECEF.X, (float)ECEF.Y, (float)ECEF.Z, 1);

        dvec4 WorldCoords = ECEFFrameToWorldFrame * multiplier;
        dvec4 UnityCoords = WorldFrameToUnityFrame * WorldCoords;

        return UnityCoords;
    }

    private Vector3Double GetECEFFromUnity(dvec3 Unity)
    {
        dvec4 Unity4 = new dvec4(Unity, 0);
        Unity4 += Offset;
        dvec4 WorldFrame = UnityFrameToWorldFrame * Unity4;
        dvec4 ECEF = WorldFrameToECEFFrame * WorldFrame;
        return new Vector3Double { X = ECEF.x, Y = ECEF.y, Z = ECEF.z };
    }

    private Vector3Double GetLatLonAltFromUnity(dvec3 Unity)
    {
        Vector3Double ECEF = GetECEFFromUnity(Unity);
        Vector3Double LatLonHeight;
        Conversions.CalculateLatLonHeightFromEcefXYZ(ECEF, out LatLonHeight);
        return LatLonHeight;
    }

    private double Abs(double i)
    {
        if (i < 0) { return -i; }
        else { return i; }
    }
    #endregion PrivateFunctions
}