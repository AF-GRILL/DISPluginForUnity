using OpenDis.Dis1998;
using UnityEngine;
using GlmSharp;
using System;

public class GeoreferenceSystem : MonoBehaviour
{
    const double EARTH_MAJOR_AXIS = 6378137;
    const double EARTH_MINOR_AXIS = 6356752.3142;
    const double EARTH_GEOCENTRIC_RADIUS = 6378137;
    const double DEG_TO_RAD = 0.017453292519943295;
    const double RAD_TO_DEG = 57.29577951308233;
    /// <summary>
    /// The Latitudinal equivalent for 0 on Unity's Z-Axis.
    /// </summary>
    [Header("DIS Georeference Settings")]
    [Tooltip("The Latitudinal equivalent for 0 in Unity's coordinate system.")]
    public double OriginLat = 0;
    /// <summary>
    /// The Longitudinal equivalent for 0 on Unity's X-Axis.
    /// </summary>
    [Tooltip("The Longitudinal equivalent for 0 in Unity's coordinate system.")]
    public double OriginLon = 0;
    /// <summary>
    /// The Altitudinal equivalent for 0 on Unity's Y-Axis.
    /// </summary>
    [Tooltip("The Altitudinal equivalent for 0 in Unity's coordinate system.")]
    public double OriginAlt = 0;

    private Vector3Double Origin;

    // Start is called before the first frame update
    void Awake()
    {
        SetupVars();

        #region Debugging

        //geodetic_to_flatearth
        
        Vector3Double temp = geodetic_to_flatearth(new Vector3Double { X = OriginLat + .001, Y = OriginLon + .001, Z = OriginAlt + .001 });
        Debug.Log("Origin: \nX: " + temp.X + "\nY: " + temp.Y + "\nZ: " + temp.Z);

        #endregion Debugging
    }

    #region PublicFunctions

    public Vector3Double LatLonAltToUnity(Vector3Double LatLonAlt)
    {
        return geodetic_to_flatearth(LatLonAlt);
    }

    public Vector3Double ECEFToUnity(Vector3Double ECEF)
    {
        Vector3Double LatLonAlt;
        Conversions.CalculateLatLonHeightFromEcefXYZ(ECEF, out LatLonAlt);
        return geodetic_to_flatearth(LatLonAlt);
    }

    public Vector3Double UnityToLatLonAlt(Vector3Double UnityChoords)
    {
        return flatearth_to_geodetic(UnityChoords);
    }

    public Vector3Double UnityToECEF(Vector3Double UnityChoords)
    {
        Vector3Double LatLonAlt = flatearth_to_geodetic(UnityChoords);
        Vector3Double ECEF;
        Conversions.CalculateEcefXYZFromLatLonHeight(LatLonAlt, out ECEF);
        return ECEF;
    }

    public Vector3Double GetOriginECEF() { return Origin; }

    #endregion PublicFunctions

    #region PrivateFunctions

    private void SetupVars()
    {
        Origin = new Vector3Double
        {
            X = OriginLat,
            Y = OriginLon,
            Z = OriginAlt
        };


    }

    private Vector3Double geodetic_to_flatearth(Vector3Double latlonalt)
    {
        Vector3Double lla = new Vector3Double
        {
            X = latlonalt.X * DEG_TO_RAD,
            Y = latlonalt.Y * DEG_TO_RAD,
            Z = latlonalt.Z,
        };

        double dlat = lla.X - (OriginLat * DEG_TO_RAD);
        double dlon = lla.Y - (OriginLon * DEG_TO_RAD);

        Vector3Double toReturn = new Vector3Double
        {
            X = (EARTH_GEOCENTRIC_RADIUS + lla.Z) * dlon * Math.Cos(lla.X), //East: possibly replace with minor axis
            Y = lla.Z - OriginAlt, //Up
            Z = (EARTH_GEOCENTRIC_RADIUS + lla.Z) * dlat //North: possibly replace with major axis
        };

        return toReturn;
    }

    private Vector3Double flatearth_to_geodetic(Vector3Double UnityChoords)
    {
        double dlat = (UnityChoords.Z / (EARTH_GEOCENTRIC_RADIUS + UnityChoords.Y));
        double dlon = (UnityChoords.X / ((EARTH_GEOCENTRIC_RADIUS + UnityChoords.Y) * Math.Cos(dlat + (OriginLat * DEG_TO_RAD))));

        Vector3Double toReturn = new Vector3Double
        {
            X = ((OriginLat * DEG_TO_RAD) + dlat) * RAD_TO_DEG, //Lat: accurate to the 6th decimal place, possibly replace with minor axis
            Y = ((OriginLon * DEG_TO_RAD) + dlon) * RAD_TO_DEG, //Lon: accurate to the 6th decimal place (2nd with cosine from above), possibly replace with major axis
            Z = UnityChoords.Y + OriginAlt //Alt
        };

        return toReturn;
    }

    #endregion PrivateFunctions

}

#region OldCode

/*
 
    private void GetWorldFrameToECEFFrame(Vector3Double ECEF)
    {
        if (Abs(ECEF.X) < 0.000000001 && Abs(ECEF.Y) < 0.000000001)
        {
            Debug.Log("Along N/S Pole");
            if (Abs(ECEF.Z) < Mathf.Epsilon) //Origin at planet center
            {
                WorldFrameToECEFFrame =
                    new dmat4(0, -1, 0, ECEF.X,
                              1,  0, 0, ECEF.Y,
                              0,  0, 1, ECEF.Z,
                              0,  0, 0,      1);

               
            } else //Origin at N/S Pole
            {
                double Sign = Mathf.Sign((float)ECEF.Z);
                WorldFrameToECEFFrame =
                    new dmat4(0, -1 * Sign,        0, ECEF.X,
                              1,         0,        0, ECEF.Y,
                              0,         0, 1 * Sign, ECEF.Z,
                              0,         0,        0,      1);
            }
        }
        else
        {
            dvec3 Up = new dvec3(ECEF.X / (EARTH_MAJOR_AXIS * EARTH_MAJOR_AXIS), ECEF.Y / (EARTH_MAJOR_AXIS * EARTH_MAJOR_AXIS), ECEF.Z / (EARTH_MINOR_AXIS * EARTH_MINOR_AXIS)).Normalized;

            dvec3 East = new dvec3(-ECEF.Y, ECEF.X, 0).Normalized;

            dvec3 North = dvec3.Cross(Up, East);

            dmat4 matTest = new dmat4
                (East.x, North.x, Up.x, ECEF.X,
                 East.y, North.y, Up.y, ECEF.Y,
                 East.z, North.z, Up.z, ECEF.Z,
                      0,       0,    0,      1);


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
*/

#endregion OldCode