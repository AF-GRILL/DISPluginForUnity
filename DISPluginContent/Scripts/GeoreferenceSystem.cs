// *************************************
// * Classification: UNCLASSIFIED      *
// *************************************

//*********************************************************************
//*********************************************************************
//***
//*** Property of the US Govt, AFMC/AFRL/SNZW
//***
//***       Methods:  geodetic_to_flatearth
//***              :  flatearth_to_geodetic
//***              :  geodetic_to_squareflatearth
//***              :  squareflatearth_to_geodetic
//***              :  FlatEarth2WGS_att
//***              :  WGS2FlatEarth_att
//***              :  FlatEarth2WGS_vect
//***              :  WGS2FlatEarth_vect
//***
//*** Description:  The above methods were converted from CoordinateTranslate class 
//***    to help translate between different coordinate systems. 
//***    The current routines mainly make it possible to translate
//***    between FlatEarth to Geocentric (WGS-84) and back again.
//***
//***
//***      Author: Ty W. Hayden
//***     Company: Ball Aerospace Corp. Dayton, OH
//***        Date: 12 July 1996
//***
//*********************************************************************
//*********************************************************************

using OpenDis.Dis1998;
using UnityEngine;
using System;
using GlmSharp;
using System.Collections;

public class GeoreferenceSystem : MonoBehaviour
{
    const double EARTH_MAJOR_AXIS = 6378137;
    const double EARTH_MINOR_AXIS = 6356752.3142;
    const double EARTH_GEOCENTRIC_RADIUS = 6378137;

    /// <summary>
    /// The Latitude in decimal degrees, Longitude in decimal degrees, and Altitude in meters of the Unity origin (0, 0, 0).
    /// </summary>
    [Header("DIS Georeference Settings")]
    [Tooltip("The Latitude in decimal degrees, Longitude in decimal degrees, and Altitude in meters of the Unity origin (0, 0, 0).")]
    public FLatLonAlt OriginLLA;

    private Vector3Double OriginECEF;
    private Vector3Double temp;
    // Start is called before the first frame update
    void Awake()
    {
        SetupVars();
    }

    #region DebuggingMethods

    IEnumerator LoopCalc()
    {
        int i = 0;
        while (true)
        {

            Debug.Log("ECEFToUnity " + i + ": " + temp.X + ", " + temp.Y + ", " + temp.Z);
            Vector3 unityLoc = new Vector3((float)temp.X, (float)temp.Y, (float)temp.Z);
            temp = UnityToECEF(unityLoc);
            FLatLonAlt tempLLA = Conversions.CalculateLatLonHeightFromEcefXYZ(temp);
            //Debug.Log("UnityToECEF " + i + ": " + tempLLA.Latitude + ", " + tempLLA.Longitude + ", " + tempLLA.Altitude);
            temp = ECEFToUnity(temp);
            i += 1;
            yield return new WaitForSeconds(2.5f);
        }
    }


    #endregion DebuggingMethods

    #region PublicFunctions

    /// <summary>
    /// Converts the given Lat, Lon, Alt coordinates to Unity coordinates.
    /// </summary>
    /// <param name="LatLonAlt">The Lat (X), Lon (Y), Alt (Z) coordinates to transform.</param>
    /// <returns>The Unity coordinates.</returns>
    public Vector3Double LatLonAltToUnity(FLatLonAlt LatLonAlt)
    {
        Vector3Double flatEarthLoc = geodetic_to_flatearth(LatLonAlt);

        //Transform the flat Earth LLA location to be in terms of Unity coordinates
        Vector3Double unityLoc = new Vector3Double
        {
            X = flatEarthLoc.X,
            Y = flatEarthLoc.Z,
            Z = flatEarthLoc.Y
        };

        return unityLoc;
    }

    /// <summary>
    /// Converts the given ECEF coordinates to Unity coordinates.
    /// </summary>
    /// <param name="ECEF">The ECEF XYZ coordinates to transform.</param>
    /// <returns>The Unity coordinates.</returns>
    public Vector3Double ECEFToUnity(Vector3Double ECEF)
    {
        Vector3Double flatEarthLoc = ecef_to_flatearth(ECEF);

        //Transform the flat Earth ECEF location to be in terms of Unity coordinates
        Vector3Double unityLoc = new Vector3Double
        {
            X = flatEarthLoc.X,
            Y = flatEarthLoc.Z,
            Z = flatEarthLoc.Y
        };

        return unityLoc;
    }

    /// <summary>
    /// Converts the given Unity coordinates into geodetic Lat, Lon, Alt coordinates.
    /// </summary>
    /// <param name="UnityCoords">The Unity coordinates to transform.</param>
    /// <returns>The Lat (X), Lon (Y), Alt (Z) coordinates.</returns>
    public FLatLonAlt UnityToLatLonAlt(Vector3 UnityCoords)
    {
        //Transform the given Unity coordinates to be in terms of a flat Earth coordinate system
        Vector3Double flatEarthCoords = new Vector3Double
        {
            X = UnityCoords.x,
            Y = UnityCoords.z,
            Z = UnityCoords.y
        };

        return flatearth_to_geodetic(flatEarthCoords);
    }

    /// <summary>
    /// Converts the given Unity coordinates into ECEF coordinates.
    /// </summary>
    /// <param name="UnityCoords">The Unity coordinates to transform.</param>
    /// <returns>The ECEF XYZ coordinates.</returns>
    public Vector3Double UnityToECEF(Vector3 UnityCoords)
    {
        //Transform the given Unity coordinates to be in terms of a flat Earth coordinate system
        Vector3Double flatEarthCoords = new Vector3Double
        {
            X = UnityCoords.x,
            Y = UnityCoords.z,
            Z = UnityCoords.y
        };

        return flatearth_to_ecef(flatEarthCoords);
    }

    /// <summary>
    /// Get the North, East, Down vectors at the given Unity location.
    /// </summary>
    /// <param name="UnityLocation">The Unity location to get the North, East, Down vectors of.</param>
    /// <returns>The North, East, Down vectors.</returns>
    public FNorthEastDown GetNEDVectorsAtEngineLocation(Vector3 UnityLocation)
    {
        Vector3Double ecefLoc = UnityToECEF(UnityLocation);
        FLatLonAlt lla = Conversions.CalculateLatLonHeightFromEcefXYZ(ecefLoc);
        return Conversions.CalculateNorthEastDownVectorsFromLatLon(lla.Latitude, lla.Longitude);
    }

    /// <summary>
    /// Get the East, North, Up vectors at the given Unity location.
    /// </summary>
    /// <param name="UnityLocation">The Unity location to get the East, North, Up vectors of.</param>
    /// <returns>The East, North, Up vectors.</returns>
    public FEastNorthUp GetENUVectorsAtEngineLocation(Vector3 UnityLocation)
    {
        Vector3Double ecefLoc = UnityToECEF(UnityLocation);
        FLatLonAlt lla = Conversions.CalculateLatLonHeightFromEcefXYZ(ecefLoc);
        FNorthEastDown northEastDownVectors = Conversions.CalculateNorthEastDownVectorsFromLatLon(lla.Latitude, lla.Longitude);
        return new FEastNorthUp(northEastDownVectors.EastVector, northEastDownVectors.NorthVector, -northEastDownVectors.DownVector);
    }

    public FLatLonAlt GetOriginLLA() { return OriginLLA; }
    public Vector3Double GetOriginECEF() { return OriginECEF; }

    #endregion PublicFunctions

    #region PrivateFunctions

    private void SetupVars()
    {
        OriginECEF = Conversions.CalculateEcefXYZFromLatLonHeight(OriginLLA);
        FLatLonAlt temp = Conversions.CalculateLatLonHeightFromEcefXYZ(OriginECEF);
    }



    /// <summary>
    /// Transforms the given geodetic Lat, Lon, Alt coordinate to a flat earth coordinate
    /// </summary>
    /// <param name="latlonalt">The Lat (X), Lon (Y), Alt (Z) coordinate to transform.</param>
    /// <returns>The X (East), Y (North), and Z (Up) location in a flat Earth coordinate system.</returns>
    private Vector3Double geodetic_to_flatearth(FLatLonAlt latlonalt)
    {
        latlonalt.Latitude = glm.Radians(latlonalt.Latitude);
        latlonalt.Longitude = glm.Radians(latlonalt.Longitude);

        //dlat and dlon are in Radians
        double dlat = latlonalt.Latitude - glm.Radians(OriginLLA.Latitude);
        double dlon = latlonalt.Longitude - glm.Radians(OriginLLA.Longitude);

        Vector3Double toReturn = new Vector3Double
        {
            X = (EARTH_GEOCENTRIC_RADIUS + latlonalt.Altitude) * dlon * Math.Cos(latlonalt.Latitude),
            Y = (EARTH_GEOCENTRIC_RADIUS + latlonalt.Altitude) * dlat,
            Z = latlonalt.Altitude - OriginLLA.Altitude
        };

        return toReturn;
    }

    /// <summary>
    /// Transforms the given flat Earth coordinates to geodetic Lat, Lon, Alt coordinates
    /// </summary>
    /// <param name="FlatEarthCoords">The flat Earth X (East), Y (North), and Z (Up) coordinate to transform.</param>
    /// <returns>The Lat (X), Lon (Y), Alt (Z) location in a geodetic coordinate system.</returns>
    private FLatLonAlt flatearth_to_geodetic(Vector3Double FlatEarthCoords)
    {
        //dlat and dlon are in Radians
        double dlat = FlatEarthCoords.Y / (EARTH_GEOCENTRIC_RADIUS + FlatEarthCoords.Z);
        double dlon = FlatEarthCoords.X / ((EARTH_GEOCENTRIC_RADIUS + FlatEarthCoords.Z) * Math.Cos(dlat + glm.Radians(OriginLLA.Latitude)));

        FLatLonAlt toReturn = new FLatLonAlt
        {
            Latitude = OriginLLA.Latitude + glm.Degrees(dlat),
            Longitude = OriginLLA.Longitude + glm.Degrees(dlon),
            Altitude = FlatEarthCoords.Z + OriginLLA.Altitude //Alt
        };

        return toReturn;
    }


    /// <summary>
    /// Transforms the given ECEF XYZ location to a flat Earth coordinate.
    /// </summary>
    /// <param name="ecefLocation">The ECEF location to transform.</param>
    /// <returns>The X (East), Y (North), and Z (Up) location in a flat Earth coordinate system.</returns>
    private Vector3Double ecef_to_flatearth(Vector3Double ecefLocation)
    {
        FLatLonAlt lla = Conversions.CalculateLatLonHeightFromEcefXYZ(ecefLocation);

        lla.Latitude = glm.Radians(lla.Latitude);
        lla.Longitude = glm.Radians(lla.Longitude);

        //dlat and dlon are in Radians
        double dlat = lla.Latitude - glm.Radians(OriginLLA.Latitude);
        double dlon = lla.Longitude - glm.Radians(OriginLLA.Longitude);

        Vector3Double toReturn = new Vector3Double
        {
            X = (EARTH_GEOCENTRIC_RADIUS + lla.Altitude) * dlon * Math.Cos(lla.Latitude),
            Y = (EARTH_GEOCENTRIC_RADIUS + lla.Altitude) * dlat,
            Z = lla.Altitude - OriginLLA.Altitude
        };

        return toReturn;
    }

    /// <summary>
    /// Transforms the given flat Earth coordinates to an ECEF XYZ coordinate
    /// </summary>
    /// <param name="FlatEarthCoords">The flat Earth X (East), Y (North), and Z (Up) coordinate to transform.</param>
    /// <returns>The ECEF XYZ location.</returns>
    private Vector3Double flatearth_to_ecef(Vector3Double FlatEarthCoords)
    {
        //dlat and dlon are in Radians
        double dlat = FlatEarthCoords.Y / (EARTH_GEOCENTRIC_RADIUS + FlatEarthCoords.Z);
        double dlon = FlatEarthCoords.X / ((EARTH_GEOCENTRIC_RADIUS + FlatEarthCoords.Z) * Math.Cos(dlat + glm.Radians(OriginLLA.Latitude)));

        FLatLonAlt lla = new FLatLonAlt
        {
            Latitude = glm.Radians(OriginLLA.Latitude) + dlat,
            Longitude = glm.Radians(OriginLLA.Longitude) + dlon,
            Altitude = FlatEarthCoords.Z + OriginLLA.Altitude
        };

        lla.Latitude = glm.Degrees(lla.Latitude);
        lla.Longitude = glm.Degrees(lla.Longitude);

        return Conversions.CalculateEcefXYZFromLatLonHeight(lla);
    }



    #endregion PrivateFunctions

}

#region OldCode

/*
 
    private Vector3Double geodetic_to_squareflatearth(Vector3Double latlonalt)
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
            X = (EARTH_GEOCENTRIC_RADIUS + lla.Z) * dlon, //East: possibly replace with minor axis
            Y = lla.Z - OriginAlt, //Up
            Z = (EARTH_GEOCENTRIC_RADIUS + lla.Z) * dlat //North: possibly replace with major axis
        };

        return toReturn;
    }

    private Vector3Double squareflatearth_to_geodetic(Vector3Double UnityCoords)
    {
        double dlat = (UnityCoords.Z / (EARTH_GEOCENTRIC_RADIUS + UnityCoords.Y));
        double dlon = (UnityCoords.X / (EARTH_GEOCENTRIC_RADIUS + UnityCoords.Y));

        Vector3Double toReturn = new Vector3Double
        {
            X = ((OriginLat * DEG_TO_RAD) + dlat) * RAD_TO_DEG, //Lat: accurate to the 6th decimal place, possibly replace with minor axis
            Y = ((OriginLon * DEG_TO_RAD) + dlon) * RAD_TO_DEG, //Lon: accurate to the 6th decimal place, possibly replace with major axis
            Z = UnityCoords.Y + OriginAlt //Alt
        };

        return toReturn;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="PsiThetaPhi"> Flat Earth PsiThetaPhi </param>
    /// <param name="LatLonAlt"> Flat Earth LatLongAlt </param>
    /// <returns> Psi Theta Phi in geocentric </returns>
    private Vector3Double FlatEarth2WGS_att(Vector3Double PsiThetaPhi, Vector3Double LatLonAlt)
    {
        Vector3Double lla = new Vector3Double
        {
            X = LatLonAlt.X * DEG_TO_RAD,
            Y = LatLonAlt.Y * DEG_TO_RAD,
            Z = LatLonAlt.Z
        };

        double sin_psi = Math.Sin(PsiThetaPhi.X);
        double cos_psi = Math.Cos(PsiThetaPhi.X);
        double sin_theta = Math.Sin(PsiThetaPhi.Y);
        double cos_theta = Math.Cos(PsiThetaPhi.Y);
        double sin_phi = Math.Sin(PsiThetaPhi.Z);
        double cos_phi = Math.Cos(PsiThetaPhi.Z);

        double sin_lat = Math.Sin(lla.X);
        double cos_lat = Math.Cos(lla.X);
        double sin_lon = Math.Sin(lla.Y);
        double cos_lon = Math.Cos(lla.Y);

        if (PsiThetaPhi.Y == 0.0 && PsiThetaPhi.Z == 0.0)
        {
            sin_theta = Math.Sin(0.00000000001);
            cos_theta = Math.Cos(0.00000000001);
        }


        double sin_gc_theta = cos_lat * cos_psi * cos_theta + sin_lat * sin_theta;
        double theta = Math.Asin(-sin_gc_theta);


        double sin_gc_psi = -sin_lat * sin_lon * cos_psi * cos_theta + cos_lon * sin_psi * cos_theta + cos_lat * sin_lon * sin_theta;
        double cos_gc_psi = -sin_lat * cos_lon * cos_psi * cos_theta - sin_lon * sin_psi * cos_theta + cos_lat * cos_lon * sin_theta;
        double psi;
        if (sin_gc_psi != 0.0 || cos_gc_psi != 0.0)
        {
            psi = Math.Atan2(sin_gc_psi, cos_gc_psi);
        }
        else
        {
            psi = 0.0;
        }


        double sin_gc_phi = cos_lat * (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi) - sin_lat * cos_theta * sin_phi;
        double cos_gc_phi = cos_lat * (cos_psi * sin_theta * cos_phi + sin_psi * cos_phi) - sin_lat * cos_theta * cos_phi;
        double phi;
        if (sin_gc_phi != 0.0 || cos_gc_phi != 0.0)
        {
            phi = Math.Atan2(sin_gc_phi, cos_gc_phi);
        }
        else
        {
            phi = 0.0;
        }

        return new Vector3Double { X = psi, Y = theta, Z = phi };
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="PsiThetaPhi"> WGS Psi Theta & Phi </param>
    /// <param name="LatLonAlt"> WGS Lat Lon & Alt </param>
    /// <returns> Flat Earth Psi Theta & Phi </returns>
    private Vector3Double WGS2FlatEarth_att(Vector3Double PsiThetaPhi, Vector3Double LatLonAlt)
    {
        Vector3Double lla = new Vector3Double
        {
            X = LatLonAlt.X * DEG_TO_RAD,
            Y = LatLonAlt.Y * DEG_TO_RAD,
            Z = LatLonAlt.Z
        };

        double sin_psi_gc = Math.Sin(PsiThetaPhi.X);
        double cos_psi_gc = Math.Cos(PsiThetaPhi.X);
        double sin_theta_gc = Math.Sin(PsiThetaPhi.Y);
        double cos_theta_gc = Math.Cos(PsiThetaPhi.Y);
        double sin_phi_gc = Math.Sin(PsiThetaPhi.Z);
        double cos_phi_gc = Math.Cos(PsiThetaPhi.Z);

        double sin_lat = Math.Sin(lla.X);
        double cos_lat = Math.Cos(lla.X);
        double sin_lon = Math.Sin(lla.Y);
        double cos_lon = Math.Cos(lla.Y);

        double sin_theta_local = cos_lat * cos_lon * cos_psi_gc * cos_theta_gc + cos_lat * sin_lon * sin_psi_gc * cos_theta_gc - sin_lat * sin_theta_gc;
        double theta = Math.Asin(sin_theta_local);

        double cos_psi_local = -sin_lat * cos_lon * cos_theta_gc * cos_psi_gc - sin_lat * sin_lon * cos_theta_gc * sin_psi_gc - cos_lat * sin_theta_gc;
        double sin_psi_local = -sin_lon * cos_theta_gc * cos_psi_gc - sin_lat * sin_lon * cos_theta_gc * sin_psi_gc - cos_lat * sin_theta_gc;
        double psi;
        if (sin_psi_local != 0.0 || cos_psi_local != 0.0)
        {
            psi = Math.Atan2(sin_psi_local, cos_psi_local);
        }
        else
        {
            psi = 0.0;
        }

        double sin_phi_local = cos_lat * cos_lon * (-cos_phi_gc * sin_psi_gc * sin_theta_gc * cos_psi_gc) + sin_lon * cos_lat * (cos_phi_gc * cos_psi_gc + sin_phi_gc * sin_theta_gc * sin_psi_gc) + sin_lat * sin_phi_gc * cos_theta_gc;
        double cos_phi_local = cos_lat * cos_lon * (sin_phi_gc * sin_psi_gc + cos_phi_gc * sin_theta_gc * sin_psi_gc) + sin_lon * cos_lat * (-sin_phi_gc * cos_psi_gc + cos_phi_gc * sin_theta_gc * sin_psi_gc) + sin_lat * cos_phi_gc * cos_theta_gc;
        double phi;
        if (sin_phi_local != 0.0 || cos_phi_local != 0.0)
        {
            phi = Math.Atan2(-sin_phi_local, -cos_phi_local);
        }
        else
        {
            phi = 0.0;
        }

        return new Vector3Double
        {
            X = psi,
            Y = theta,
            Z = phi
        };
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="Velocity"> Flat Earth Velocity </param>
    /// <param name="LatLonAlt"> OBJ Lat Lon & Alt </param>
    /// <returns> WGS Velocity </returns>
    private Vector3Double FlatEarth2WGS_vect(Vector3Double Velocity, Vector3Double LatLonAlt)
    {
        Vector3Double lla = new Vector3Double
        {
            X = LatLonAlt.X * DEG_TO_RAD,
            Y = LatLonAlt.Y * DEG_TO_RAD,
            Z = LatLonAlt.Z
        };

        double sin_lat = Math.Sin(lla.X);
        double cos_lat = Math.Cos(lla.X);
        double sin_lon = Math.Sin(lla.Y);
        double cos_lon = Math.Cos(lla.Y);

        double x = -sin_lat * cos_lon * Velocity.Y - sin_lon * Velocity.X + cos_lat * cos_lon * Velocity.Z;
        double y = -sin_lat * sin_lon * Velocity.Y + cos_lon * Velocity.X + cos_lat * sin_lon * Velocity.Z;
        double z = cos_lat * Velocity.Y + sin_lat * Velocity.Z;

        return new Vector3Double { X = x, Y = y, Z = z };
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="Velocity"> WGS Velocity </param>
    /// <param name="LatLonAlt"> OBJ Lat Lon & Alw </param>
    /// <returns> Flat Earth Velocity </returns>
    private Vector3Double WGS2FlatEarth_vect(Vector3Double Velocity, Vector3Double LatLonAlt)
    {
        Vector3Double lla = new Vector3Double
        {
            X = LatLonAlt.X * DEG_TO_RAD,
            Y = LatLonAlt.Y * DEG_TO_RAD,
            Z = LatLonAlt.Z
        };

        double sin_lat = Math.Sin(lla.X);
        double cos_lat = Math.Cos(lla.X);
        double sin_lon = Math.Sin(lla.Y);
        double cos_lon = Math.Cos(lla.Y);

        double x = -sin_lon * Velocity.X + cos_lon * Velocity.Y;
        double y = -sin_lat * cos_lon * Velocity.X - sin_lat * sin_lon * Velocity.Y + cos_lat * Velocity.Z;
        double z = cos_lat * cos_lon * Velocity.X + cos_lat * sin_lon * Velocity.Y + sin_lat * Velocity.Z;

        return new Vector3Double { X = x, Y = y, Z = z };
    }

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