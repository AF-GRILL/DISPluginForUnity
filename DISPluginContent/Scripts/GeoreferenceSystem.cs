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
//***    The current rotines mainly make it possible to translate
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
    //[Tooltip("The Altitudinal equivalent for 0 in Unity's coordinate system.")]
    //public double OriginAlt = 0;

    private Vector3Double OriginLLA;
    private Vector3Double OriginECEF;
    private Vector3Double temp;
    private Vector3Double tempLLA;
    // Start is called before the first frame update
    void Awake()
    {
        SetupVars();

        #region Debugging
        temp = ecef_to_flatearth(OriginECEF);
        Debug.Log("ECEFToUnity: " + temp.X + ", " + temp.Y + ", " + temp.Z);

        temp = geodetic_to_flatearth(new Vector3Double { X = 37.4720128, Y = -115.5050288, Z= 1420});
        Debug.Log("LLAToUnity: " + temp.X + ", " + temp.Y + ", " + temp.Z);
        //StartCoroutine(LoopCalc());

        #endregion Debugging
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
            tempLLA = Conversions.CalculateLatLonHeightFromEcefXYZ(temp);
            //Debug.Log("UnityToECEF " + i + ": " + tempLLA.X + ", " + tempLLA.Y + ", " + tempLLA.Z);
            temp = ECEFToUnity(temp);
            i += 1;
            yield return new WaitForSeconds(2.5f);
        }
    }


    #endregion DebuggingMethods

    #region PublicFunctions

    public Vector3Double LatLonAltToUnity(Vector3Double LatLonAlt)
    {
        return geodetic_to_flatearth(LatLonAlt);
    }

    public Vector3Double ECEFToUnity(Vector3Double ECEF)
    {
        return ecef_to_flatearth(ECEF);
    }

    public Vector3Double UnityToLatLonAlt(Vector3 UnityCoords)
    {
        Vector3Double unityCoordsDouble = new Vector3Double
        {
            X = UnityCoords.x,
            Y = UnityCoords.y,
            Z = UnityCoords.z
        };

        return flatearth_to_geodetic(unityCoordsDouble);
    }

    public Vector3Double UnityToECEF(Vector3 UnityCoords)
    {
        Vector3Double unityCoordsDouble = new Vector3Double
        {
            X = UnityCoords.x,
            Y = UnityCoords.y,
            Z = UnityCoords.z
        };

        return flatearth_to_ecef(unityCoordsDouble);
    }

    public FNorthEastDown GetNEDVectorsAtEngineLocation(Vector3 UnityLocation)
    {
        Vector3Double ecefLoc = UnityToECEF(UnityLocation);
        Vector3Double lla = Conversions.CalculateLatLonHeightFromEcefXYZ(ecefLoc);
        return Conversions.CalculateNorthEastDownVectorsFromLatLon(lla.X, lla.Y);
    }

    public FEastNorthUp GetENUVectorsAtEngineLocation(Vector3 UnityLocation)
    {
        Vector3Double ecefLoc = UnityToECEF(UnityLocation);
        Vector3Double lla = Conversions.CalculateLatLonHeightFromEcefXYZ(ecefLoc);
        FNorthEastDown northEastDownVectors = Conversions.CalculateNorthEastDownVectorsFromLatLon(lla.X, lla.Y);
        return new FEastNorthUp(northEastDownVectors.EastVector, northEastDownVectors.NorthVector, -northEastDownVectors.DownVector);
    }

    public Vector3Double GetOriginLLA() { return OriginLLA; }
    public Vector3Double GetOriginECEF() { return OriginECEF; }

    #endregion PublicFunctions

    #region PrivateFunctions

    private void SetupVars()
    {
        OriginLLA = new Vector3Double
        {
            X = OriginLat,
            Y = OriginLon,
            Z = 0
        };

        OriginECEF = Conversions.CalculateEcefXYZFromLatLonHeight(OriginLLA);
    }



    /// <summary>
    /// 
    /// </summary>
    /// <param name="latlonalt"></param>
    /// <returns></returns>
    private Vector3Double geodetic_to_flatearth(Vector3Double latlonalt)
    {
        //angle between d = difference calc and convert
        Vector3Double lla = new Vector3Double
        {
            X = latlonalt.X * Math.PI / 180.0,
            Y = latlonalt.Y * Math.PI / 180.0,
            Z = latlonalt.Z,
        };

        double dlat = lla.X - (OriginLat * Math.PI / 180.0);
        double dlon = lla.Y - (OriginLon * Math.PI / 180.0);

        Vector3Double toReturn = new Vector3Double
        {
            X = (EARTH_GEOCENTRIC_RADIUS + lla.Z) * dlon * Math.Cos(lla.X),
            Y = lla.Z - OriginLLA.Z,
            Z = (EARTH_GEOCENTRIC_RADIUS + lla.Z) * dlat
        };

        return toReturn;
    }

    private Vector3Double flatearth_to_geodetic(Vector3Double UnityCoords)
    {
        double dlat = (UnityCoords.Z / (EARTH_GEOCENTRIC_RADIUS + UnityCoords.Y));
        double dlon = (UnityCoords.X / ((EARTH_GEOCENTRIC_RADIUS + UnityCoords.Y) * Math.Cos(dlat + (OriginLat * Math.PI / 180.0))));

        Vector3Double toReturn = new Vector3Double
        {
            X = ((OriginLat * Math.PI / 180.0) + dlat) * 180.0 / Math.PI,
            Y = ((OriginLon * Math.PI / 180.0) + dlon) * 180.0 / Math.PI,
            Z = UnityCoords.Y + OriginLLA.Z //Alt
        };

        return toReturn;
    }



    private Vector3Double ecef_to_flatearth(Vector3Double ecefLocation)
    {
        double earthSemiMajorRadiusMeters = 6378137;
        double earthSemiMinorRadiusMeters = 6356752.3142;

        double earthSemiMajorRadiusMetersSquare = Math.Pow(earthSemiMajorRadiusMeters, 2);
        double earthSemiMinorRadiusMetersSquare = Math.Pow(earthSemiMinorRadiusMeters, 2);
        double distFromXToY = Math.Sqrt(Math.Pow(ecefLocation.X, 2) + Math.Pow(ecefLocation.Y, 2));

        double longitude = Math.Atan2(ecefLocation.Y, ecefLocation.X);
        double latitude = Math.Atan(earthSemiMajorRadiusMetersSquare / earthSemiMinorRadiusMetersSquare * (ecefLocation.Z / distFromXToY));
        double cosLatitude = Math.Cos(latitude);
        double sinLatitude = Math.Sin(latitude);
        double height = (distFromXToY / cosLatitude) - (earthSemiMajorRadiusMetersSquare / Math.Sqrt((earthSemiMajorRadiusMetersSquare * Math.Pow(cosLatitude, 2)) + (earthSemiMinorRadiusMetersSquare * Math.Pow(sinLatitude, 2))));
        
        double dlat = latitude - glm.Radians(OriginLat);
        double dlon = longitude - glm.Radians(OriginLon);

        Vector3Double toReturn = new Vector3Double
        {
            X = (EARTH_GEOCENTRIC_RADIUS + height) * dlon * Math.Cos(latitude),
            Y = height - OriginLLA.Z,
            Z = (EARTH_GEOCENTRIC_RADIUS + height) * dlat
        };

        return toReturn;
    }

    private Vector3Double flatearth_to_ecef(Vector3Double UnityCoords)
    {
        double dlat = (UnityCoords.Z / (EARTH_GEOCENTRIC_RADIUS + UnityCoords.Y));
        double dlon = (UnityCoords.X / ((EARTH_GEOCENTRIC_RADIUS + UnityCoords.Y) * Math.Cos(dlat + glm.Radians(OriginLat))));

        Vector3Double lla = new Vector3Double
        {
            X = (glm.Radians(OriginLat) + dlat),
            Y = (glm.Radians(OriginLon) + dlon),
            Z = UnityCoords.Y + OriginLLA.Z
        };

        double earthSemiMajorRadiusMeters = 6378137;
        double earthSemiMinorRadiusMeters = 6356752.3142;

        double cosLatitude = Math.Cos(lla.X);
        double sinLatitude = Math.Sin(lla.X);
        double cosLongitude = Math.Cos(lla.Y);
        double sinLongitude = Math.Sin(lla.Y);

        double XYBaseConversion = (earthSemiMajorRadiusMeters / (Math.Sqrt(Math.Pow(cosLatitude, 2) + ((Math.Pow(earthSemiMinorRadiusMeters, 2) / Math.Pow(earthSemiMajorRadiusMeters, 2)) * Math.Pow(sinLatitude, 2))))) + lla.Z;
        double ZBaseConversion = (earthSemiMinorRadiusMeters / (((Math.Sqrt(Math.Pow(cosLatitude, 2) * (Math.Pow(earthSemiMajorRadiusMeters, 2) / Math.Pow(earthSemiMinorRadiusMeters, 2)) + Math.Pow(sinLatitude, 2)))))) + lla.Z;

        return new Vector3Double
        {
            X = XYBaseConversion * cosLatitude * cosLongitude,
            Y = XYBaseConversion * cosLatitude * sinLongitude,
            Z = ZBaseConversion * sinLatitude
        };

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