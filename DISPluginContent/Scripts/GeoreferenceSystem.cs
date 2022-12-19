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

namespace GRILLDIS
{
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
                temp = UnityFlatearthToECEF(unityLoc);
                FLatLonAlt tempLLA = Conversions.CalculateLatLonHeightFromEcefXYZ(temp);
                //Debug.Log("UnityToECEF " + i + ": " + tempLLA.Latitude + ", " + tempLLA.Longitude + ", " + tempLLA.Altitude);
                temp = ECEFToUnityFlatearth(temp);
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
        public Vector3Double LatLonAltToUnityFlatearth(FLatLonAlt LatLonAlt)
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
        public Vector3Double ECEFToUnityFlatearth(Vector3Double ECEF)
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
        public FLatLonAlt UnityFlatearthToLatLonAlt(Vector3 UnityCoords)
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
        public Vector3Double UnityFlatearthToECEF(Vector3 UnityCoords)
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
            Vector3Double ecefLoc = UnityFlatearthToECEF(UnityLocation);
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
            Vector3Double ecefLoc = UnityFlatearthToECEF(UnityLocation);
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
}