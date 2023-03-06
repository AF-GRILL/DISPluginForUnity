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
        const double EPSILON = 2.220446049250313e-16;
        const double EARTH_MAJOR_AXIS = 6378137;
        const double EARTH_MINOR_AXIS = 6356752.314245;

        /// <summary>
        /// The Latitude in decimal degrees, Longitude in decimal degrees, and Altitude in meters of the Unity origin (0, 0, 0).
        /// </summary>
        [Header("DIS Georeference Settings")]
        [Tooltip("The shape of the world in the scene.")]
        public EEarthShape EarthShape;
        [Tooltip("The Latitude in decimal degrees, Longitude in decimal degrees, and Altitude in meters of the Unity origin (0, 0, 0).")]
        public FLatLonAlt OriginLLA;

        private Vector3Double OriginECEF;
        private dmat4 WorldFrameToECEFFrame;
        private dmat4 ECEFFrameToWorldFrame;
        // Start is called before the first frame update
        void Awake()
        {
            SetupVars();
        }

        #region PublicFunctions

        /// <summary>
        /// Converts ECEF coordinates to Unity coordinates.
        /// </summary>
        /// <param name="ECEFLocation">The ECEF location to convert to Unity coordinates</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        /// <returns>The Unity coordinates.</returns>
        public Vector3 ECEFToUnity(Vector3Double ECEFLocation, Vector3 OriginRebasingOffset = default)
        {
            Vector3 unityCoords = Vector3.zero;

            switch (EarthShape)
            {
                case EEarthShape.RoundEarth:
                    {
                        dmat4 ecefLocationToTransform = new dmat4(new dvec4(ECEFLocation.X, 0, 0, 0),
                            new dvec4(ECEFLocation.Y, 0, 0, 0),
                            new dvec4(ECEFLocation.Z, 0, 0, 0),
                            new dvec4(1, 0, 0, 0));

                        dmat4 transformedLocation = ecefLocationToTransform * ECEFFrameToWorldFrame;
                        //Convert to Unity coords
                        unityCoords = new Vector3
                        {
                            x = (float)transformedLocation.m00,
                            y = (float)transformedLocation.m20,
                            z = (float)transformedLocation.m10
                        };

                        break;
                    }
                case EEarthShape.FlatEarth:
                    {
                        Vector3Double flatEarthLoc = ecef_to_flatearth(ECEFLocation);

                        //Transform the flat Earth ECEF location to be in terms of Unity coordinates
                        unityCoords = new Vector3
                        {
                            x = (float)flatEarthLoc.X,
                            y = (float)flatEarthLoc.Z,
                            z = (float)flatEarthLoc.Y
                        };

                        break;
                    }
            }

            // TODO: Find a better way to do origin rebasing rather than passing in a value as it has a cascading effect. Would be nice to be able to get the offset in the function.
            // Unreal Engine has a built in way of accessing origin offset, but does Unity?...
            Vector3 rebasedUnityLocation = unityCoords - OriginRebasingOffset;
            return rebasedUnityLocation;
        }

        /// <summary>
        /// Converts Unity coordinates into ECEF coordinates.
        /// </summary>
        /// <param name="UnityLocation">The Unity coordinates to transform.</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        /// <returns>The ECEF XYZ coordinates.</returns>
        public Vector3Double UnityToECEF(Vector3 UnityLocation, Vector3 OriginRebasingOffset = default)
        {
            Vector3Double ecefCoords = new Vector3Double();
            // TODO: Find a better way to do origin rebasing rather than passing in a value as it has a cascading effect. Would be nice to be able to get the offset in the function.
            // Unreal Engine has a built in way of accessing origin offset, but does Unity?...
            Vector3 rebasedUnityLocation = UnityLocation + OriginRebasingOffset;

            switch (EarthShape)
            {
                case EEarthShape.RoundEarth:
                    {
                        //Place unity location into double vector and make relative to ECEF coordinate system
                        dmat4 unityLocationToTransform = new dmat4(new dvec4(rebasedUnityLocation.x, 0, 0, 0),
                            new dvec4(rebasedUnityLocation.z, 0, 0, 0),
                            new dvec4(rebasedUnityLocation.y, 0, 0, 0),
                            new dvec4(1, 0, 0, 0));

                        dmat4 transformedLocation = unityLocationToTransform * WorldFrameToECEFFrame;
                        ecefCoords = new Vector3Double
                        {
                            X = transformedLocation.m00,
                            Y = transformedLocation.m10,
                            Z = transformedLocation.m20
                        };

                        break;
                    }
                case EEarthShape.FlatEarth:
                    {
                        //Transform the given Unity coordinates to be in terms of a flat Earth coordinate system
                        Vector3Double flatEarthCoords = new Vector3Double
                        {
                            X = rebasedUnityLocation.x,
                            Y = rebasedUnityLocation.z,
                            Z = rebasedUnityLocation.y
                        };

                        ecefCoords = flatearth_to_ecef(flatEarthCoords);
                        break;
                    }
            }

            return ecefCoords;
        }

        /// <summary>
        /// Converts the given Lat, Lon, Alt coordinates to Unity coordinates.
        /// </summary>
        /// <param name="LatLonAlt">The Lat, Lon, Alt coordinates to transform.</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        /// <returns>The Unity coordinates.</returns>
        public Vector3 LatLonAltToUnity(FLatLonAlt LatLonAlt, Vector3 OriginRebasingOffset = default)
        {
            Vector3 unityCoords = Vector3.zero;

            switch (EarthShape) 
            {
                case EEarthShape.RoundEarth:
                    {
                        Vector3Double ecef = Conversions.CalculateEcefXYZFromLatLonHeight(LatLonAlt);
                        unityCoords = ECEFToUnity(ecef, OriginRebasingOffset);
                        break;
                    }
                case EEarthShape.FlatEarth:
                    {
                        Vector3Double flatEarthLoc = geodetic_to_flatearth(LatLonAlt);

                        //Transform the flat Earth LLA location to be in terms of Unity coordinates
                        unityCoords = new Vector3
                        {
                            x = (float)flatEarthLoc.X,
                            y = (float)flatEarthLoc.Z,
                            z = (float)flatEarthLoc.Y
                        };

                        // TODO: Find a better way to do origin rebasing rather than passing in a value as it has a cascading effect. Would be nice to be able to get the offset in the function.
                        // Unreal Engine has a built in way of accessing origin offset, but does Unity?...
                        unityCoords -= OriginRebasingOffset;
                        break;
                    }
            }

            return unityCoords;
        }

        /// <summary>
        /// Converts Unity coordinates into geodetic Lat, Lon, Alt coordinates.
        /// </summary>
        /// <param name="UnityLocation">The Unity coordinates to transform.</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        /// <returns>The Lat, Lon, Alt coordinates.</returns>
        public FLatLonAlt UnityToLatLonAlt(Vector3 UnityLocation, Vector3 OriginRebasingOffset = default)
        {
            FLatLonAlt llaCoords = new FLatLonAlt();

            switch (EarthShape)
            {
                case EEarthShape.RoundEarth:
                    {
                        Vector3Double ecef = UnityToECEF(UnityLocation, OriginRebasingOffset);
                        llaCoords = Conversions.CalculateLatLonHeightFromEcefXYZ(ecef);
                        break;
                    }
                case EEarthShape.FlatEarth:
                    {
                        // TODO: Find a better way to do origin rebasing rather than passing in a value as it has a cascading effect. Would be nice to be able to get the offset in the function.
                        // Unreal Engine has a built in way of accessing origin offset, but does Unity?...
                        UnityLocation += OriginRebasingOffset;

                        //Transform the given Unity coordinates to be in terms of a flat Earth coordinate system
                        Vector3Double flatEarthCoords = new Vector3Double
                        {
                            X = UnityLocation.x,
                            Y = UnityLocation.z,
                            Z = UnityLocation.y
                        };

                        llaCoords = flatearth_to_geodetic(flatEarthCoords);
                        break;
                    }
            }

            return llaCoords;
        }

        /// <summary>
        /// Get the North, East, Down vectors at the given Unity location.
        /// </summary>
        /// <param name="UnityLocation">The Unity location to get the North, East, Down vectors of.</param>
        /// <returns>The North, East, Down vectors.</returns>
        public FNorthEastDown GetNEDVectorsAtEngineLocation(Vector3 UnityLocation, Vector3 OriginRebasingOffset = default)
        {
            Vector3Double ecefLoc = UnityToECEF(UnityLocation, OriginRebasingOffset);
            FLatLonAlt lla = Conversions.CalculateLatLonHeightFromEcefXYZ(ecefLoc);
            return Conversions.CalculateNorthEastDownVectorsFromLatLon(lla.Latitude, lla.Longitude);
        }

        /// <summary>
        /// Get the East, North, Up vectors at the given Unity location.
        /// </summary>
        /// <param name="UnityLocation">The Unity location to get the East, North, Up vectors of.</param>
        /// <returns>The East, North, Up vectors.</returns>
        public FEastNorthUp GetENUVectorsAtEngineLocation(Vector3 UnityLocation, Vector3 OriginRebasingOffset = default)
        {
            Vector3Double ecefLoc = UnityToECEF(UnityLocation, OriginRebasingOffset);
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

            WorldFrameToECEFFrame = GetWorldFrameToECEFFrame();
            ECEFFrameToWorldFrame = WorldFrameToECEFFrame.Inverse;
        }

        private dmat4 GetWorldFrameToECEFFrame()
        {
            Vector3Double GeographicEllipsoid = new Vector3Double
            {
                X = EARTH_MAJOR_AXIS,
                Y = EARTH_MAJOR_AXIS,
                Z = EARTH_MINOR_AXIS
            };

            // See ECEF standard : https://commons.wikimedia.org/wiki/File:ECEF_ENU_Longitude_Latitude_right-hand-rule.svg
            if (Math.Abs(OriginECEF.X) < EPSILON &&
                Math.Abs(OriginECEF.Y) < EPSILON)
            {
                // Special Case - On earth axis... 
                double Sign = 1.0;
                if (Math.Abs(OriginECEF.Z) < EPSILON)
                {
                    // At origin - Should not happen, but consider it's the same as north pole
                    // Leave Sign = 1
                }
                else
                {
                    // At South or North pole - Axis are set to be continuous with other points
                    Sign = (OriginECEF.Z < 0) ? -1 : 1;
                }

                return new dmat4(
                    new dvec4(1, 0, 0, OriginECEF.X),           // East = X
                    new dvec4(0, 0, 1 * Sign, OriginECEF.Y),   // North = Sign * Z
                    new dvec4(0, -1 * Sign, 0, OriginECEF.Z),    // Up = Sign*Y
                    new dvec4(0, 0, 0, 1));
            }
            else
            {
                //Calculate the North, East, Up vectors of the origin and create a transform matrix from them
                Vector3Double Up = new Vector3Double
                {
                    X = OriginECEF.X * (1 / Math.Pow(GeographicEllipsoid.X, 2)),
                    Y = OriginECEF.Y * (1 / Math.Pow(GeographicEllipsoid.Y, 2)),
                    Z = OriginECEF.Z * (1 / Math.Pow(GeographicEllipsoid.Z, 2))
                };
                double UpMagnitude = Math.Sqrt(Math.Pow(Up.X, 2) + Math.Pow(Up.Y, 2) + Math.Pow(Up.Z, 2));
                Up = new Vector3Double
                {
                    X = Up.X / UpMagnitude,
                    Y = Up.Y / UpMagnitude,
                    Z = Up.Z / UpMagnitude
                };

                Vector3Double East = new Vector3Double
                {
                    X = -OriginECEF.Y,
                    Y = OriginECEF.X,
                    Z = 0
                };
                double EastMagnitude = Math.Sqrt(Math.Pow(East.X, 2) + Math.Pow(East.Y, 2) + Math.Pow(East.Z, 2));
                East = new Vector3Double
                {
                    X = East.X / EastMagnitude,
                    Y = East.Y / EastMagnitude,
                    Z = East.Z / EastMagnitude
                };

                Vector3Double North = new Vector3Double
                {
                    X = Up.Y * East.Z - Up.Z * East.Y,
                    Y = Up.Z * East.X - Up.X * East.Z,
                    Z = Up.X * East.Y - Up.Y * East.X
                };

                return new dmat4(new dvec4(East.X, North.X, Up.X, OriginECEF.X),
                    new dvec4(East.Y, North.Y, Up.Y, OriginECEF.Y),
                    new dvec4(East.Z, North.Z, Up.Z, OriginECEF.Z),
                    new dvec4(0, 0, 0, 1));
            }
        }

        /// <summary>
        /// Transforms the given geodetic Lat, Lon, Alt coordinate to a flat earth coordinate
        /// </summary>
        /// <param name="latlonalt">The Lat, Lon, Alt coordinate to transform.</param>
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
                X = (EARTH_MAJOR_AXIS + latlonalt.Altitude) * dlon * Math.Cos(latlonalt.Latitude),
                Y = (EARTH_MAJOR_AXIS + latlonalt.Altitude) * dlat,
                Z = latlonalt.Altitude - OriginLLA.Altitude
            };

            return toReturn;
        }

        /// <summary>
        /// Transforms the given flat Earth coordinates to geodetic Lat, Lon, Alt coordinates
        /// </summary>
        /// <param name="FlatEarthCoords">The flat Earth X (East), Y (North), and Z (Up) coordinate to transform.</param>
        /// <returns>The Lat, Lon, Alt location in a geodetic coordinate system.</returns>
        private FLatLonAlt flatearth_to_geodetic(Vector3Double FlatEarthCoords)
        {
            //dlat and dlon are in Radians
            double dlat = FlatEarthCoords.Y / (EARTH_MAJOR_AXIS + FlatEarthCoords.Z);
            double dlon = FlatEarthCoords.X / ((EARTH_MAJOR_AXIS + FlatEarthCoords.Z) * Math.Cos(dlat + glm.Radians(OriginLLA.Latitude)));

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
                X = (EARTH_MAJOR_AXIS + lla.Altitude) * dlon * Math.Cos(lla.Latitude),
                Y = (EARTH_MAJOR_AXIS + lla.Altitude) * dlat,
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
            double dlat = FlatEarthCoords.Y / (EARTH_MAJOR_AXIS + FlatEarthCoords.Z);
            double dlon = FlatEarthCoords.X / ((EARTH_MAJOR_AXIS + FlatEarthCoords.Z) * Math.Cos(dlat + glm.Radians(OriginLLA.Latitude)));

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