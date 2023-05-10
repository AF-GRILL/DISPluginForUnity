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
using System.Collections.Generic;

namespace GRILLDIS
{
    public class GeoreferenceSystem : MonoBehaviour
    {
        const double EPSILON = 2.220446049250313e-16;
        const double EARTH_MAJOR_AXIS = 6378137;
        const double EARTH_MINOR_AXIS = 6356752.314245;
        const double EARTH_FLATTENING = 298.257223563;

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
        private dmat4 UnityFrameToWorldFrame;
        private dmat4 WorldFrameToUnityFrame;
        private Vector3Double GeographicEllipsoid = new Vector3Double
        {
            X = EARTH_MAJOR_AXIS,
            Y = EARTH_MAJOR_AXIS,
            Z = EARTH_MINOR_AXIS
        };

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
        /// Converts the given Lat and Lon to UTM coordinates. Lat and Lon should be in decimal degrees.
        /// </summary>
        /// <param name="LatLonAltToConvert">The Lat and Lon to convert</param>
        /// <returns>The UTM Coordinates.</returns>
        public FUTMCoordinates LatLonAltToProjected(FLatLonAlt LatLonAltToConvert)
        {
            //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
            //East Longitudes are positive, West longitudes are negative. 
            //North latitudes are positive, South latitudes are negative
            //Lat and Long are in decimal degrees
            //Written by Chuck Gantz- chuck.gantz@globalstar.com
            //http://www.gpsy.com/gpsinfo/geotoutm/

            FUTMCoordinates utmCoordinates = new FUTMCoordinates();

            double lon = LatLonAltToConvert.Longitude;
            double lat = LatLonAltToConvert.Latitude;

            //Make sure the longitude is between -180.00 .. 179.9
            double LongTemp = (lon + 180) - (int)((lon + 180) / 360) * 360 - 180; // -180.00 .. 179.9;

            int ZoneNumber = (int)((LongTemp + 180) / 6) + 1;

            if (lat >= 56.0 && lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
            {
                ZoneNumber = 32;
            }

            // Special zones for Svalbard
            if (lat >= 72.0 && lat < 84.0)
            {
                if (LongTemp >= 0.0 && LongTemp < 9.0)
                {
                    ZoneNumber = 31;
                }
                else if (LongTemp >= 9.0 && LongTemp < 21.0)
                {
                    ZoneNumber = 33;
                }
                else if (LongTemp >= 21.0 && LongTemp < 33.0)
                {
                    ZoneNumber = 35;
                }
                else if (LongTemp >= 33.0 && LongTemp < 42.0)
                {
                    ZoneNumber = 37;
                }
            }

            //compute the UTM Zone from the latitude and longitude
            utmCoordinates.Zone = ZoneNumber.ToString() + GetUTMLetterDesignator(lat);

            double k0 = 0.9996;
            double eccSquared = (Math.Pow(EARTH_MAJOR_AXIS, 2) - Math.Pow(EARTH_MINOR_AXIS, 2)) / Math.Pow(EARTH_MAJOR_AXIS, 2);

            double LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone
            double LongOriginRad = glm.Radians(LongOrigin);

            double LatRad = glm.Radians(lat);
            double LongRad = glm.Radians(LongTemp);

            double eccPrimeSquared = (eccSquared) / (1 - eccSquared);

            double N = EARTH_MAJOR_AXIS / Math.Sqrt(1 - eccSquared * Math.Sin(LatRad) * Math.Sin(LatRad));
            double T = Math.Tan(LatRad) * Math.Tan(LatRad);
            double C = eccPrimeSquared * Math.Cos(LatRad) * Math.Cos(LatRad);
            double A = Math.Cos(LatRad) * (LongRad - LongOriginRad);

            double M = EARTH_MAJOR_AXIS * ((1 - eccSquared / 4 - 3 * Math.Pow(eccSquared, 2) / 64 - 5 * Math.Pow(eccSquared, 3) / 256) * LatRad
                - (3 * eccSquared / 8 + 3 * Math.Pow(eccSquared, 2) / 32 + 45 * Math.Pow(eccSquared, 3) / 1024) * Math.Sin(2 * LatRad)
                + (15 * Math.Pow(eccSquared, 2) / 256 + 45 * Math.Pow(eccSquared, 3) / 1024) * Math.Sin(4 * LatRad)
                - (35 * Math.Pow(eccSquared, 3) / 3072) * Math.Sin(6 * LatRad));

            utmCoordinates.Easting = (double)(k0 * N * (A + (1 - T + C) * Math.Pow(A, 3) / 6
                + (5 - 18 * T + Math.Pow(T, 2) + 72 * C - 58 * eccPrimeSquared) * Math.Pow(A, 5) / 120) + 500000.0);

            double UTMNorthing = (double)(k0 * (M + N * Math.Tan(LatRad) * (Math.Pow(A, 2) / 2 + (5 - T + 9 * C + 4 * Math.Pow(C, 2)) * Math.Pow(A, 4) / 24
                + (61 - 58 * T + Math.Pow(T, 2) + 600 * C - 330 * eccPrimeSquared) * Math.Pow(A, 6) / 720)));

            if (lat < 0)
            {
                UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
            }

            utmCoordinates.Northing = UTMNorthing;

            return utmCoordinates;
        }

        /// <summary>
        /// Get the North, East, Down vectors at the given Unity location.
        /// </summary>
        /// <param name="UnityLocation">The Unity location to get the North, East, Down vectors of.</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        /// <returns>The North, East, Down vectors in terms of Unity's coordinate system.</returns>
        public FNorthEastDown GetNEDVectorsAtEngineLocation(Vector3 UnityLocation, Vector3 OriginRebasingOffset = default)
        {
            FEastNorthUp enuVectors = GetENUVectorsAtEngineLocation(UnityLocation, OriginRebasingOffset);
            return new FNorthEastDown(enuVectors.NorthVector, enuVectors.EastVector, -enuVectors.UpVector);
        }

        /// <summary>
        /// Get the North, East, Down vectors at the given ECEF location.
        /// </summary>
        /// <param name="ECEFLocation">The ECEF location to get the North, East, Down vectors of.</param>
        /// <returns>The North, East, Down vectors in terms of Unity's coordinate system</returns>
        public FNorthEastDown GetNEDVectorsAtECEFLocation(Vector3Double ECEFLocation)
        {
            FEastNorthUp enuVectors = GetENUVectorsAtECEFLocation(ECEFLocation);
            return new FNorthEastDown(enuVectors.NorthVector, enuVectors.EastVector, -enuVectors.UpVector);
        }

        /// <summary>
        /// Get the East, North, Up vectors at the given Unity location.
        /// </summary>
        /// <param name="UnityLocation">The Unity location to get the East, North, Up vectors of.</param>
        /// <param name="OriginRebasingOffset">The offset that has been applied to the origin if any origin shifting has been performed.</param>
        /// <returns>The East, North, Up vectors in terms of Unity's coordinate system.</returns>
        public FEastNorthUp GetENUVectorsAtEngineLocation(Vector3 UnityLocation, Vector3 OriginRebasingOffset = default)
        {
            Vector3Double ecefLocation = UnityToECEF(UnityLocation, OriginRebasingOffset);
            return GetENUVectorsAtECEFLocation(ecefLocation);
        }

        /// <summary>
        /// Get the East, North, Up vectors at the given ECEF location.
        /// </summary>
        /// <param name="ECEFLocation">The ECEF location to get the East, North, Up vectors of.</param>
        /// <returns>The East, North, Up vectors in terms of Unity's coordinate system</returns>
        public FEastNorthUp GetENUVectorsAtECEFLocation(Vector3Double ECEFLocation)
        {
            FEastNorthUp ENU = new FEastNorthUp();
            dmat4 WorldFrameToECEFFrameAtLocation = GetWorldFrameToECEFFrame(GeographicEllipsoid, OriginECEF);
            switch (EarthShape)
            {
                case EEarthShape.RoundEarth:
                    {
                        dmat4 UnityToECEF = WorldFrameToECEFFrameAtLocation * ECEFFrameToWorldFrame * UnityFrameToWorldFrame;

                        //Put in terms of Unity coordinate system
                        dvec4 East = UnityToECEF.Column0;
                        dvec4 North = UnityToECEF.Column2;
                        dvec4 Up = UnityToECEF.Column1;

                        ENU.EastVector = new Vector3((float)East.x, (float)East.y, (float)East.z);
                        ENU.NorthVector = new Vector3((float)North.x, (float)North.y, (float)North.z);
                        ENU.UpVector = new Vector3((float)Up.x, (float)Up.y, (float)Up.z);

                        break;
                    }
                case EEarthShape.FlatEarth:
                    {
                        dvec4 ecefLocationToTransform = new dvec4(ECEFLocation.X, ECEFLocation.Y, ECEFLocation.Z, 1);

                        dmat4 eastUnitVector = new dmat4(
                            dvec4.UnitX,
                            dvec4.Zero,
                            dvec4.Zero,
                            dvec4.Zero);
                        dmat4 northUnitVector = new dmat4(
                            dvec4.Zero,
                            dvec4.UnitX,
                            dvec4.Zero,
                            dvec4.Zero);

                        dmat4 transformedEastUnitVector = eastUnitVector * WorldFrameToECEFFrameAtLocation;
                        dmat4 transformedNorthUnitVector = northUnitVector * WorldFrameToECEFFrameAtLocation;

                        dvec4 easternPoint = ecefLocationToTransform + transformedEastUnitVector.Row0;
                        dvec4 northernPoint = ecefLocationToTransform + transformedNorthUnitVector.Row0;

                        FLatLonAlt originLLA = Conversions.CalculateLatLonHeightFromEcefXYZ(ECEFLocation);
                        FLatLonAlt easternLLA = Conversions.CalculateLatLonHeightFromEcefXYZ(
                            new Vector3Double
                            {
                                X = easternPoint.x,
                                Y = easternPoint.y,
                                Z = easternPoint.z
                            });
                        FLatLonAlt northernLLA = Conversions.CalculateLatLonHeightFromEcefXYZ(
                            new Vector3Double
                            {
                                X = northernPoint.x,
                                Y = northernPoint.y,
                                Z = northernPoint.z
                            });

                        FUTMCoordinates projectedOrigin = LatLonAltToProjected(originLLA);
                        FUTMCoordinates projectedEastern = LatLonAltToProjected(easternLLA);
                        FUTMCoordinates projectedNorthern = LatLonAltToProjected(northernLLA);

                        Vector3Double eastDirection = new Vector3Double
                        {
                            X = projectedEastern.Easting - projectedOrigin.Easting,
                            Y = projectedEastern.Northing - projectedOrigin.Northing,
                            Z = easternLLA.Altitude - originLLA.Altitude
                        };
                        Vector3Double northDirection = new Vector3Double
                        {
                            X = projectedNorthern.Easting - projectedOrigin.Easting,
                            Y = projectedNorthern.Northing - projectedOrigin.Northing,
                            Z = northernLLA.Altitude - originLLA.Altitude
                        };

                        //Normalize the east and north vectors
                        double eastDirectionMagnitude = Math.Sqrt(Math.Pow(eastDirection.X, 2) + Math.Pow(eastDirection.Y, 2) + Math.Pow(eastDirection.Z, 2));
                        double northDirectionMagnitude = Math.Sqrt(Math.Pow(northDirection.X, 2) + Math.Pow(northDirection.Y, 2) + Math.Pow(northDirection.Z, 2));
                        eastDirection = new Vector3Double
                        {
                            X = eastDirection.X / eastDirectionMagnitude,
                            Y = eastDirection.Y / eastDirectionMagnitude,
                            Z = eastDirection.Z / eastDirectionMagnitude
                        };
                        northDirection = new Vector3Double
                        {
                            X = northDirection.X / northDirectionMagnitude,
                            Y = northDirection.Y / northDirectionMagnitude,
                            Z = northDirection.Z / northDirectionMagnitude
                        };

                        //Put ENU vectors in terms of Unity coordinate system
                        ENU.EastVector = new Vector3((float)eastDirection.X, (float)-eastDirection.Y, (float)eastDirection.Z);
                        ENU.UpVector = new Vector3((float)northDirection.X, (float)-northDirection.Y, (float)northDirection.Z);
                        ENU.NorthVector = Vector3.Cross(ENU.UpVector, ENU.EastVector);

                        break;
                    }
            }

            return ENU;
        }

        public FLatLonAlt GetOriginLLA() { return OriginLLA; }
        public Vector3Double GetOriginECEF() { return OriginECEF; }

        #endregion PublicFunctions

        #region PrivateFunctions

        private void SetupVars()
        {
            OriginECEF = Conversions.CalculateEcefXYZFromLatLonHeight(OriginLLA);

            GeographicEllipsoid = new Vector3Double
            {
                X = EARTH_MAJOR_AXIS,
                Y = EARTH_MAJOR_AXIS,
                Z = EARTH_MINOR_AXIS
            };
            WorldFrameToECEFFrame = GetWorldFrameToECEFFrame(GeographicEllipsoid, OriginECEF);
            ECEFFrameToWorldFrame = WorldFrameToECEFFrame.Inverse;

            //Note: dmat constructor is set up using columns
            WorldFrameToUnityFrame = new dmat4(
                dvec4.UnitX,     // Easting (X) is Unity World Z
                -dvec4.UnitY,    // Northing (Y) is Unity World -X because of left-handed convention
                dvec4.UnitZ,     // Up (Z) is Unity World Y
                dvec4.UnitW);    // No Origin offset
            UnityFrameToWorldFrame = WorldFrameToUnityFrame.Inverse;
        }

        /// <summary>
        /// Get the matrix to transform a vector from Unity to ECEF CRS
        /// </summary>
        /// <param name="Ellisoid">The ellipsoid of the world.</param>
        /// <param name="ECEFLocation">The ECEF to get the world frame of.</param>
        /// <returns>The transformation matrix needed to convert Unity locations to ECEF CRS.</returns>
        private dmat4 GetWorldFrameToECEFFrame(Vector3Double Ellisoid, Vector3Double ECEFLocation)
        {
            // See ECEF standard : https://commons.wikimedia.org/wiki/File:ECEF_ENU_Longitude_Latitude_right-hand-rule.svg
            if (Math.Abs(ECEFLocation.X) < EPSILON &&
                Math.Abs(ECEFLocation.Y) < EPSILON)
            {
                // Special Case - On earth axis... 
                double Sign = 1.0;
                if (Math.Abs(ECEFLocation.Z) < EPSILON)
                {
                    // At origin - Should not happen, but consider it's the same as north pole
                    // Leave Sign = 1
                }
                else
                {
                    // At South or North pole - Axis are set to be continuous with other points
                    Sign = (ECEFLocation.Z < 0) ? -1 : 1;
                }

                return new dmat4(
                    new dvec4(1, 0, 0, ECEFLocation.X),           // East = X
                    new dvec4(0, 0, 1 * Sign, ECEFLocation.Y),   // North = Sign * Z
                    new dvec4(0, -1 * Sign, 0, ECEFLocation.Z),    // Up = Sign*Y
                    new dvec4(0, 0, 0, 1));
            }
            else
            {
                //Calculate the North, East, Up vectors of the ECEF location and create a transform matrix from them
                Vector3Double Up = new Vector3Double
                {
                    X = ECEFLocation.X * (1 / Math.Pow(Ellisoid.X, 2)),
                    Y = ECEFLocation.Y * (1 / Math.Pow(Ellisoid.Y, 2)),
                    Z = ECEFLocation.Z * (1 / Math.Pow(Ellisoid.Z, 2))
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
                    X = -ECEFLocation.Y,
                    Y = ECEFLocation.X,
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

                return new dmat4(new dvec4(East.X, North.X, Up.X, ECEFLocation.X),
                    new dvec4(East.Y, North.Y, Up.Y, ECEFLocation.Y),
                    new dvec4(East.Z, North.Z, Up.Z, ECEFLocation.Z),
                    new dvec4(0, 0, 0, 1));
            }
        }

        /// <summary>
        /// Gets the UTM letter designator for the given latitude.
        /// </summary>
        /// <param name="Lat">The latitude to get the UTM letter designator of. Should be given in decimal degrees.</param>
        /// <returns>A string containing the UTM letter designator.</returns>
        private string GetUTMLetterDesignator(double Lat)
        {
            string LetterDesignator;

            if ((84 >= Lat) && (Lat >= 72)) LetterDesignator = "X";
            else if ((72 > Lat) && (Lat >= 64)) LetterDesignator = "W";
            else if ((64 > Lat) && (Lat >= 56)) LetterDesignator = "V";
            else if ((56 > Lat) && (Lat >= 48)) LetterDesignator = "U";
            else if ((48 > Lat) && (Lat >= 40)) LetterDesignator = "T";
            else if ((40 > Lat) && (Lat >= 32)) LetterDesignator = "S";
            else if ((32 > Lat) && (Lat >= 24)) LetterDesignator = "R";
            else if ((24 > Lat) && (Lat >= 16)) LetterDesignator = "Q";
            else if ((16 > Lat) && (Lat >= 8)) LetterDesignator = "P";
            else if ((8 > Lat) && (Lat >= 0)) LetterDesignator = "N";
            else if ((0 > Lat) && (Lat >= -8)) LetterDesignator = "M";
            else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = "L";
            else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = "K";
            else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = "J";
            else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = "H";
            else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = "G";
            else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = "F";
            else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = "E";
            else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = "D";
            else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = "C";
            else LetterDesignator = "Z"; //This is here as an error flag to show that the Latitude is outside the UTM limits

            return LetterDesignator;
        }

        /// <summary>
        /// Transforms the given geodetic Lat, Lon, Alt coordinate to a flat earth coordinate
        /// </summary>
        /// <param name="latlonalt">The Lat in decimal degrees, Lon in decimal degrees, and Alt in meters to transform.</param>
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
        /// <returns>The Lat in decimal degrees, Lon in decimal degrees, and Alt in meters.</returns>
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

            return geodetic_to_flatearth(lla);
        }

        /// <summary>
        /// Transforms the given flat Earth coordinates to an ECEF XYZ coordinate
        /// </summary>
        /// <param name="FlatEarthCoords">The flat Earth X (East), Y (North), and Z (Up) coordinate to transform.</param>
        /// <returns>The ECEF XYZ location.</returns>
        private Vector3Double flatearth_to_ecef(Vector3Double FlatEarthCoords)
        {
            FLatLonAlt lla = flatearth_to_geodetic(FlatEarthCoords);

            return Conversions.CalculateEcefXYZFromLatLonHeight(lla);
        }

        #endregion PrivateFunctions
    }
}