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
    private dvec3 North, East, Down;

    private mat4 WorldFrameToECEFFrame; // Matrix to transform a vector from EU to ECEF CRS
    private mat4 ECEFFrameToWorldFrame; // Matrix to transform a vector from ECEF to UE CRS - Inverse of the previous one kept in cache. 

    private mat4 WorldFrameToUnityFrame;
    private mat4 UnityFrameToWorldFrame;

    // Start is called before the first frame update
    void Awake()
    {
        Vector3Double b = new Vector3Double { 
        X = OriginLat,
        Y = OriginLon,
        Z = OriginAlt
        };

        Conversions.CalculateEcefXYZFromLatLonHeight(b, out Origin);
        Conversions.CalculateNorthEastDownVectorsFromLatLon(OriginLat, OriginLon, out North, out East, out Down);
        GetWorldFrameToECEFFrame(Origin);
        GetWorldFrameToUnityFrame();
        
    }

    public Vector3Double GetOrigin() { return Origin; }
    public dvec3 GetNorth() { return North; }
    public dvec3 GetEast() { return East; }
    public dvec3 GetDown() { return Down; }
    
    private void GetWorldFrameToECEFFrame(Vector3Double ECEF)
    {
        dvec3 Up = new dvec3(ECEF.X / Mathf.Pow((float)EARTH_MAJOR_AXIS, 2), ECEF.Y / Mathf.Pow((float)EARTH_MAJOR_AXIS, 2), ECEF.Z / Mathf.Pow((float)EARTH_MINOR_AXIS, 2));

        dvec3 East = new dvec3( -ECEF.Y, ECEF.X, 0).Normalized;

        dvec3 North = dvec3.Cross(Up, East);

        mat4 mat = new mat4
            ((float) East.x,   (float) East.y,    (float) East.z,  0,
             (float) North.x,  (float) North.y,   (float) North.z, 0,
             (float) Up.x,     (float) Up.y,      (float) Up.z,    0,
             (float) ECEF.X,   (float) ECEF.Y,    (float) ECEF.Z,   1);


        WorldFrameToECEFFrame = mat;
        ECEFFrameToWorldFrame = WorldFrameToECEFFrame.Inverse;
    }

    private void GetWorldFrameToUnityFrame()
    {
        mat4 WorldFrameToUEFrame =
            new mat4(1,  0,  0,  0,
                     0,  -1, 0,  0,
                     0,  0,  1,  0,
                     0,  0,  0,  1);

        WorldFrameToUnityFrame = WorldFrameToUEFrame * 
            new mat4(0,  0,  1,  0,
                     1,  0,  0,  0,
                     0,  1,  0,  0,
                     0,  0,  0,  1);

        UnityFrameToWorldFrame = WorldFrameToUnityFrame.Inverse;
    }

    public vec4 GetUnityCoordinatesFromECEF(Vector3Double ECEF)
    {
        vec4 multiplier = new vec4((float)ECEF.X, (float)ECEF.Y, (float)ECEF.Z, 1);

        float elem0 = vec4.Dot(ECEFFrameToWorldFrame.Row0, multiplier);
        float elem1 = vec4.Dot(ECEFFrameToWorldFrame.Row1, multiplier);
        float elem2 = vec4.Dot(ECEFFrameToWorldFrame.Row2, multiplier);
        float elem3 = vec4.Dot(ECEFFrameToWorldFrame.Row3, multiplier);

        return new vec4(elem0, elem1, elem2, elem3);
    }


    #region OldMath
    ///OldMath
    /*
    dvec2 N2 = new dvec2(North.x, North.z);
    heading = (Mathf.Acos((float)(dvec2.Dot(N2, -dvec2.UnitY) / (N2.Length * Mathf.Sqrt(1.0f))))) * 180 / Mathf.PI;
    Debug.Log("Heading: " + heading + "  Math: " + (180 - OriginLat));

    dvec2 E2 = new dvec2(East.z, East.y);
    pitch = (Mathf.Acos((float)(dvec2.Dot(E2, dvec2.UnitX) / (E2.Length * Mathf.Sqrt(1.0f))))) * 180 / Mathf.PI;
    Debug.Log("Pitch: " + pitch);

    dvec2 D2 = new dvec2(Down.x, Down.y);
    roll = (Mathf.Acos((float)(dvec2.Dot(D2, -dvec2.UnitY) / (D2.Length * Mathf.Sqrt(1.0f))))) * 180 / Mathf.PI;
    Debug.Log("Roll: " + roll + "  Math: " + (90 - OriginLon));

    Conversions.ApplyHeadingPitchRollToNorthEastDownVector(heading, pitch, roll, North, East, Down, out testN, out testE, out testD);
    Debug.Log(testN);
    Debug.Log(testE);
    Debug.Log(testD);
    */
    ///
    #endregion OldMath
}
