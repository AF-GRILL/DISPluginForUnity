using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenDis.Dis1998;
using GRILLDIS;

namespace GRILLDIS.Sample
{
    public class Sample_HandleDISUpdates : MonoBehaviour
    {
        private GeoreferenceSystem georeferenceSystemScript;
        private void Awake()
        {
            georeferenceSystemScript = FindObjectOfType<GeoreferenceSystem>();
        }

        public void Start()
        {
        }

        public void HandleDeadReckoningUpdate(EntityStatePdu DeadReckonedPDUIn)
        {
            Conversions.GetUnityLocationAndOrientationFromEntityStatePdu(DeadReckonedPDUIn, georeferenceSystemScript, out Vector3Double unityLoc, out Vector3 unityRot);
            transform.SetPositionAndRotation(new Vector3((float)unityLoc.X, (float)unityLoc.Y, (float)unityLoc.Z), Quaternion.Euler(unityRot));
            //transform.position = new Vector3((float)unityLoc.X, (float)unityLoc.Y, (float)unityLoc.Z);
            //transform.rotation = Quaternion.Euler(unityRot);
        }

        public void HandleEntityStateProcessed(EntityStatePdu EntityStatePDUIn)
        {
            Conversions.GetUnityLocationAndOrientationFromEntityStatePdu(EntityStatePDUIn, georeferenceSystemScript, out Vector3Double unityLoc, out Vector3 unityRot);
            transform.SetPositionAndRotation(new Vector3((float)unityLoc.X, (float)unityLoc.Y, (float)unityLoc.Z), Quaternion.Euler(unityRot));
            //transform.position = new Vector3((float)unityLoc.X, (float)unityLoc.Y, (float)unityLoc.Z);
            //transform.rotation = Quaternion.Euler(unityRot);
        }

    }
}