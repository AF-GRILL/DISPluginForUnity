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
            Conversions.GetUnityLocationAndOrientationFromEntityStatePdu(DeadReckonedPDUIn, georeferenceSystemScript, out Vector3 unityLoc, out Vector3 unityRot);
            transform.position = unityLoc;
            transform.rotation = Quaternion.Euler(unityRot);
        }

        public void HandleEntityStateProcessed(EntityStatePdu EntityStatePDUIn)
        {
            Conversions.GetUnityLocationAndOrientationFromEntityStatePdu(EntityStatePDUIn, georeferenceSystemScript, out Vector3 unityLoc, out Vector3 unityRot);
            transform.position = unityLoc;
            transform.rotation = Quaternion.Euler(unityRot);
        }

    }
}