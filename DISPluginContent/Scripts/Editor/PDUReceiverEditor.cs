using UnityEngine;
using UnityEditor;

namespace GRILLDIS
{
    /// <summary>
    /// Custom editor for the PDUReceiver class
    /// </summary>
    [CustomEditor(typeof(PDUReceiver))]
    public class PDUReceiverEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            //Draw all non-custom variables
            DrawDefaultInspector();
            var receiver = target as PDUReceiver;

            //Draw IP Address if relavent
            EditorGUI.BeginDisabledGroup(receiver.useMulticast);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("ipAddressString"), new GUIContent("IP Address", "The IP Address to receive UDP packets from. An IP Address of 0.0.0.0 listens to all connections on the specified port."));
            EditorGUI.EndDisabledGroup();

            //Draw Multicast Address if relavent
            EditorGUI.BeginDisabledGroup(!receiver.useMulticast);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("multicastAddress"), new GUIContent("Multicast Address", "The multicast address to receive UDP packets from. DIS transient multicast groups range from 224.252.0.0 - 224.255.255.255 per IANA standards."));
            EditorGUI.EndDisabledGroup();

            //Draw port and confirm valid entry
            EditorGUILayout.PropertyField(serializedObject.FindProperty("port"), new GUIContent("Port", "The Port to receive UDP packets from. Valid Port ranges are from 1024 to 65535."));
            if (receiver.port < 1024) { receiver.port = 1024; }
            if (receiver.port > 65535) { receiver.port = 65535; }

            serializedObject.ApplyModifiedProperties();
        }
    }
}