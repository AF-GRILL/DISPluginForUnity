using UnityEngine;
using UnityEditor;


namespace GRILLDIS
{
    /// <summary>
    /// Custom editor for the PDUSender class
    /// </summary>
    [CustomEditor(typeof(PDUSender))]
    public class PDUSenderEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            //Draw all non-custom variables
            DrawDefaultInspector();
            var sender = target as PDUSender;

            //Draw Auto Connect
            EditorGUILayout.PropertyField(serializedObject.FindProperty("autoConnectAtStart"), new GUIContent("Auto-Connect", "Whether or not to auto connect the send address on start."));

            //Draw IP Address if relavent
            EditorGUI.BeginDisabledGroup(sender.connectionType == ConnectionType.Broadcast);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("ipAddressString"), new GUIContent("IP Address", "The IP Address to send UDP packets on."));
            EditorGUI.EndDisabledGroup();

            //Draw port and confirm valid entry
            EditorGUILayout.PropertyField(serializedObject.FindProperty("port"), new GUIContent("Port", "The Port to send UDP packets on. Valid Port ranges are from 1024 to 65535."));
            if (sender.port < 1024) { sender.port = 1024; }
            if (sender.port > 65535) { sender.port = 65535; }

            //Draw Max Queue Size
            EditorGUILayout.PropertyField(serializedObject.FindProperty("maxQueueSize"), new GUIContent("Max Queue Size", "The maximum number of queued PDUs before the most recent ones get removed."));

            serializedObject.ApplyModifiedProperties();
        }
    }
}