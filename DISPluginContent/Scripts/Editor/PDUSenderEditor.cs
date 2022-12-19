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
            sender.autoConnectAtStart = EditorGUILayout.Toggle(new GUIContent("Auto-Connect", "Whether or not to auto connect the send address on start."), sender.autoConnectAtStart);

            //Draw IP Address if relavent
            EditorGUI.BeginDisabledGroup(sender.connectionType == ConnectionType.Broadcast);
            sender.ipAddressString = EditorGUILayout.TextField(new GUIContent("IP Address", "The IP Address to send UDP packets on."), sender.ipAddressString);
            EditorGUI.EndDisabledGroup();

            //Draw port and confirm valid entry
            sender.port = EditorGUILayout.IntField(new GUIContent("Port", "The Port to send UDP packets on. Valid Port ranges are from 1024 to 65535."), sender.port);
            if (sender.port < 1024) { sender.port = 1024; }
            if (sender.port > 65535) { sender.port = 65535; }

            //Draw Max Queue Size
            sender.maxQueueSize = EditorGUILayout.IntField(new GUIContent("Max Queue Size", "The maximum number of queued PDUs before the most recent ones get removed."), sender.maxQueueSize);
        }
    }
}