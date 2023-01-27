using OpenDis.Core;
using OpenDis.Dis1998;
using System.Collections.Concurrent;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.Events;

namespace GRILLDIS
{
    public class PDUSender : MonoBehaviour
    {
        /// <summary>
        /// The maximum number of queued PDUs before the most recent ones get removed.
        /// </summary>
        [HideInInspector]
        public int maxQueueSize = 1 * 1024;

        /// <summary>
        /// The type of connection used in sending out the PDUs.
        /// </summary>
        [Tooltip("The type of connection used in sending out the PDUs.")]
        public ConnectionType connectionType;

        /// <summary>
        /// The IP Address to send UDP packets on.
        /// </summary>
        [HideInInspector]
        public string ipAddressString = "10.0.1.255";

        /// <summary>
        /// The Port to send UDP packets on. Valid Port ranges are from 1024 to 65535.
        /// </summary>
        [HideInInspector]
        public int port = 5001;

        /// <summary>
        /// Whether or not to auto connect the send address on start.
        /// </summary>
        [HideInInspector]
        public bool autoConnectAtStart = true;

        [HideInInspector]
        public UnityEvent<SocketException> OnFailedToConnect = new UnityEvent<SocketException>();

        private ConcurrentQueue<byte[]> pdus = new ConcurrentQueue<byte[]>();
        private bool sending = true;
        private Thread thread;
        private DataOutputStream dos = new DataOutputStream(Endian.Big);

        void Awake() { if (autoConnectAtStart) { Init(); } }
        void OnApplicationQuit() { if (thread != null) { Stop(); } }

        //Method run by the thread
        private void SenderWork(IPAddress targetIP)
        {
            switch (connectionType)
            {
                #region Unicast

                case ConnectionType.Unicast:
                    IPEndPoint uCastEndPoint = new IPEndPoint(targetIP, port);
                    UdpClient client = new UdpClient();

                    while (sending)
                    {
                        if (pdus.TryDequeue(out byte[] pdu))
                        {
                            client.Send(pdu, pdu.Length, uCastEndPoint);
                        }
                    }
                    client.Close();
                    break;

                #endregion Unicast

                #region Broadcast

                case ConnectionType.Broadcast:
                    EndPoint broadcastEP = new IPEndPoint(IPAddress.Broadcast, port);
                    Socket broadcastSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                    broadcastSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, 1);

                    while (sending)
                    {
                        if (pdus.TryDequeue(out byte[] pdu))
                        {
                            broadcastSocket.SendTo(pdu, broadcastEP);
                        }
                    }
                    broadcastSocket.Close();
                    break;

                #endregion Broadcast

                #region Multicast

                case ConnectionType.Multicast:

                    UdpClient mcastSocket = new UdpClient();

                    mcastSocket.JoinMulticastGroup(targetIP, LocalIPAddress());

                    IPEndPoint mcastEP = new IPEndPoint(targetIP, port);

                    while (sending)
                    {
                        if (pdus.TryDequeue(out byte[] pdu))
                        {
                            mcastSocket.Send(pdu, pdu.Length, mcastEP);
                        }
                    }
                    mcastSocket.Close();
                    break;

                #endregion Multicast

                default:
                    Debug.LogError("ERROR Invalid connection type on PDU sender");
                    break;
            }
        }

        //Starts up the thread from the desired ip
        public void Init()
        {
            if (thread == null)
            {
                IPAddress IP = IPAddress.Parse(ipAddressString);
                if (System.Net.NetworkInformation.IPGlobalProperties.GetIPGlobalProperties().GetActiveUdpListeners().Any(p => p.Port == port))
                { OnFailedToConnect.Invoke(new SocketException(10048)); return; }

                thread = new Thread(() => SenderWork(IP));
                thread.Start();
            }
            else
            {
                Debug.LogWarning("Thread already active. Run Stop() method before attempting connection");
            }
        }

        //Stops the thread and disconnects
        public void Stop()
        {
            if (thread != null)
            {
                sending = false;
                thread.Join();
                thread = null;
            }
            else
            {
                Debug.LogWarning("No thread active. Run Init() method before attempting disconnect");
            }
        }

        //Changes IP safely
        public void ChangeAddress(string IP, int port)
        {
            if (port > 65535 || port < 0) { Debug.LogError("ERROR: Invalid port change"); return; }
            Stop();
            ipAddressString = IP;
            this.port = port;
            Init();
        }

        public bool IsConnected()
        {
            return thread != null;
        }

        #region Send PDU Methods

        public void SendPdu(AcknowledgePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ActionRequestPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ActionResponsePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(CollisionPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(CommentPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(CreateEntityPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(DataPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(DataQueryPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(DesignatorPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(DetonationPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ElectronicEmissionsPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(EntityStatePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(EntityStateUpdatePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(EventReportPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(FirePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ReceiverPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(RemoveEntityPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(RepairCompletePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(RepairResponsePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ResupplyCancelPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ResupplyOfferPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ResupplyReceivedPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(ServiceRequestPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(SetDataPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(SignalPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(StartResumePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(StopFreezePdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        public void SendPdu(TransmitterPdu pdu)
        {
            dos.DS.Clear();
            pdu.MarshalAutoLengthSet(dos);
            pdus.Enqueue(dos.ConvertToBytes());
            if (pdus.Count > maxQueueSize) { pdus.TryDequeue(out byte[] trash); }
        }

        #endregion Send PDU Methods

        //Returns the local IP address
        private static IPAddress LocalIPAddress()
        {
            IPHostEntry host;
            string localIP = "0.0.0.0";
            host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (IPAddress ip in host.AddressList)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    localIP = ip.ToString();
                    break;
                }
            }
            return IPAddress.Parse(localIP);
        }
    }
}