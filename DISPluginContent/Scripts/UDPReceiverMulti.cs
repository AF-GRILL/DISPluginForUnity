using System;
using System.Net;
using System.Net.Sockets;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using UnityEngine.Events;
using OpenDis.Dis1998;

namespace GRILLDIS.UDPReceiverMulti
{
    class UDPReceiverMulti
    {
        List<PDUProcessorInterface> pduProcessors;

        private IPAddress ipAddress;
        private int port;
        private IPAddress multicastAddress;
        private bool useMulticast;
        private bool allowLoopback;

        /// <summary>
        /// The IP Address this UDP Receiver is on. If the receiver is using multicast, use the Multicast Address variable.
        /// </summary>
        public IPAddress IPAddress { get => ipAddress; set => ipAddress = value; }
        /// <summary>
        /// The port this UDP Receiver is on.
        /// </summary>
        public int Port { get => port; set => port = value; }
        /// <summary>
        /// Whether or not this UDP Receiver is using multicast.
        /// </summary>
        public bool UseMulticast { get => useMulticast; set => useMulticast = value; }
        /// <summary>
        /// Whether or not this UDP Receiver is processing network loopback traffic.
        /// </summary>
        public bool AllowLoopback { get => allowLoopback; set => allowLoopback = value; }
        /// <summary>
        /// The multicast address this UDP receiver is on. If the receiver is not using multicast, use the IP Address variable.
        /// </summary>
        public IPAddress MulticastAddress { get => multicastAddress; set => multicastAddress = value; }

        private UdpClient client;
        private IPEndPoint epReceive;

        private ConcurrentList<byte[]> udpPacketList;

        private Thread receiveThread;

        private string LocalIPAddress;

        private static int MAX_QUEUE_SIZE = 1 * 1024;

        private bool isCancelled;

        private Exception exception;
        public UnityEvent<Exception> OnFailedToConnect = new UnityEvent<Exception>();

        public UDPReceiverMulti(IPAddress ipAddress, int port, IPAddress multicastAddress, bool useMulticast, bool allowLoopback)
        {
            this.ipAddress = ipAddress;
            this.port = port;
            this.multicastAddress = multicastAddress;
            this.useMulticast = useMulticast;
            this.allowLoopback = allowLoopback;
            isCancelled = false;
            pduProcessors = new List<PDUProcessorInterface>();

            var host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var ip in host.AddressList)
            {
                if (ip.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                {
                    LocalIPAddress = ip.ToString();
                    break;
                }
            }
        }

        public void beginReceiving()
        {
            exception = null;
            receiveThread = new Thread(new ThreadStart(BeginReceive));
            receiveThread.Start();
            receiveThread.Join();
            if (exception != null) { OnFailedToConnect.Invoke(exception); }
        }

        public void stopReceiving()
        {
            isCancelled = true;
            client?.Close(); 
            receiveThread.Abort();
            isCancelled = false;
        }

        private void BeginReceive()
        {
            try
            {
                udpPacketList = new ConcurrentList<byte[]>(MAX_QUEUE_SIZE);
                epReceive = new IPEndPoint(ipAddress, port);

                //Make the receive socket non-binding to make the IP Endpoint reusable
                client = new UdpClient();
                client.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                client.ExclusiveAddressUse = false;
                client.Client.Bind(epReceive);

                if (UseMulticast)
                {
                    //Setup multicast loopback if enabled.
                    client.MulticastLoopback = allowLoopback;

                    epReceive = new IPEndPoint(IPAddress.Any, port);
                    client.JoinMulticastGroup(multicastAddress);
                }
                client.BeginReceive(new AsyncCallback(asyncReceive), client);
            }catch (SocketException ex)
            {
                exception = ex;
                stopReceiving();
            }
        }

        private void asyncReceive(IAsyncResult result)
        {
            byte[] receivedBytes;
            if (!isCancelled)
            {
                try
                {
                    receivedBytes = client.EndReceive(result, ref epReceive);
                }
                catch (System.ObjectDisposedException ex)
                {
                    Console.WriteLine("Async receive socket object disposed exception: " + ex.Message);
                    return;
                }

                //Ignore packets from self if loopback is disabled. This will cover ignoring broadcast packets. Ignoring multicast packets is covered in setting up of the receive socket above through MulticastLoopback.
                if (!allowLoopback && epReceive.Address.ToString().Equals(LocalIPAddress))
                {
                    return;
                }

                udpPacketList.Add(receivedBytes);

                while (udpPacketList.Count > MAX_QUEUE_SIZE)
                {
                    //Remove the last element
                    udpPacketList.RemoveAt(0);
                    //Console.WriteLine("Packet list count: " + udpPacketList.Count);
                }
                //Console.WriteLine("Received packet of " + receivedBytes.Length + " bytes");
                //Debug.Log("Received packet of " + receivedBytes.Length + " bytes");
                //Debug.Log("udpPacketList count: " + udpPacketList.Count);
                client.BeginReceive(new AsyncCallback(asyncReceive), client);
                foreach (PDUProcessorInterface pduProcessor in pduProcessors)
                {
                    pduProcessor.OnUDPPacketReceived(receivedBytes, this);
                }
            }
        }

        public void registerPDUProcessor(PDUProcessorInterface pduProcessor)
        {
            pduProcessors.Add(pduProcessor);
        }

        public void unregisterPDUProcessor(PDUProcessorInterface pduProcessor)
        {
            pduProcessors.Remove(pduProcessor);
        }

    } //end of class
}
