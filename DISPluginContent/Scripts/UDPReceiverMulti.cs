using System;
using System.Net;
using System.Net.Sockets;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using OpenDis.Dis1998;

namespace UDPReceiverMulti
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
            receiveThread = new Thread(new ThreadStart(BeginReceive));
            receiveThread.Start();
        }

        public void stopReceiving()
        {
            isCancelled = true;
            client.Close();
            receiveThread.Abort();
            isCancelled = false;
        }

        private void BeginReceive()
        {

            udpPacketList = new ConcurrentList<byte[]>(MAX_QUEUE_SIZE);
            client = new UdpClient(port);
            epReceive = new IPEndPoint(ipAddress, port);
            
            if (UseMulticast)
            {
                //Setup multicast loopback if enabled.
                client.MulticastLoopback = allowLoopback;

                epReceive = new IPEndPoint(IPAddress.Any, port);
                client.JoinMulticastGroup(multicastAddress);
            }
            client.BeginReceive(new AsyncCallback(asyncReceive), client);
        }

        private byte[] getLatestBytes()
        {
            if (udpPacketList.Count > 0)
            {
                return udpPacketList[udpPacketList.Count - 1];
            }
            return new byte[0];
        }

        private void asyncReceive(IAsyncResult result)
        {

            if (!isCancelled)
            {
                byte[] receivedBytes = client.EndReceive(result, ref epReceive);

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

        static void Main(string[] args)
        {
            Console.WriteLine("Starting receiver");
            //IPAddress ipAddress = IPAddress.Parse("10.0.1.134");
            IPAddress ipAddress = IPAddress.Any;
            int port = 3000;

            UDPReceiverMulti receiver = new UDPReceiverMulti(ipAddress, port, IPAddress.Parse("224.252.0.1"), false, true);
            receiver.beginReceiving();
            Thread.Sleep(5000);
            receiver.stopReceiving();
            Thread.Sleep(5000);
            receiver.beginReceiving();
            Console.ReadKey();
            //broadcastPort = 3000;  //3000 is default for DisMapper / VBS

            // Start a multicast or broadcast.
            //StartMulticast();  // Mulicast only
            //StartBroadcast(); //Works with DisMapper and EspduSender when they are set to 'Broadcast'           

            // Process Incomming messages.
            //ReceiveBroadcastMessages();
        }

        public void registerPDUProcessor(PDUProcessorInterface pduProcessor)
        {
            pduProcessors.Add(pduProcessor);
        }

        public void unregisterPDUProcess(PDUProcessorInterface pduProcessor)
        {
            pduProcessors.Remove(pduProcessor);
        }

    } //end of class
}
