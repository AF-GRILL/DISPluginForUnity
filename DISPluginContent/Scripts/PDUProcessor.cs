using System.Collections;
using System.Collections.Generic;
using OpenDis.Core;
using OpenDis.Dis1998;
using System;
using UnityEngine;
using System.Threading;
using System.Collections.Concurrent;

namespace GRILLDIS
{
    class PDUProcessor : PDUProcessorInterface
    {
        public delegate void MessageDelegate(Pdu receivedPDU);


        private readonly MessageDelegate _pduDelegate;
        private readonly ConcurrentQueue<Pdu> _pduMessageQueue = new ConcurrentQueue<Pdu>();

        public PDUProcessor(MessageDelegate pduDelegate)
        {
            _pduDelegate = pduDelegate;
        }

        public void OnUDPPacketReceived(byte[] receivedBytes, UDPReceiverMulti.UDPReceiverMulti udpReceiverMulti)
        {
            //Debug.Log("PDU processor received packet of " + receivedBytes.Length + " bytes");
            uint pduTypePosition = OpenDis.Core.PduProcessor.PDU_TYPE_POSITION;
            byte pduType = receivedBytes[pduTypePosition];
            //List<object> pduList = openDISpduProcessor.ProcessPdu(receivedBytes, Endian.Big);
            try
            {
                OpenDis.Dis1998.Pdu pdu = OpenDis.Core.PduProcessor.UnmarshalRawPdu(pduType, receivedBytes, Endian.Big);

                _pduMessageQueue.Enqueue(pdu);
            }
            catch (Exception ex)
            {
                Debug.LogError("Error occurred while unmarshalling received PDU: " + ex.Message);
            }
        }

        public void CheckForUpdate()
        {
            //Check if any messages have been received. If so, dequeue them and delegate them appropriately
            if (_pduMessageQueue.TryDequeue(out Pdu receivedPdu))
            {
                _pduDelegate(receivedPdu);
            }
        }
    }
}