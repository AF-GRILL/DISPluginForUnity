namespace GRILLDIS
{
    interface PDUProcessorInterface
    {
        void OnUDPPacketReceived(byte[] bytes, UDPReceiverMulti.UDPReceiverMulti udpReceiverMulti);

    }
}