using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEngine;
using UnityEditor;
using UDPReceiverMulti;
using OpenDis.Dis1998;
using System;
using System.Reflection;
using System.Linq;

public class PDUReceiver : MonoBehaviour
{

    /// <summary>
    /// The IP Address to receive UDP packets from. An IP Address of 0.0.0.0 listens to all connection on the specified port.
    /// </summary>
    [HideInInspector]
    public string ipAddressString = "0.0.0.0";
    /// <summary>
    /// The Port to receive UDP packets from. Valid Port ranges are from 1024 to 65535.
    /// </summary>
    [HideInInspector]
    public int port = 3000;
    /// <summary>
    /// Whether or not to auto connect the receive address on start.
    /// </summary>
    [Tooltip("Whether or not to auto connect the receive address on start.")]
    public bool autoConnectAtStart = true;
    /// <summary>
    /// Whether or not multicast should be used with this receive socket.
    /// </summary>
    [Tooltip("Whether or not multicast should be used with this receive socket.")]
    public bool useMulticast = false;
    /// <summary>
    /// Whether or not packets sent by the local machine should be processed.
    /// </summary>
    [Tooltip("Whether or not packets sent by the local machine should be processed.")]
    public bool allowLoopback = false;
    /// <summary>
    /// The multicast address to receive UDP packets from. DIS transient multicast groups range from 224.252.0.0 - 224.255.255.255 per IANA standards.
    /// </summary>
    [HideInInspector]
    public string multicastAddress = "224.252.0.1";

    // Start is called before the first frame update
    private UDPReceiverMulti.UDPReceiverMulti receiver;
    private IPAddress ipAddress;
    private PDUProcessor pduProcessor;
    private DISGameManager disManagerScript;

    void Start()
    {
        if (GetComponent<DISGameManager>() != null)
        {
            disManagerScript = gameObject.GetComponent<DISGameManager>();
        }

        if (autoConnectAtStart)
        {
            startUDPReceiver();
        }
    }

    // Update is called once per frame
    void Update()
    {
        //Check to see if any PDUs have been received since the last tick
        pduProcessor.CheckForUpdate();
    }

    private void OnApplicationQuit()
    {
        receiver.stopReceiving();
    }

    public void startUDPReceiver()
    {
        ipAddress = IPAddress.Any;
        if (ipAddressString != "0.0.0.0")
        {
            ipAddress = IPAddress.Parse(ipAddressString);
        }
        IPAddress multicastIpAddress = IPAddress.Parse(multicastAddress);
        pduProcessor = new PDUProcessor(ProcessDISPacket);
        receiver = new UDPReceiverMulti.UDPReceiverMulti(ipAddress, port, multicastIpAddress, useMulticast, allowLoopback);
        receiver.registerPDUProcessor(pduProcessor);
        receiver.beginReceiving();
    }

    private void ProcessDISPacket(Pdu PDUPacketToProcess)
    {
        if (PDUPacketToProcess.ExerciseID == disManagerScript.ExerciseID)
        {
            if (PDUPacketToProcess is EntityStatePdu)
            {
                EntityStatePdu entityStatePdu = (EntityStatePdu)PDUPacketToProcess;
                GameObject obj = disManagerScript.SpawnOrGetGameObjectFromEntityStatePDU(entityStatePdu);

                if (obj != null)
                {
                    DISComponent disComponentScript = obj.GetComponent<DISComponent>();

                    if (disComponentScript != null)
                    {
                        disComponentScript.HandleEntityStatePDU(PDUPacketToProcess as EntityStatePdu);
                    }
                    else
                    {
                        Debug.Log("GameObject " + obj.name + " does not have a DIS Component attached for further handling of PDUs!");
                    }
                }
            }
            else if (PDUPacketToProcess is EntityStateUpdatePdu)
            {
                EntityStateUpdatePdu entityStateUpdatePdu = (EntityStateUpdatePdu)PDUPacketToProcess;

                // NOTE: Entity State Update PDUs do not contain an Entity Type, so we cannot spawn an entity from one
                DISComponent disComponent = disManagerScript.GetAssociatedDISComponent(entityStateUpdatePdu.EntityID);

                if (disComponent != null)
                {
                    disComponent.HandleEntityStateUpdatePDU(entityStateUpdatePdu);
                }
            }
            else if (PDUPacketToProcess is FirePdu)
            {
                FirePdu firePdu = (FirePdu)PDUPacketToProcess;
                DISComponent disComponent = disManagerScript.GetAssociatedDISComponent(firePdu.FiringEntityID);

                if (disComponent != null)
                {
                    disComponent.HandleFirePDU(firePdu);
                }
            }
            else if (PDUPacketToProcess is DetonationPdu)
            {
                DetonationPdu detonationPdu = (DetonationPdu)PDUPacketToProcess;
                DISComponent disComponent = disManagerScript.GetAssociatedDISComponent(detonationPdu.MunitionID);

                if (disComponent != null)
                {
                    disComponent.HandleDetonationPDU(detonationPdu);
                }
            }
            else if (PDUPacketToProcess is RemoveEntityPdu)
            {
                RemoveEntityPdu removeEntityPdu = (RemoveEntityPdu)PDUPacketToProcess;
                //Verify we are the appropriate sim to handle this PDU
                if (removeEntityPdu.ReceivingEntityID.Site == disManagerScript.SiteID && removeEntityPdu.ReceivingEntityID.Application == disManagerScript.ApplicationID)
                {
                    DISComponent disComponent = disManagerScript.GetAssociatedDISComponent(removeEntityPdu.ReceivingEntityID);

                    if (disComponent != null)
                    {
                        disComponent.HandleRemoveEntityPDU(removeEntityPdu);
                    }
                }
            }
            else if (PDUPacketToProcess is StartResumePdu)
            {
                StartResumePdu startResumePdu = (StartResumePdu)PDUPacketToProcess;
                //Verify we are the appropriate sim to handle this PDU
                if (startResumePdu.ReceivingEntityID.Site == disManagerScript.SiteID && startResumePdu.ReceivingEntityID.Application == disManagerScript.ApplicationID)
                {
                    DISComponent disComponent = disManagerScript.GetAssociatedDISComponent(startResumePdu.ReceivingEntityID);

                    if (disComponent != null)
                    {
                        disComponent.HandleStartResumePDU(startResumePdu);
                    }
                }
            }
            else if (PDUPacketToProcess is StopFreezePdu)
            {
                StopFreezePdu stopFreezePdu = (StopFreezePdu)PDUPacketToProcess;
                //Verify we are the appropriate sim to handle this PDU
                if (stopFreezePdu.ReceivingEntityID.Site == disManagerScript.SiteID && stopFreezePdu.ReceivingEntityID.Application == disManagerScript.ApplicationID)
                {
                    DISComponent disComponent = disManagerScript.GetAssociatedDISComponent(stopFreezePdu.ReceivingEntityID);

                    if (disComponent != null)
                    {
                        disComponent.HandleStopFreezePDU(stopFreezePdu);
                    }
                }
            }
        }
    }
}
