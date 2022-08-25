using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenDis.Dis1998;

/// <summary>
/// Temporary Class for testing sending and recieving PDUs
/// </summary>



public class PDUTester : MonoBehaviour
{
    PDUSender sender;
    PDUReceiver receiver;




    // Start is called before the first frame update
    void Start()
    {
        sender = GameObject.FindObjectOfType<PDUSender>();
        receiver = GameObject.FindObjectOfType<PDUReceiver>();
        StartCoroutine(send());
    }

    private IEnumerator send()
    {
        while (true)
        {
            //yield return new WaitForSeconds(0.1f);
            Debug.Log("Sent PDUs");
            //sender.SendPdu(new AcknowledgePdu());
            //sender.SendPdu(new ActionRequestPdu());
            //sender.SendPdu(new ActionResponsePdu());
            //sender.SendPdu(new CollisionPdu());
            //sender.SendPdu(new CommentPdu());
            //sender.SendPdu(new CreateEntityPdu());
            //sender.SendPdu(new DataPdu());
            //sender.SendPdu(new DataQueryPdu());
            //sender.SendPdu(new DesignatorPdu());
            //sender.SendPdu(new DetonationPdu());
            //sender.SendPdu(new ElectronicEmissionsPdu());
            sender.SendPdu(new EntityStatePdu());
            //sender.SendPdu(new EntityStateUpdatePdu());
            //sender.SendPdu(new EventReportPdu());
            //sender.SendPdu(new FirePdu());
            //sender.SendPdu(new ReceiverPdu());
            //sender.SendPdu(new RemoveEntityPdu());
            //sender.SendPdu(new RepairCompletePdu());
            //sender.SendPdu(new RepairResponsePdu());
            //sender.SendPdu(new ResupplyCancelPdu());
            //sender.SendPdu(new ResupplyOfferPdu());
            //sender.SendPdu(new ResupplyReceivedPdu());
            //sender.SendPdu(new ServiceRequestPdu());
            //sender.SendPdu(new SetDataPdu());
            //sender.SendPdu(new SignalPdu());
            //sender.SendPdu(new StartResumePdu());
            //sender.SendPdu(new StopFreezePdu());
            //sender.SendPdu(new TransmitterPdu());
        }
    }
}
