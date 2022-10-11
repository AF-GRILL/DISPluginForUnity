using UnityEngine;
using OpenDis.Dis1998;

public enum ConnectionType
{
    Unicast,
    Broadcast,
    Multicast
}

public enum EForceID : byte
{
    Other,
    Friendly,
    Opposing,
    Neutral,
    Friendly2,
    Opposing2,
    Neutral2,
    Friendly3,
    Opposing3,
    Neutral3,
    Friendly4,
    Opposing4,
    Neutral4,
    Friendly5,
    Opposing5,
    Neutral5,
    Friendly6,
    Opposing6,
    Neutral6,
    Friendly7,
    Opposing7,
    Neutral7,
    Friendly8,
    Opposing8,
    Neutral8,
    Friendly9,
    Opposing9,
    Neutral9,
    Friendly10,
    Opposing10,
    Neutral10
};

public enum EGroundClampingMode
{
    None,
    GroundClampWithDISOptions,
    AlwaysGroundClamp
};

public enum EDISCullingMode
{
    None,
    CullDeadReckoning,
    CullAll
};

public enum EEntityStateSendingMode
{
    None,
    EntityStatePDU,
    EntityStateUpdatePDU
};

public enum EDeadReckoningAlgorithm
{
    Other,
    Static,
    FPW,
    RPW,
    RVW,
    FVW,
    FPB,
    RPB,
    RVB,
    FVB
}

public struct FEastNorthUp
{
    public Vector3 EastVector;
    public Vector3 NorthVector;
    public Vector3 UpVector;

    public FEastNorthUp(Vector3 EastVector, Vector3 NorthVector, Vector3 UpVector)
    {
        this.EastVector = EastVector;
        this.NorthVector = NorthVector;
        this.UpVector = UpVector;
    }
};

public struct FNorthEastDown
{
    public Vector3 NorthVector;
    public Vector3 EastVector;
    public Vector3 DownVector;

    public FNorthEastDown(Vector3 NorthVector, Vector3 EastVector, Vector3 DownVector)
    {
        this.NorthVector = NorthVector;
        this.EastVector = EastVector;
        this.DownVector = DownVector;
    }
};

public struct FHeadingPitchRoll
{
    public float Heading;
    public float Pitch;
    public float Roll;

    public FHeadingPitchRoll(float Heading, float Pitch, float Roll)
    {
        this.Heading = Heading;
        this.Pitch = Pitch;
        this.Roll = Roll;
    }
    public FHeadingPitchRoll(Orientation InOrientation)
    {
        this.Heading = InOrientation.Psi;
        this.Pitch = InOrientation.Theta;
        this.Roll = InOrientation.Phi;
    }
};

public struct FPsiThetaPhi
{
    public float Psi;
    public float Theta;
    public float Phi;

    public FPsiThetaPhi(float Psi, float Theta, float Phi)
    {
        this.Psi = Psi;
        this.Theta = Theta;
        this.Phi = Phi;
    }

    public FPsiThetaPhi(Orientation InOrientation)
    {
        this.Psi = InOrientation.Psi;
        this.Theta = InOrientation.Theta;
        this.Phi = InOrientation.Phi;
    }
};