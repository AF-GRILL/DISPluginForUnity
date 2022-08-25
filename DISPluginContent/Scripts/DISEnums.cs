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