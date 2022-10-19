using OpenDis.Dis1998;
using System;
using System.Text;
using UnityEngine;

public static class PDUUtil
{
    public const string DELIMITER_PERIOD = ".";
    public static UInt64 EntityIDToUInt64(EntityID entityID)
    {
        ushort site = entityID.Site;
        ushort application = entityID.Application;
        ushort entity = entityID.Entity;
        UInt64 entityIDU64 = ThreeUShortsToUInt64(site, application, entity);

        return entityIDU64;
    }

    public static UInt64 ThreeUShortsToUInt64(ushort low, ushort mid, ushort high)
    {
        return ((UInt64)mid << 16) | ((UInt64)high << 32) | (UInt64)low;
    }

    public static UInt64 EntityTypeToUInt64(EntityType entityType)
    {
        byte category = entityType.Category;
        ushort country = entityType.Country;
        byte domain = entityType.Domain;
        byte entityKind = entityType.EntityKind;
        byte extra = entityType.Extra;
        byte specific = entityType.Specific;
        byte subcategory = entityType.Subcategory;

        UInt64 entityType64 = ((UInt64)subcategory << 56) | ((UInt64)specific << 48) | ((UInt64)extra << 40) | ((UInt64)entityKind << 32) |
                              ((UInt64)domain << 24)      | ((UInt64)country << 8)   | (UInt64)category;
        //For debug
        //string binary = UInt64ToString(entityType64);
        return entityType64;
    }

    public static string UInt64ToString(UInt64 uInt64)
    {
        string binary = Convert.ToString((long)uInt64, 2).PadLeft(64, '0');

        return binary;
    }
    public static string getMarkingAsString(EntityStatePdu entityStatePdu)
    {
        byte[] markingCharacters = entityStatePdu.Marking.Characters;
        string markingString = System.Text.Encoding.Default.GetString(markingCharacters);
        int length = (markingString.Length < 12) ? markingString.Length : 11;
        markingString = markingString.Substring(0, length);
        return markingString;
    }
    public static Marking getStringAsMarking(string stringToConvert)
    {
        Marking marking = new Marking();

        //Set character set to ASCII and encode appropriately
        marking.CharacterSet = 1;
        byte[] stringBytes = Encoding.ASCII.GetBytes(stringToConvert);

        int count = (stringBytes.Length < 11) ? stringBytes.Length : 11;
        for(int i = 0; i < count; i++)
        {
            marking.Characters[i] = stringBytes[i];
        }

        return marking;
    }

    public static string getEntityStatePDUMarkingSAE(EntityStatePdu entityStatePdu)
    {
        string marking = getMarkingAsString(entityStatePdu);

        ushort site = entityStatePdu.EntityID.Site;
        ushort application = entityStatePdu.EntityID.Application;
        ushort entity = entityStatePdu.EntityID.Entity;

        StringBuilder sb = new StringBuilder();
        sb.Append(marking + " (");
        sb.Append(site + DELIMITER_PERIOD);
        sb.Append(application + DELIMITER_PERIOD);
        sb.Append(entity + ")");
        
        return sb.ToString();
    }

    public static string getEntityTypeAsString(EntityStatePdu entityStatePdu)
    {
        EntityType entityType = entityStatePdu.EntityType;
        
        StringBuilder sb = new StringBuilder();
        sb.Append(entityType.Category + DELIMITER_PERIOD);
        sb.Append(entityType.Country + DELIMITER_PERIOD);
        sb.Append(entityType.Domain + DELIMITER_PERIOD);
        sb.Append(entityType.EntityKind + DELIMITER_PERIOD);
        sb.Append(entityType.Extra + DELIMITER_PERIOD);
        sb.Append(entityType.Specific + DELIMITER_PERIOD);
        sb.Append(entityType.Subcategory);
        return sb.ToString();
    }
    
    public static EntityStatePdu DeepCopyEntityStatePDU(EntityStatePdu EntityStatePDUIn)
    {
        EntityStatePdu entityStatePDUOut = new EntityStatePdu();

        entityStatePDUOut.AlternativeEntityType = EntityStatePDUIn.AlternativeEntityType;
        entityStatePDUOut.ArticulationParameters.AddRange(EntityStatePDUIn.ArticulationParameters);
        entityStatePDUOut.Capabilities = EntityStatePDUIn.Capabilities;
        entityStatePDUOut.DeadReckoningParameters = EntityStatePDUIn.DeadReckoningParameters;
        entityStatePDUOut.EntityAppearance = EntityStatePDUIn.EntityAppearance;
        entityStatePDUOut.EntityID = EntityStatePDUIn.EntityID;
        entityStatePDUOut.EntityLinearVelocity = EntityStatePDUIn.EntityLinearVelocity;

        entityStatePDUOut.EntityLocation = new Vector3Double
        {
            X = EntityStatePDUIn.EntityLocation.X,
            Y = EntityStatePDUIn.EntityLocation.Y,
            Z = EntityStatePDUIn.EntityLocation.Z
        };

        entityStatePDUOut.EntityOrientation = new Orientation
        {
            Psi = EntityStatePDUIn.EntityOrientation.Psi,
            Theta = EntityStatePDUIn.EntityOrientation.Theta,
            Phi = EntityStatePDUIn.EntityOrientation.Phi
        };

        entityStatePDUOut.EntityType = EntityStatePDUIn.EntityType;
        entityStatePDUOut.ExerciseID = EntityStatePDUIn.ExerciseID;
        entityStatePDUOut.ForceId = EntityStatePDUIn.ForceId;
        entityStatePDUOut.Length = EntityStatePDUIn.Length;
        entityStatePDUOut.Marking = EntityStatePDUIn.Marking;
        entityStatePDUOut.NumberOfArticulationParameters = EntityStatePDUIn.NumberOfArticulationParameters;
        entityStatePDUOut.Padding = EntityStatePDUIn.Padding;
        entityStatePDUOut.PduType = EntityStatePDUIn.PduType;
        entityStatePDUOut.ProtocolFamily = EntityStatePDUIn.ProtocolFamily;
        entityStatePDUOut.ProtocolVersion = EntityStatePDUIn.ProtocolVersion;
        entityStatePDUOut.Timestamp = EntityStatePDUIn.Timestamp;

        return entityStatePDUOut;
    }
}
