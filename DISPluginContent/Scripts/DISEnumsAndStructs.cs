using UnityEngine;
using OpenDis.Dis1998;

namespace GRILLDIS
{
    public enum EEarthShape
    {
        RoundEarth,
        FlatEarth
    }

    public enum EConnectionType
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

    public enum EDestroyCode
    {
        DetonationPDU,
        TimeOut,
        Disabled,
        RemovePDU,
        UnityDestroy
    }

    public enum EEntityDamage
    {
        NoDamage,
        SlightDamage,
        ModerateDamage,
        Destroyed
    };

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

    [System.Serializable]
    public struct FLatLonAlt
    {
        public double Latitude;
        public double Longitude;
        public double Altitude;

        public FLatLonAlt(double Lat, double Lon, double Alt)
        {
            Latitude = Lat;
            Longitude = Lon;
            Altitude = Alt;
        }

        /// <summary>
        /// Form a new FLatLonAlt variable from a given Vector3Double.
        /// </summary>
        /// <param name="lla">Represents Latitude (X), Longitude (Y), and Altitude (Z) to convert to an FLatLonAlt struct.</param>
        public FLatLonAlt(Vector3Double lla)
        {
            Latitude = lla.X;
            Longitude = lla.Y;
            Altitude = lla.Z;
        }
    };

    public struct FUTMCoordinates
    {
        public double Easting;
        public double Northing;
        public string Zone;

        public FUTMCoordinates(double UTMEasting, double UTMNorthing, string UTMZone)
        {
            Easting = UTMEasting;
            Northing = UTMNorthing;
            Zone = UTMZone;
        }
    }

    public struct FEntityAppearance
    {

        public bool PaintScheme;
        public bool MobilityKilled;
        public bool FirePowerKilled;
        public EEntityDamage Damage;
        public bool IsSmoking;
        public bool IsEngineSmoking;
        public int Trailing;
        public int HatchState;
        public bool LightPrimary;
        public bool LightSecondary;
        public bool LightCollision;
        public bool IsFlaming;
        public bool IsFrozen;
        public bool IsDeactivated;
        public bool IsLandingGearExtended;
        public int RawVal;

        public FEntityAppearance(int val)
        {
            RawVal = val;
            PaintScheme = getField(val, 0);
            MobilityKilled = getField(val, 1);
            FirePowerKilled = getField(val, 2);
            Damage = (EEntityDamage)getField(val, 0b11, 3);
            IsSmoking = getField(val, 5);
            IsEngineSmoking = getField(val, 6);
            Trailing = getField(val, 0b11, 7);
            HatchState = getField(val, 0b111, 9);
            LightPrimary = getField(val, 12);
            LightSecondary = getField(val, 13);
            LightCollision = getField(val, 14);
            IsFlaming = getField(val, 15);

            IsFrozen = getField(val, 21);
            IsDeactivated = getField(val, 23);

            IsLandingGearExtended = getField(val, 25);
        }

        static int getField(int val, int mask, int pos)
        {
            return (int)((val & (mask << pos)) >> pos);
        }
        static bool getField(int val, int pos)
        {
            return getField(val, 0b1, pos) != 0;
        }

        int UpdateValue()
        {
            RawVal |= PaintScheme ? 1 << 0 : 0 << 0;
            RawVal |= MobilityKilled ? 1 << 1 : 0 << 1;
            RawVal |= FirePowerKilled ? 1 << 2 : 0 << 2;
            RawVal |= (int)Damage << 3;
            RawVal |= IsSmoking ? 1 << 5 : 0 << 5;
            RawVal |= IsEngineSmoking ? 1 << 6 : 0 << 6;
            RawVal |= Trailing << 7;
            RawVal |= HatchState << 9;
            RawVal |= LightPrimary ? 1 << 12 : 0 << 12;
            RawVal |= LightSecondary ? 1 << 13 : 0 << 13;
            RawVal |= LightCollision ? 1 << 14 : 0 << 14;
            RawVal |= IsFlaming ? 1 << 15 : 0 << 15;
            RawVal |= IsFrozen ? 1 << 21 : 0 << 21;
            RawVal |= IsDeactivated ? 1 << 23 : 0 << 23;
            RawVal |= IsLandingGearExtended ? 1 << 25 : 0 << 25;
            return RawVal;
        }

    }
}