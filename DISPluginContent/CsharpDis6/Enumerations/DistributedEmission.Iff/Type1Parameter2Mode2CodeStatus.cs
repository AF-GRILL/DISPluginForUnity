// Copyright 2008-2011. This work is licensed under the BSD license, available at
// http://www.movesinstitute.org/licenses
//
// Orignal authors: DMcG, Jason Nelson
// Modified for use with C#:
// - Peter Smith (Naval Air Warfare Center - Training Systems Division)
// - Zvonko Bostjancic (Blubit d.o.o. - zvonko.bostjancic@blubit.si)

using System;
using System.ComponentModel;
using System.Diagnostics.CodeAnalysis;
using System.Reflection;

namespace OpenDis.Enumerations.DistributedEmission.Iff
{
    /// <summary>
    /// Enumeration values for Type1Parameter2Mode2CodeStatus (der.iff.type.1.fop.param2, Parameter 2 - Mode 2 Code/Status, 
    /// section 8.3.1.2.3)
    /// The enumeration values are generated from the SISO DIS XML EBV document (R35), which was
    /// obtained from http://discussions.sisostds.org/default.asp?action=10&amp;fd=31
    /// </summary>
    [SuppressMessage("Microsoft.Naming", "CA1702:CompoundWordsShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
    [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", Justification = "Due to SISO standardized naming.")]
    [SuppressMessage("Microsoft.Naming", "CA1707:IdentifiersShouldNotContainUnderscores", Justification = "Due to SISO standardized naming.")]
    [SuppressMessage("Microsoft.Naming", "CA1709:IdentifiersShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
    [Serializable]
    public struct Type1Parameter2Mode2CodeStatus
    {
        /// <summary>
        /// Status
        /// </summary>
        [SuppressMessage("Microsoft.Naming", "CA1702:CompoundWordsShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1707:IdentifiersShouldNotContainUnderscores", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1709:IdentifiersShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
        [Description("Status")]
        public enum StatusValue : uint
        {
            /// <summary>
            /// Off
            /// </summary>
            Off = 0,

            /// <summary>
            /// On
            /// </summary>
            On = 1
        }

        /// <summary>
        /// Damage
        /// </summary>
        [SuppressMessage("Microsoft.Naming", "CA1702:CompoundWordsShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1707:IdentifiersShouldNotContainUnderscores", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1709:IdentifiersShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
        [Description("Damage")]
        public enum DamageValue : uint
        {
            /// <summary>
            /// No damage
            /// </summary>
            NoDamage = 0,

            /// <summary>
            /// Damage
            /// </summary>
            Damage = 1
        }

        /// <summary>
        /// Malfunction
        /// </summary>
        [SuppressMessage("Microsoft.Naming", "CA1702:CompoundWordsShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1707:IdentifiersShouldNotContainUnderscores", Justification = "Due to SISO standardized naming.")]
        [SuppressMessage("Microsoft.Naming", "CA1709:IdentifiersShouldBeCasedCorrectly", Justification = "Due to SISO standardized naming.")]
        [Description("Malfunction")]
        public enum MalfunctionValue : uint
        {
            /// <summary>
            /// No malfunction
            /// </summary>
            NoMalfunction = 0,

            /// <summary>
            /// Malfunction
            /// </summary>
            Malfunction = 1
        }

        private byte codeElement1;
        private byte codeElement2;
        private byte codeElement3;
        private byte codeElement4;
        private Type1Parameter2Mode2CodeStatus.StatusValue status;
        private Type1Parameter2Mode2CodeStatus.DamageValue damage;
        private Type1Parameter2Mode2CodeStatus.MalfunctionValue malfunction;

        /// <summary>
        /// Implements the operator !=.
        /// </summary>
        /// <param name="left">The left operand.</param>
        /// <param name="right">The right operand.</param>
        /// <returns>
        /// 	<c>true</c> if operands are not equal; otherwise, <c>false</c>.
        /// </returns>
        public static bool operator !=(Type1Parameter2Mode2CodeStatus left, Type1Parameter2Mode2CodeStatus right)
        {
            return !(left == right);
        }

        /// <summary>
        /// Implements the operator ==.
        /// </summary>
        /// <param name="left">The left operand.</param>
        /// <param name="right">The right operand.</param>
        /// <returns>
        /// 	<c>true</c> if operands are not equal; otherwise, <c>false</c>.
        /// </returns>
        public static bool operator ==(Type1Parameter2Mode2CodeStatus left, Type1Parameter2Mode2CodeStatus right)
        {
            if (object.ReferenceEquals(left, right))
            {
                return true;
            }

            // If parameters are null return false (cast to object to prevent recursive loop!)
            if (((object)left == null) || ((object)right == null))
            {
                return false;
            }

            return left.Equals(right);
        }

        /// <summary>
        /// Performs an explicit conversion from <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> to <see cref="System.UInt16"/>.
        /// </summary>
        /// <param name="obj">The <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> scheme instance.</param>
        /// <returns>The result of the conversion.</returns>
        public static explicit operator ushort(Type1Parameter2Mode2CodeStatus obj)
        {
            return obj.ToUInt16();
        }

        /// <summary>
        /// Performs an explicit conversion from <see cref="System.UInt16"/> to <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/>.
        /// </summary>
        /// <param name="value">The ushort value.</param>
        /// <returns>The result of the conversion.</returns>
        public static explicit operator Type1Parameter2Mode2CodeStatus(ushort value)
        {
            return Type1Parameter2Mode2CodeStatus.FromUInt16(value);
        }

        /// <summary>
        /// Creates the <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance from the byte array.
        /// </summary>
        /// <param name="array">The array which holds the values for the <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/>.</param>
        /// <param name="index">The starting position within value.</param>
        /// <returns>The <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance, represented by a byte array.</returns>
        /// <exception cref="ArgumentNullException">if the <c>array</c> is null.</exception>
        /// <exception cref="IndexOutOfRangeException">if the <c>index</c> is lower than 0 or greater or equal than number of elements in array.</exception>
        public static Type1Parameter2Mode2CodeStatus FromByteArray(byte[] array, int index)
        {
            if (array == null)
            {
                throw new ArgumentNullException("array");
            }

            if (index < 0 ||
                index > array.Length - 1 ||
                index + 2 > array.Length - 1)
            {
                throw new IndexOutOfRangeException();
            }

            return FromUInt16(BitConverter.ToUInt16(array, index));
        }

        /// <summary>
        /// Creates the <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance from the ushort value.
        /// </summary>
        /// <param name="value">The ushort value which represents the <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance.</param>
        /// <returns>The <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance, represented by the ushort value.</returns>
        public static Type1Parameter2Mode2CodeStatus FromUInt16(ushort value)
        {
            Type1Parameter2Mode2CodeStatus ps = new Type1Parameter2Mode2CodeStatus();

            uint mask0 = 0x0007;
            byte shift0 = 0;
            uint newValue0 = (value & mask0) >> shift0;
            ps.CodeElement1 = (byte)newValue0;

            uint mask1 = 0x0038;
            byte shift1 = 3;
            uint newValue1 = (value & mask1) >> shift1;
            ps.CodeElement2 = (byte)newValue1;

            uint mask2 = 0x01c0;
            byte shift2 = 6;
            uint newValue2 = (value & mask2) >> shift2;
            ps.CodeElement3 = (byte)newValue2;

            uint mask3 = 0x0e00;
            byte shift3 = 9;
            uint newValue3 = (value & mask3) >> shift3;
            ps.CodeElement4 = (byte)newValue3;

            uint mask5 = 0x2000;
            byte shift5 = 13;
            uint newValue5 = (value & mask5) >> shift5;
            ps.Status = (Type1Parameter2Mode2CodeStatus.StatusValue)newValue5;

            uint mask6 = 0x4000;
            byte shift6 = 14;
            uint newValue6 = (value & mask6) >> shift6;
            ps.Damage = (Type1Parameter2Mode2CodeStatus.DamageValue)newValue6;

            uint mask7 = 0x8000;
            byte shift7 = 15;
            uint newValue7 = (value & mask7) >> shift7;
            ps.Malfunction = (Type1Parameter2Mode2CodeStatus.MalfunctionValue)newValue7;

            return ps;
        }

        /// <summary>
        /// Gets or sets the codeelement1.
        /// </summary>
        /// <value>The codeelement1.</value>
        public byte CodeElement1
        {
            get { return this.codeElement1; }
            set { this.codeElement1 = value; }
        }

        /// <summary>
        /// Gets or sets the codeelement2.
        /// </summary>
        /// <value>The codeelement2.</value>
        public byte CodeElement2
        {
            get { return this.codeElement2; }
            set { this.codeElement2 = value; }
        }

        /// <summary>
        /// Gets or sets the codeelement3.
        /// </summary>
        /// <value>The codeelement3.</value>
        public byte CodeElement3
        {
            get { return this.codeElement3; }
            set { this.codeElement3 = value; }
        }

        /// <summary>
        /// Gets or sets the codeelement4.
        /// </summary>
        /// <value>The codeelement4.</value>
        public byte CodeElement4
        {
            get { return this.codeElement4; }
            set { this.codeElement4 = value; }
        }

        /// <summary>
        /// Gets or sets the status.
        /// </summary>
        /// <value>The status.</value>
        public Type1Parameter2Mode2CodeStatus.StatusValue Status
        {
            get { return this.status; }
            set { this.status = value; }
        }

        /// <summary>
        /// Gets or sets the damage.
        /// </summary>
        /// <value>The damage.</value>
        public Type1Parameter2Mode2CodeStatus.DamageValue Damage
        {
            get { return this.damage; }
            set { this.damage = value; }
        }

        /// <summary>
        /// Gets or sets the malfunction.
        /// </summary>
        /// <value>The malfunction.</value>
        public Type1Parameter2Mode2CodeStatus.MalfunctionValue Malfunction
        {
            get { return this.malfunction; }
            set { this.malfunction = value; }
        }

        /// <summary>
        /// Determines whether the specified <see cref="System.Object"/> is equal to this instance.
        /// </summary>
        /// <param name="obj">The <see cref="System.Object"/> to compare with this instance.</param>
        /// <returns>
        /// 	<c>true</c> if the specified <see cref="System.Object"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
            {
                return false;
            }

            if (!(obj is Type1Parameter2Mode2CodeStatus))
            {
                return false;
            }

            return this.Equals((Type1Parameter2Mode2CodeStatus)obj);
        }

        /// <summary>
        /// Determines whether the specified <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance is equal to this instance.
        /// </summary>
        /// <param name="other">The <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance to compare with this instance.</param>
        /// <returns>
        /// 	<c>true</c> if the specified <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public bool Equals(Type1Parameter2Mode2CodeStatus other)
        {
            // If parameter is null return false (cast to object to prevent recursive loop!)
            if ((object)other == null)
            {
                return false;
            }

            return
                this.CodeElement1 == other.CodeElement1 &&
                this.CodeElement2 == other.CodeElement2 &&
                this.CodeElement3 == other.CodeElement3 &&
                this.CodeElement4 == other.CodeElement4 &&
                this.Status == other.Status &&
                this.Damage == other.Damage &&
                this.Malfunction == other.Malfunction;
        }

        /// <summary>
        /// Converts the instance of <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> to the byte array.
        /// </summary>
        /// <returns>The byte array representing the current <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance.</returns>
        public byte[] ToByteArray()
        {
            return BitConverter.GetBytes(this.ToUInt16());
        }

        /// <summary>
        /// Converts the instance of <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> to the ushort value.
        /// </summary>
        /// <returns>The ushort value representing the current <see cref="OpenDis.Enumerations.DistributedEmission.Iff.Type1Parameter2Mode2CodeStatus"/> instance.</returns>
        public ushort ToUInt16()
        {
            ushort val = 0;

            val |= (ushort)((uint)this.CodeElement1 << 0);
            val |= (ushort)((uint)this.CodeElement2 << 3);
            val |= (ushort)((uint)this.CodeElement3 << 6);
            val |= (ushort)((uint)this.CodeElement4 << 9);
            val |= (ushort)((uint)this.Status << 13);
            val |= (ushort)((uint)this.Damage << 14);
            val |= (ushort)((uint)this.Malfunction << 15);

            return val;
        }

        /// <summary>
        /// Returns a hash code for this instance.
        /// </summary>
        /// <returns>
        /// 	A hash code for this instance, suitable for use in hashing algorithms and data structures like a hash table.
        /// </returns>
        public override int GetHashCode()
        {
            int hash = 17;

            // Overflow is fine, just wrap
            unchecked
            {
                hash = (hash * 29) + this.CodeElement1.GetHashCode();
                hash = (hash * 29) + this.CodeElement2.GetHashCode();
                hash = (hash * 29) + this.CodeElement3.GetHashCode();
                hash = (hash * 29) + this.CodeElement4.GetHashCode();
                hash = (hash * 29) + this.Status.GetHashCode();
                hash = (hash * 29) + this.Damage.GetHashCode();
                hash = (hash * 29) + this.Malfunction.GetHashCode();
            }

            return hash;
        }
    }
}
