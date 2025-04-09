using OpenDis.Dis1998;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;

namespace GRILLDIS
{
    [System.Serializable]
    public class EntityTypeEditor
    {
        public bool useSpecific_EntityKind = true;
        public bool useSpecific_Domain = true;
        public bool useSpecific_Country = true;
        public bool useSpecific_Category = true;
        public bool useSpecific_Subcategory = true;
        public bool useSpecific_Specific = true;
        public bool useSpecific_Extra = true;

        public byte entityKind;
        public byte domain;
        public ushort country;
        public byte category;
        public byte subcategory;
        public byte specific;
        public byte extra;

        public const string DELIMITER_PERIOD = ".";

        public EntityTypeEditor()
        {

        }

        public EntityTypeEditor(EntityType EntityTypeIn)
        {
            entityKind = EntityTypeIn.EntityKind;
            domain = EntityTypeIn.Domain;
            country = EntityTypeIn.Country;
            category = EntityTypeIn.Category;
            subcategory = EntityTypeIn.Subcategory;
            specific = EntityTypeIn.Specific;
            extra = EntityTypeIn.Extra;
        }

        public EntityType toEntityType()
        {
            EntityType entityType = new EntityType();
            entityType.EntityKind = entityKind;
            entityType.Domain = domain;
            entityType.Country = country;
            entityType.Category = category;
            entityType.Subcategory = subcategory;
            entityType.Specific = specific;
            entityType.Extra = extra;
            return entityType;
        }

        public void fromEntityType(EntityType EntityTypeIn)
        {
            entityKind = EntityTypeIn.EntityKind;
            domain = EntityTypeIn.Domain;
            country = EntityTypeIn.Country;
            category = EntityTypeIn.Category;
            subcategory = EntityTypeIn.Subcategory;
            specific = EntityTypeIn.Specific;
            extra = EntityTypeIn.Extra;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();

            sb.Append("(");
            sb.Append((useSpecific_EntityKind ? entityKind.ToString() : "*") + DELIMITER_PERIOD);
            sb.Append((useSpecific_Domain ? domain.ToString() : "*") + DELIMITER_PERIOD);
            sb.Append((useSpecific_Country ? country.ToString() : "*") + DELIMITER_PERIOD);
            sb.Append((useSpecific_Category ? category.ToString() : "*") + DELIMITER_PERIOD);
            sb.Append((useSpecific_Subcategory ? subcategory.ToString() : "*") + DELIMITER_PERIOD);
            sb.Append((useSpecific_Specific ? specific.ToString() : "*") + DELIMITER_PERIOD);
            sb.Append((useSpecific_Extra ? extra.ToString() : "*"));

            sb.Append(")");
            return sb.ToString();
        }

        public EntityTypeEditor FillWildcards(EntityTypeEditor WildcardFiller)
        {
            //Deep copy so we don't change this instance
            EntityTypeEditor filledEntityType = DeepCopyEntityType();

            if (!filledEntityType.useSpecific_EntityKind)
            {
                filledEntityType.entityKind = WildcardFiller.entityKind;
                filledEntityType.useSpecific_EntityKind = !filledEntityType.useSpecific_EntityKind;
            }
            if (!filledEntityType.useSpecific_Domain)
            {
                filledEntityType.domain = WildcardFiller.domain;
                filledEntityType.useSpecific_Domain = !filledEntityType.useSpecific_Domain;
            }
            if (!filledEntityType.useSpecific_Country)
            {
                filledEntityType.country = WildcardFiller.country;
                filledEntityType.useSpecific_Country = !filledEntityType.useSpecific_Country;
            }
            if (!filledEntityType.useSpecific_Category)
            {
                filledEntityType.category = WildcardFiller.category;
                filledEntityType.useSpecific_Category = !filledEntityType.useSpecific_Category;
            }
            if (!filledEntityType.useSpecific_Subcategory)
            {
                filledEntityType.subcategory = WildcardFiller.subcategory;
                filledEntityType.useSpecific_Subcategory = !filledEntityType.useSpecific_Subcategory;
            }
            if (!filledEntityType.useSpecific_Specific)
            {
                filledEntityType.specific = WildcardFiller.specific;
                filledEntityType.useSpecific_Specific = !filledEntityType.useSpecific_Specific;
            }
            if (!filledEntityType.useSpecific_Extra)
            {
                filledEntityType.extra = WildcardFiller.extra;
                filledEntityType.useSpecific_Extra = !filledEntityType.useSpecific_Extra;
            }

            return filledEntityType;
        }

        public EntityTypeEditor DeepCopyEntityType()
        {
            EntityTypeEditor deepCopy = new EntityTypeEditor();

            deepCopy.entityKind = entityKind;
            deepCopy.useSpecific_EntityKind = useSpecific_EntityKind;
            deepCopy.domain = domain;
            deepCopy.useSpecific_Domain = useSpecific_Domain;
            deepCopy.country = country;
            deepCopy.useSpecific_Country = useSpecific_Country;
            deepCopy.category = category;
            deepCopy.useSpecific_Category = useSpecific_Category;
            deepCopy.subcategory = subcategory;
            deepCopy.useSpecific_Subcategory = useSpecific_Subcategory;
            deepCopy.specific = specific;
            deepCopy.useSpecific_Specific = useSpecific_Specific;
            deepCopy.extra = extra;
            deepCopy.useSpecific_Extra = useSpecific_Extra;

            return deepCopy;
        }

        public bool HasWildcards()
        {
            return !useSpecific_EntityKind || !useSpecific_Domain || !useSpecific_Country || !useSpecific_Category
                || !useSpecific_Subcategory || !useSpecific_Specific || !useSpecific_Extra;
        }

        public static bool operator ==(EntityTypeEditor lhs, EntityTypeEditor rhs)
        {
            if (ReferenceEquals(lhs, rhs))
                return true;
            if (ReferenceEquals(lhs, null))
                return false;
            if (ReferenceEquals(rhs, null))
                return false;
            return lhs.Equals(rhs);
        }
        public static bool operator !=(EntityTypeEditor lhs, EntityTypeEditor rhs) => !(lhs == rhs);

        public override bool Equals(object obj)
        {
            return Equals(obj as EntityTypeEditor);
        }

        public override int GetHashCode()
        {
            string entityTypeString = ToString();
            return entityTypeString.GetHashCode();
        }

        public bool Equals(EntityTypeEditor Other)
        {
            if(ReferenceEquals(Other, null))
            {
                return false;
            }
            if (ReferenceEquals(this, Other))
            {
                return true;
            }
            return EntityTypeFieldEqualTo(entityKind, Other.entityKind, useSpecific_EntityKind, Other.useSpecific_EntityKind)
                && EntityTypeFieldEqualTo(domain, Other.domain, useSpecific_Domain, Other.useSpecific_Domain)
                && EntityTypeFieldEqualTo(country, Other.country, useSpecific_Country, Other.useSpecific_Country)
                && EntityTypeFieldEqualTo(category, Other.category, useSpecific_Category, Other.useSpecific_Category)
                && EntityTypeFieldEqualTo(subcategory, Other.subcategory, useSpecific_Subcategory, Other.useSpecific_Subcategory)
                && EntityTypeFieldEqualTo(specific, Other.specific, useSpecific_Specific, Other.useSpecific_Specific)
                && EntityTypeFieldEqualTo(extra, Other.extra, useSpecific_Extra, Other.useSpecific_Extra);
        }

        public static bool operator <(EntityTypeEditor lhs, EntityTypeEditor rhs)
        {
            if (ReferenceEquals(lhs, rhs))
                return false;
            if (ReferenceEquals(lhs, null))
                return false;
            if (ReferenceEquals(rhs, null))
                return false;
            return lhs.LessThan(rhs);
        }
        public static bool operator >(EntityTypeEditor lhs, EntityTypeEditor rhs) => (rhs < lhs);
        public static bool operator <=(EntityTypeEditor lhs, EntityTypeEditor rhs) => !(lhs > rhs);
        public static bool operator >=(EntityTypeEditor lhs, EntityTypeEditor rhs) => !(lhs < rhs);

        public bool LessThan(EntityTypeEditor Other)
        {
            bool bIsLessThan = false;

            //Compare each field individually. If the fields are equal, move to the next field. We can stop comparing once a non-equal field is found. Compare fields starting with most significant
            //Entity Kind
            if (!EntityTypeFieldEqualTo(entityKind, Other.entityKind, useSpecific_EntityKind, Other.useSpecific_EntityKind))
            {
                bIsLessThan = EntityTypeFieldLessThan(entityKind, Other.entityKind, useSpecific_EntityKind, Other.useSpecific_EntityKind);
            }
            //Domain
            else if (!EntityTypeFieldEqualTo(domain, Other.domain, useSpecific_Domain, Other.useSpecific_Domain))
            {
                bIsLessThan = EntityTypeFieldLessThan(domain, Other.domain, useSpecific_Domain, Other.useSpecific_Domain);
            }
            //Country
            else if (!EntityTypeFieldEqualTo(country, Other.country, useSpecific_Country, Other.useSpecific_Country))
            {
                bIsLessThan = EntityTypeFieldLessThan(country, Other.country, useSpecific_Country, Other.useSpecific_Country);
            }
            //Category
            else if (!EntityTypeFieldEqualTo(category, Other.category, useSpecific_Category, Other.useSpecific_Category))
            {
                bIsLessThan = EntityTypeFieldLessThan(category, Other.category, useSpecific_Category, Other.useSpecific_Category);
            }
            //Subcategory
            else if (!EntityTypeFieldEqualTo(subcategory, Other.subcategory, useSpecific_Subcategory, Other.useSpecific_Subcategory))
            {
                bIsLessThan = EntityTypeFieldLessThan(subcategory, Other.subcategory, useSpecific_Subcategory, Other.useSpecific_Subcategory);
            }
            //Specific
            else if (!EntityTypeFieldEqualTo(specific, Other.specific, useSpecific_Specific, Other.useSpecific_Specific))
            {
                bIsLessThan = EntityTypeFieldLessThan(specific, Other.specific, useSpecific_Specific, Other.useSpecific_Specific);
            }
            //Extra
            else if (!EntityTypeFieldEqualTo(extra, Other.extra, useSpecific_Extra, Other.useSpecific_Extra))
            {
                bIsLessThan = EntityTypeFieldLessThan(extra, Other.extra, useSpecific_Extra, Other.useSpecific_Extra);
            }

            //Otherwise all are equal
            return bIsLessThan;
        }

        bool EntityTypeFieldLessThan(ushort lhs, ushort rhs, bool lhsUseSpecific, bool rhsUseSpecific)
	    {
		    //Only less than if both specific values and lhs < rhs OR lhs is wildcard and rhs is specific
		    return (lhsUseSpecific && rhsUseSpecific && lhs<rhs) || (!lhsUseSpecific && rhsUseSpecific);
	    }

        bool EntityTypeFieldGreaterThan(ushort lhs, ushort rhs, bool lhsUseSpecific, bool rhsUseSpecific)
	    {
		    //Only greater than if both specific values and lhs > rhs OR lhs is specific and rhs is wildcard
		    return (lhsUseSpecific && rhsUseSpecific && lhs > rhs) || (lhsUseSpecific && !rhsUseSpecific);
	    }

        bool EntityTypeFieldEqualTo(ushort lhs, ushort rhs, bool lhsUseSpecific, bool rhsUseSpecific)
	    {
		    //Only greater than if both specific values and lhs > rhs OR lhs and rhs are both wildcard
		    return (lhsUseSpecific && rhsUseSpecific && lhs == rhs) || (!lhsUseSpecific && !rhsUseSpecific);
	    }
    }

    [CustomPropertyDrawer(typeof(EntityTypeEditor))]
    public class EntityTypePropertyDrawer : PropertyDrawer
    {
        SerializedProperty entityKind, domain, country, category, subcategory, specific, extra;
        SerializedProperty useSpecific_EntityKind, useSpecific_Domain, useSpecific_Country, useSpecific_Category, useSpecific_Subcategory, useSpecific_Specific, useSpecific_Extra;

        private Rect customPosition;
        private int lineHeight;
        private int marginBetweenFields;

        //Draw the property inside the given rect
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            //Setup contents sizes
            lineHeight = (int)EditorGUIUtility.singleLineHeight;
            position.height = lineHeight;
            marginBetweenFields = (int)EditorGUIUtility.standardVerticalSpacing;

            //Draw foldout label
            property.isExpanded = EditorGUI.Foldout(position, property.isExpanded, label);

            //Draw contents container
            EditorGUI.indentLevel += 1;
            //position.x = 106;
            position.y += lineHeight + marginBetweenFields;
            customPosition = EditorGUI.IndentedRect(position);
            customPosition.height = lineHeight;

            //Set contents
            entityKind = property.FindPropertyRelative("entityKind");
            useSpecific_EntityKind = property.FindPropertyRelative("useSpecific_EntityKind");
            domain = property.FindPropertyRelative("domain");
            useSpecific_Domain = property.FindPropertyRelative("useSpecific_Domain");
            country = property.FindPropertyRelative("country");
            useSpecific_Country = property.FindPropertyRelative("useSpecific_Country");
            category = property.FindPropertyRelative("category");
            useSpecific_Category = property.FindPropertyRelative("useSpecific_Category");
            subcategory = property.FindPropertyRelative("subcategory");
            useSpecific_Subcategory = property.FindPropertyRelative("useSpecific_Subcategory");
            specific = property.FindPropertyRelative("specific");
            useSpecific_Specific = property.FindPropertyRelative("useSpecific_Specific");
            extra = property.FindPropertyRelative("extra");
            useSpecific_Extra = property.FindPropertyRelative("useSpecific_Extra");

            //Draw fields
            if (property.isExpanded)
            {
                AddNewProperty(entityKind, useSpecific_EntityKind);
                AddNewProperty(domain, useSpecific_Domain);
                AddNewProperty(country, useSpecific_Country);
                AddNewProperty(category, useSpecific_Category);
                AddNewProperty(subcategory, useSpecific_Subcategory);
                AddNewProperty(specific, useSpecific_Specific);
                AddNewProperty(extra, useSpecific_Extra, false);
            }
            //Reset indent level
            EditorGUI.indentLevel -= 1;
        }

        private void AddNewProperty(SerializedProperty newProperty, SerializedProperty associatedCheckbox, bool spaceBelow = true)
        {
            Rect tempRect = customPosition;
            EditorGUI.PropertyField(tempRect, associatedCheckbox, GUIContent.none);

            //Add offset so text doesn't overlap the checkbox
            tempRect.x += EditorGUI.GetPropertyHeight(associatedCheckbox);
            //Remove from the end so field entry length doesn't go outside of bounds
            tempRect.xMax -= EditorGUI.GetPropertyHeight(associatedCheckbox);

            EditorGUI.BeginDisabledGroup(!associatedCheckbox.boolValue);
            EditorGUI.PropertyField(tempRect, newProperty);
            EditorGUI.EndDisabledGroup();

            int addY = (int)EditorGUIUtility.singleLineHeight;
            if (spaceBelow)
            {
                addY += marginBetweenFields;
            }

            customPosition.y += addY;
        }

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            return (property.isExpanded) ? (lineHeight + marginBetweenFields) * 8 : (lineHeight + marginBetweenFields);
        }
    }
}