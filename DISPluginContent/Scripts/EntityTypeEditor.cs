using OpenDis.Dis1998;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace GRILLDIS
{
    [System.Serializable]
    public class EntityTypeEditor
    {
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

        public EntityType toEntityType()
        {
            EntityType entityType = new EntityType();
            entityType.Category = category;
            entityType.Country = country;
            entityType.Domain = domain;
            entityType.EntityKind = entityKind;
            entityType.Extra = extra;
            entityType.Specific = specific;
            entityType.Subcategory = subcategory;
            return entityType;
        }

        public void fromEntityType(EntityType EntityTypeIn)
        {
            category = EntityTypeIn.Category;
            country = EntityTypeIn.Country;
            domain = EntityTypeIn.Domain;
            entityKind = EntityTypeIn.EntityKind;
            extra = EntityTypeIn.Extra;
            specific = EntityTypeIn.Specific;
            subcategory = EntityTypeIn.Subcategory;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("(");
            sb.Append(category + DELIMITER_PERIOD);
            sb.Append(country + DELIMITER_PERIOD);
            sb.Append(domain + DELIMITER_PERIOD);
            sb.Append(entityKind + DELIMITER_PERIOD);
            sb.Append(extra + DELIMITER_PERIOD);
            sb.Append(specific + DELIMITER_PERIOD);
            sb.Append(subcategory);
            sb.Append(")");
            return sb.ToString();
        }

    }

}