using OpenDis.Dis1998;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;

namespace GRILLDIS
{
    [System.Serializable]
    public class EntityIDEditor
    {
        public ushort site;
        public ushort application;
        public ushort entity;

        public const string DELIMITER_PERIOD = ".";

        public EntityIDEditor()
        {

        }

        public EntityID toEntityID()
        {
            EntityID entityID = new EntityID();
            entityID.Site = site;
            entityID.Application = application;
            entityID.Entity = entity;
            return entityID;
        }

        public void fromEntityID(EntityID EntityIDIn)
        {
            site = EntityIDIn.Site;
            application = EntityIDIn.Application;
            entity = EntityIDIn.Entity;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("(");
            sb.Append(site + DELIMITER_PERIOD);
            sb.Append(application + DELIMITER_PERIOD);
            sb.Append(entity);
            sb.Append(")");
            return sb.ToString();
        }
    }
}