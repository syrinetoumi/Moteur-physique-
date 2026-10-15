using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Contacts "point-plan" (Chapitre 3) : chaque sommet d'un corps qui passe de l'autre côté
    /// d'un plan fixe (sol, mur) donne un contact. Distance point-plan = n.P + D.
    /// </summary>
    public class PlaneContactDetector : ICollisionDetector
    {
        public List<Plane3> Planes = new List<Plane3>();

        public PlaneContactDetector() { }
        public PlaneContactDetector(params Plane3[] planes) { Planes.AddRange(planes); }

        public void Detect(List<RigidBody> bodies, List<Contact> contacts)
        {
            for (int b = 0; b < bodies.Count; b++)
            {
                RigidBody body = bodies[b];
                if (body.IsStatic) continue;
                Vector3[] verts = body.WorldVertices();

                for (int p = 0; p < Planes.Count; p++)
                    for (int i = 0; i < verts.Length; i++)
                    {
                        float d = Planes[p].SignedDistance(verts[i]);
                        if (d >= 0f) continue;
                        Contact c;
                        c.A = body; c.B = null;
                        c.point = verts[i];
                        c.normal = Planes[p].n;
                        c.penetration = -d;
                        contacts.Add(c);
                    }
            }
        }
    }
}
