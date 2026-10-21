using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Détection boîte contre boîte : un sommet d'une boîte situé À L'INTÉRIEUR de l'autre donne un contact.
    ///   1) on passe le sommet dans le repère de l'autre boîte :  p_local = R^T . (p - x)
    ///   2) il est dedans si |p_local| &lt;= demi-taille sur les 3 axes
    ///   3) la normale est celle de la face la plus proche (profondeur de pénétration minimale)
    /// Convention du Contact : A = corps qui possède le sommet, B = corps dont on touche la face,
    /// normal = normale sortante de la face de B (donc dirigée vers A).
    /// Limite : les contacts arête contre arête (briques en croix) ne sont pas détectés.
    /// </summary>
    public class BoxBoxContactDetector : ICollisionDetector
    {
        public void Detect(List<RigidBody> bodies, List<Contact> contacts)
        {
            int n = bodies.Count;
            Vector3[][] worldVerts = new Vector3[n][];          // sommets calculés à la demande, une seule fois par pas

            for (int i = 0; i < n; i++)
            {
                BoxShape sa = bodies[i].Shape as BoxShape;
                if (sa == null) continue;
                for (int j = i + 1; j < n; j++)
                {
                    BoxShape sb = bodies[j].Shape as BoxShape;
                    if (sb == null) continue;
                    if (bodies[i].IsStatic && bodies[j].IsStatic) continue;

                    // filtre rapide : sphères englobantes
                    float ra = sa.Size.magnitude * 0.5f, rb = sb.Size.magnitude * 0.5f;
                    if ((bodies[i].Position - bodies[j].Position).sqrMagnitude > (ra + rb) * (ra + rb)) continue;

                    if (worldVerts[i] == null) worldVerts[i] = bodies[i].WorldVertices();
                    if (worldVerts[j] == null) worldVerts[j] = bodies[j].WorldVertices();

                    VerticesInside(bodies[i], worldVerts[i], bodies[j], sb, contacts);   // sommets de i dans j
                    VerticesInside(bodies[j], worldVerts[j], bodies[i], sa, contacts);   // sommets de j dans i
                }
            }
        }

        static void VerticesInside(RigidBody owner, Vector3[] verts, RigidBody faceBody, BoxShape faceShape,
                                   List<Contact> contacts)
        {
            Mat3 Rt = faceBody.State.R.Transpose();
            Vector3 half = faceShape.Size * 0.5f;

            for (int k = 0; k < verts.Length; k++)
            {
                Vector3 p = Rt * (verts[k] - faceBody.Position);        // repère local de faceBody
                float dx = half.x - Mathf.Abs(p.x);
                float dy = half.y - Mathf.Abs(p.y);
                float dz = half.z - Mathf.Abs(p.z);
                if (dx < 0f || dy < 0f || dz < 0f) continue;            // dehors

                // face la plus proche = axe de pénétration minimale
                Vector3 nLocal; float depth;
                if (dx <= dy && dx <= dz) { depth = dx; nLocal = new Vector3(p.x >= 0f ? 1f : -1f, 0f, 0f); }
                else if (dy <= dz)        { depth = dy; nLocal = new Vector3(0f, p.y >= 0f ? 1f : -1f, 0f); }
                else                      { depth = dz; nLocal = new Vector3(0f, 0f, p.z >= 0f ? 1f : -1f); }

                Contact c;
                c.A = owner;
                c.B = faceBody;
                c.point = verts[k];
                c.normal = faceBody.State.R * nLocal;
                c.penetration = depth;
                contacts.Add(c);
            }
        }
    }
}
