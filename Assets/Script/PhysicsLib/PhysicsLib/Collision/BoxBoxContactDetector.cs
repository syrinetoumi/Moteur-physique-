using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Détection boîte contre boîte par AXES SÉPARATEURS (SAT) sur les 6 normales des faces.
    ///   1) Pour chaque axe n (3 de A, 3 de B), on projette les deux boîtes sur n :
    ///         recouvrement = rA + rB - |d.n|     avec  r = somme des (demi-taille_i . |n.axe_i|)
    ///      Si un recouvrement est négatif, les boîtes sont séparées : pas de contact.
    ///   2) L'axe de recouvrement MINIMAL donne la normale du contact et la boîte "de référence" (celle dont c'est la face).
    ///   3) On prend la face de l'autre boîte qui lui fait face, on la DÉCOUPE par les bords de la face de référence
    ///      (Sutherland-Hodgman), et les points obtenus qui sont passés derrière la face de référence
    ///      deviennent les points de contact (jusqu'à 8). Ça marche pour tous les recouvrements, même partiels ou en croix.
    /// Convention du Contact : A = boîte qui possède le sommet, B = boîte de référence,
    /// normal = normale sortante de la face de B (dirigée vers A), penetration = profondeur du sommet.
    /// Limite : les contacts arête contre arête (briques posées sur la tranche, en croix) ne sont pas traités.
    /// </summary>
    public class BoxBoxContactDetector : ICollisionDetector
    {
        public float LateralTolerance = 0.02f;     // un sommet pile au bord d'une face compte comme "dessus"

        static Vector3 Axis(Mat3 R, int i) { return i == 0 ? R.c0 : (i == 1 ? R.c1 : R.c2); }
        static float Comp(Vector3 v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

        /// <summary>Axe de séparation de recouvrement minimal. Faux si les boîtes sont séparées.</summary>
        static bool Sat(RigidBody A, BoxShape sa, RigidBody B, BoxShape sb,
                        out int owner, out int axis, out float overlap)
        {
            owner = 0; axis = 0; overlap = float.MaxValue;
            Mat3 RA = A.State.R, RB = B.State.R;
            Vector3 hA = sa.Size * 0.5f, hB = sb.Size * 0.5f;
            Vector3 d = A.Position - B.Position;

            for (int o = 0; o < 2; o++)
                for (int k = 0; k < 3; k++)
                {
                    Vector3 n = Axis(o == 0 ? RA : RB, k);
                    float rA = hA.x * Mathf.Abs(Vector3.Dot(n, RA.c0)) + hA.y * Mathf.Abs(Vector3.Dot(n, RA.c1)) + hA.z * Mathf.Abs(Vector3.Dot(n, RA.c2));
                    float rB = hB.x * Mathf.Abs(Vector3.Dot(n, RB.c0)) + hB.y * Mathf.Abs(Vector3.Dot(n, RB.c1)) + hB.z * Mathf.Abs(Vector3.Dot(n, RB.c2));
                    float ov = rA + rB - Mathf.Abs(Vector3.Dot(d, n));
                    if (ov < 0f) return false;                                   // axe séparateur trouvé
                    if (ov < overlap) { overlap = ov; owner = o; axis = k; }
                }
            return true;
        }

        /// <summary>Profondeur d'interpénétration de deux boîtes (0 si elles sont séparées). Utile pour déboguer.</summary>
        public static float PenetrationDepth(RigidBody A, RigidBody B)
        {
            BoxShape sa = A.Shape as BoxShape, sb = B.Shape as BoxShape;
            if (sa == null || sb == null) return 0f;
            int o, k; float ov;
            return Sat(A, sa, B, sb, out o, out k, out ov) ? ov : 0f;
        }

        public void Detect(List<RigidBody> bodies, List<Contact> contacts)
        {
            int n = bodies.Count;

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

                    int owner, axis; float overlap;
                    if (!Sat(bodies[i], sa, bodies[j], sb, out owner, out axis, out overlap)) continue;

                    int refIdx = owner == 0 ? i : j;            // boîte dont on touche la face
                    int incIdx = owner == 0 ? j : i;            // boîte dont les sommets pénètrent
                    BoxShape refShape = owner == 0 ? sa : sb;
                    BoxShape incShape = owner == 0 ? sb : sa;

                    AddFaceContacts(bodies[incIdx], bodies[refIdx], refShape, incShape, axis, contacts);
                }
            }
        }

        static readonly Vector3[] bufA = new Vector3[16];
        static readonly Vector3[] bufB = new Vector3[16];

        /// <summary>Découpe (Sutherland-Hodgman) le polygone "input" pour ne garder que sign*coord(axe) &lt;= limite.</summary>
        static int ClipPlane(Vector3[] input, int n, Vector3[] output, int axisIdx, float sign, float limit)
        {
            int m = 0;
            for (int i = 0; i < n; i++)
            {
                Vector3 a = input[i], b = input[(i + 1) % n];
                float da = sign * Comp(a, axisIdx) - limit;
                float db = sign * Comp(b, axisIdx) - limit;
                if (db <= 0f)
                {
                    if (da > 0f) output[m++] = a + (b - a) * (da / (da - db));     // a dehors, b dedans : intersection
                    output[m++] = b;
                }
                else if (da <= 0f)
                {
                    output[m++] = a + (b - a) * (da / (da - db));                   // a dedans, b dehors : intersection
                }
            }
            return m;
        }

        void AddFaceContacts(RigidBody inc, RigidBody rf, BoxShape rfShape, BoxShape incShape, int axis,
                             List<Contact> contacts)
        {
            Mat3 Rr = rf.State.R, Ri = inc.State.R;
            Vector3 nAxis = Axis(Rr, axis);
            float sign = Vector3.Dot(inc.Position - rf.Position, nAxis) >= 0f ? 1f : -1f;
            Vector3 nOut = nAxis * sign;                         // normale sortante de la face de rf, vers inc

            // face de inc la plus opposée à nOut
            int m = 0; float bestDot = -1f;
            for (int k = 0; k < 3; k++)
            {
                float dk = Mathf.Abs(Vector3.Dot(Axis(Ri, k), nOut));
                if (dk > bestDot) { bestDot = dk; m = k; }
            }
            Vector3 um = Axis(Ri, m);
            float sm = Vector3.Dot(um, nOut) > 0f ? -1f : 1f;
            Vector3 hi = incShape.Size * 0.5f;
            Vector3 center = inc.Position + um * (sm * Comp(hi, m));
            int m1 = (m + 1) % 3, m2 = (m + 2) % 3;
            Vector3 e1 = Axis(Ri, m1) * Comp(hi, m1), e2 = Axis(Ri, m2) * Comp(hi, m2);

            // les 4 coins de la face, exprimés dans le repère de rf
            Mat3 Rt = Rr.Transpose();
            bufA[0] = Rt * (center + e1 + e2 - rf.Position);
            bufA[1] = Rt * (center - e1 + e2 - rf.Position);
            bufA[2] = Rt * (center - e1 - e2 - rf.Position);
            bufA[3] = Rt * (center + e1 - e2 - rf.Position);

            Vector3 half = rfShape.Size * 0.5f;
            float hk = Comp(half, axis);
            int i1 = (axis + 1) % 3, i2 = (axis + 2) % 3;

            // découpe par les 4 bords de la face de référence
            int n = ClipPlane(bufA, 4, bufB, i1, 1f, Comp(half, i1) + LateralTolerance);
            if (n > 0) n = ClipPlane(bufB, n, bufA, i1, -1f, Comp(half, i1) + LateralTolerance);
            if (n > 0) n = ClipPlane(bufA, n, bufB, i2, 1f, Comp(half, i2) + LateralTolerance);
            if (n > 0) n = ClipPlane(bufB, n, bufA, i2, -1f, Comp(half, i2) + LateralTolerance);

            for (int k = 0; k < n; k++)
            {
                Vector3 p = bufA[k];
                float depth = hk - sign * Comp(p, axis);          // > 0 : le point est passé derrière la face
                if (depth < 0f) continue;

                Contact c;
                c.A = inc;
                c.B = rf;
                c.point = rf.Position + Rr * p;
                c.normal = nOut;
                c.penetration = depth;
                contacts.Add(c);
            }
        }
    }
}