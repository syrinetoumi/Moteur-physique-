using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Détection boîte contre boîte (OBB) : phase large (IBroadPhase) puis phase étroite par le
    /// THÉORÈME DE L'AXE SÉPARATEUR (Chapitre 4) sur les 15 axes :
    ///   3 axes de A + 3 axes de B + 9 produits vectoriels  u_i(A) ^ u_j(B)   (les produits quasi nuls sont ignorés).
    ///   Axe séparateur :   |T.L| > rA + rB     avec T = cB - cA   et   r = somme des (e_k . |u_k.L|)
    /// Si aucun axe ne sépare, il y a collision : l'axe de recouvrement MINIMAL donne la normale n et la profondeur δ.
    ///   - axe d'une face : les points de contact sont obtenus en découpant la face de l'autre boîte par la face de référence ;
    ///   - axe "arête x arête" : le point de contact est le milieu des points les plus proches des deux arêtes.
    /// Convention du Contact : normal = normale dirigée vers A (A est repoussé le long de +n), penetration = δ.
    /// </summary>
    public class BoxBoxContactDetector : ICollisionDetector
    {
        public IBroadPhase BroadPhase = new SweepAndPruneBroadPhase();
        public float LateralTolerance = 0.02f;     // un sommet pile au bord d'une face compte comme "dessus"

        public int PairCount { get; private set; }     // paires candidates de la phase large (statistiques)
        readonly List<int> pairs = new List<int>();

        // ---------------------------------------------------------------- utilitaires
        static Vector3 Axis(Mat3 R, int i) { return i == 0 ? R.c0 : (i == 1 ? R.c1 : R.c2); }
        static float Comp(Vector3 v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

        /// <summary>Rayon de projection d'une OBB sur l'axe L :  r = somme e_k |u_k . L|.</summary>
        static float Radius(Mat3 R, Vector3 e, Vector3 L)
        {
            return e.x * Mathf.Abs(Vector3.Dot(R.c0, L)) + e.y * Mathf.Abs(Vector3.Dot(R.c1, L)) + e.z * Mathf.Abs(Vector3.Dot(R.c2, L));
        }

        struct SatResult
        {
            public bool isEdge;
            public int owner, axis;      // axe de face : propriétaire (0 = A, 1 = B) et indice
            public int ia, ib;           // axe d'arête : u_ia(A) ^ u_ib(B)
            public Vector3 L;            // axe d'arête normalisé, orienté de A vers B
            public float overlap;
        }

        static bool Sat(RigidBody A, BoxShape sa, RigidBody B, BoxShape sb, out SatResult res)
        {
            res = new SatResult();
            Mat3 RA = A.State.R, RB = B.State.R;
            Vector3 eA = sa.Size * 0.5f, eB = sb.Size * 0.5f;
            Vector3 T = B.Position - A.Position;

            // 6 axes de faces
            float bestFace = float.MaxValue; int fo = 0, fk = 0;
            for (int o = 0; o < 2; o++)
                for (int k = 0; k < 3; k++)
                {
                    Vector3 L = Axis(o == 0 ? RA : RB, k);
                    float ov = Radius(RA, eA, L) + Radius(RB, eB, L) - Mathf.Abs(Vector3.Dot(T, L));
                    if (ov < 0f) return false;                           // axe séparateur : pas de collision
                    if (ov < bestFace) { bestFace = ov; fo = o; fk = k; }
                }

            // 9 axes arête x arête
            float bestEdge = float.MaxValue; int bi = 0, bj = 0; Vector3 bL = Vector3.zero;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    Vector3 L = Vector3.Cross(Axis(RA, i), Axis(RB, j));
                    float len2 = L.sqrMagnitude;
                    if (len2 < 1e-6f) continue;                          // arêtes parallèles : axe ignoré
                    L = L / Mathf.Sqrt(len2);
                    float ov = Radius(RA, eA, L) + Radius(RB, eB, L) - Mathf.Abs(Vector3.Dot(T, L));
                    if (ov < 0f) return false;
                    if (ov < bestEdge) { bestEdge = ov; bi = i; bj = j; bL = L; }
                }

            // on préfère une face (contacts plus stables) sauf si une arête est nettement meilleure
            if (bestEdge < 0.95f * bestFace - 0.005f)
            {
                if (Vector3.Dot(bL, T) < 0f) bL = -bL;                   // orienté de A vers B
                res.isEdge = true; res.ia = bi; res.ib = bj; res.L = bL; res.overlap = bestEdge;
            }
            else
            {
                res.isEdge = false; res.owner = fo; res.axis = fk; res.overlap = bestFace;
            }
            return true;
        }

        /// <summary>Profondeur d'interpénétration de deux boîtes (0 si séparées). Utile pour déboguer et pour les tests.</summary>
        public static float PenetrationDepth(RigidBody A, RigidBody B)
        {
            BoxShape sa = A.Shape as BoxShape, sb = B.Shape as BoxShape;
            if (sa == null || sb == null) return 0f;
            SatResult r;
            return Sat(A, sa, B, sb, out r) ? r.overlap : 0f;
        }

        // ---------------------------------------------------------------- détection
        public void Detect(List<RigidBody> bodies, List<Contact> contacts)
        {
            pairs.Clear();
            BroadPhase.FindPairs(bodies, pairs);
            PairCount = pairs.Count / 2;

            for (int k = 0; k < pairs.Count; k += 2)
            {
                RigidBody a = bodies[pairs[k]], b = bodies[pairs[k + 1]];
                BoxShape sa = a.Shape as BoxShape, sb = b.Shape as BoxShape;
                if (sa == null || sb == null) continue;

                SatResult r;
                if (!Sat(a, sa, b, sb, out r)) continue;

                int before = contacts.Count;
                if (r.isEdge) AddEdgeContact(a, sa, b, sb, r, contacts);
                else
                {
                    bool aIsRef = r.owner == 0;
                    AddFaceContacts(aIsRef ? b : a, aIsRef ? a : b, aIsRef ? sb : sa, aIsRef ? sa : sb, r.axis, contacts);
                }

                if (contacts.Count == before)                            // dernier recours : un contact au milieu
                {
                    Vector3 nTo = a.Position - b.Position;
                    Contact c;
                    c.A = a; c.B = b;
                    c.normal = nTo.sqrMagnitude > 1e-8f ? nTo.normalized : Vector3.up;
                    c.point = 0.5f * (a.Position + b.Position);
                    c.penetration = r.overlap;
                    contacts.Add(c);
                }
            }
        }

        // ---------------------------------------------------------------- contact arête x arête
        void AddEdgeContact(RigidBody A, BoxShape sa, RigidBody B, BoxShape sb, SatResult r, List<Contact> contacts)
        {
            Mat3 RA = A.State.R, RB = B.State.R;
            Vector3 eA = sa.Size * 0.5f, eB = sb.Size * 0.5f;

            // arête de A la plus avancée vers B (direction L), arête de B la plus avancée vers A (direction -L)
            Vector3 pA = A.Position, pB = B.Position;
            for (int k = 0; k < 3; k++)
            {
                if (k != r.ia) { Vector3 u = Axis(RA, k); pA += u * (Comp(eA, k) * (Vector3.Dot(u, r.L) >= 0f ? 1f : -1f)); }
                if (k != r.ib) { Vector3 u = Axis(RB, k); pB += u * (Comp(eB, k) * (Vector3.Dot(u, r.L) >= 0f ? -1f : 1f)); }
            }
            Vector3 d1 = Axis(RA, r.ia), d2 = Axis(RB, r.ib);

            // points les plus proches de deux droites
            Vector3 w = pA - pB;
            float b = Vector3.Dot(d1, d2), d = Vector3.Dot(d1, w), e = Vector3.Dot(d2, w);
            float den = 1f - b * b;
            float t1 = den > 1e-8f ? (b * e - d) / den : 0f;
            float t2 = den > 1e-8f ? (e - b * d) / den : 0f;
            Vector3 qA = pA + d1 * t1, qB = pB + d2 * t2;

            Contact c;
            c.A = A; c.B = B;
            c.normal = -r.L;                 // dirigée de B vers A
            c.point = 0.5f * (qA + qB);
            c.penetration = r.overlap;
            contacts.Add(c);
        }

        // ---------------------------------------------------------------- contacts face : découpage
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
                    if (da > 0f) output[m++] = a + (b - a) * (da / (da - db));     // a dehors, b dedans
                    output[m++] = b;
                }
                else if (da <= 0f)
                {
                    output[m++] = a + (b - a) * (da / (da - db));                   // a dedans, b dehors
                }
            }
            return m;
        }

        void AddFaceContacts(RigidBody inc, RigidBody rf, BoxShape incShape, BoxShape rfShape, int axis,
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