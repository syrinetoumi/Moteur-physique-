using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Résolution des contacts par IMPULSIONS (sol et briques) :
    ///   - une impulsion j.n change la quantité de mouvement :   P += j.n
    ///   - et le moment angulaire :                              L += r ^ (j.n)     (r = point de contact - centre de masse)
    ///   - j est choisi pour que la vitesse relative au point de contact, selon la normale, ne soit plus rentrante :
    ///         j = -(1+e).vn / ( 1/mA + 1/mB + n.[ (I_A^-1 (rA^n)) ^ rA + (I_B^-1 (rB^n)) ^ rB ] )
    ///   - friction de Coulomb : l'impulsion tangentielle est limitée à mu . j
    ///   - correction de position : on écarte légèrement les corps qui se pénètrent (sans changer leur vitesse).
    /// On répète plusieurs itérations car chaque contact modifie les voisins (piles de briques).
    /// </summary>
    public class ImpulseResolver : ICollisionResolver
    {
        public int Iterations = 10;
        public float Restitution = 0.0f;           // rebond (0 = les briques ne rebondissent pas)
        public float RestitutionThreshold = 0.5f;  // sous cette vitesse d'impact, pas de rebond
        public float Friction = 0.5f;              // coefficient de friction mu
        public float PositionCorrection = 0.2f;    // fraction de la pénétration corrigée à chaque pas
        public float Slop = 0.005f;                // pénétration tolérée (évite le tremblement)

        struct Work
        {
            public RigidBody A, B;                 // B = null : plan fixe
            public Vector3 n, t1, t2, ra, rb;
            public float kN, kT1, kT2, target;
            public float accN, accT1, accT2;
            public float invMa, invMb;
            public Mat3 IA, IB;
        }

        Work[] work = new Work[0];

        static float InvMass(RigidBody b) { return (b == null || b.IsStatic) ? 0f : 1f / b.Mass; }

        static Vector3 PointVelocity(RigidBody b, Vector3 r, float invM, Mat3 Iinv)
        {
            if (b == null || invM == 0f) return Vector3.zero;
            return b.State.P * invM + Vector3.Cross(Iinv * b.State.L, r);
        }

        static void Apply(RigidBody b, float invM, Vector3 r, Vector3 impulse)
        {
            if (b == null || invM == 0f) return;
            b.State.P += impulse;
            b.State.L += Vector3.Cross(r, impulse);
        }

        static float Eff(Vector3 dir, Vector3 r, Mat3 Iinv, float invM)
        {
            if (invM == 0f) return 0f;
            return invM + Vector3.Dot(dir, Vector3.Cross(Iinv * Vector3.Cross(r, dir), r));
        }

        public void Resolve(List<Contact> contacts, float dt)
        {
            int n = contacts.Count;
            if (work.Length < n) work = new Work[System.Math.Max(n, work.Length * 2)];

            // ---- préparation (positions fixes pendant les itérations)
            for (int i = 0; i < n; i++)
            {
                Contact c = contacts[i];
                Work w = new Work();
                w.A = c.A; w.B = c.B;
                w.n = c.normal;
                w.invMa = InvMass(c.A); w.invMb = InvMass(c.B);
                w.IA = w.invMa > 0f ? c.A.InertiaInvWorld : Mat3.Zero;
                w.IB = w.invMb > 0f ? c.B.InertiaInvWorld : Mat3.Zero;
                w.ra = c.point - c.A.Position;
                w.rb = c.B != null ? c.point - c.B.Position : Vector3.zero;

                Vector3 helper = Mathf.Abs(w.n.x) < 0.9f ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
                w.t1 = Vector3.Cross(w.n, helper).normalized;
                w.t2 = Vector3.Cross(w.n, w.t1);

                w.kN = Eff(w.n, w.ra, w.IA, w.invMa) + Eff(w.n, w.rb, w.IB, w.invMb);
                w.kT1 = Eff(w.t1, w.ra, w.IA, w.invMa) + Eff(w.t1, w.rb, w.IB, w.invMb);
                w.kT2 = Eff(w.t2, w.ra, w.IA, w.invMa) + Eff(w.t2, w.rb, w.IB, w.invMb);

                // vitesse normale cible : 0, ou -e.vn0 pour un impact assez violent
                Vector3 vrel0 = PointVelocity(c.A, w.ra, w.invMa, w.IA) - PointVelocity(c.B, w.rb, w.invMb, w.IB);
                float vn0 = Vector3.Dot(vrel0, w.n);
                w.target = (Restitution > 0f && vn0 < -RestitutionThreshold) ? -Restitution * vn0 : 0f;

                work[i] = w;
            }

            // ---- itérations : impulsions normales puis friction
            for (int it = 0; it < Iterations; it++)
            {
                for (int i = 0; i < n; i++)
                {
                    ref Work w = ref work[i];                 // référence : évite de recopier la structure
                    if (w.kN <= 1e-9f) continue;

                    Vector3 vrel = PointVelocity(w.A, w.ra, w.invMa, w.IA) - PointVelocity(w.B, w.rb, w.invMb, w.IB);

                    // normale
                    float vn = Vector3.Dot(vrel, w.n);
                    float lambda = (w.target - vn) / w.kN;
                    float newAcc = Mathf.Max(w.accN + lambda, 0f);          // une impulsion ne peut que repousser
                    lambda = newAcc - w.accN;
                    w.accN = newAcc;
                    Vector3 imp = w.n * lambda;
                    Apply(w.A, w.invMa, w.ra, imp);
                    Apply(w.B, w.invMb, w.rb, -imp);

                    // friction (2 directions tangentes), limitée par mu . impulsion normale
                    float maxF = Friction * w.accN;
                    vrel = PointVelocity(w.A, w.ra, w.invMa, w.IA) - PointVelocity(w.B, w.rb, w.invMb, w.IB);

                    if (w.kT1 > 1e-9f)
                    {
                        float l1 = -Vector3.Dot(vrel, w.t1) / w.kT1;
                        float a1 = Mathf.Max(-maxF, Mathf.Min(maxF, w.accT1 + l1));
                        l1 = a1 - w.accT1; w.accT1 = a1;
                        Apply(w.A, w.invMa, w.ra, w.t1 * l1);
                        Apply(w.B, w.invMb, w.rb, -(w.t1 * l1));
                    }
                    if (w.kT2 > 1e-9f)
                    {
                        float l2 = -Vector3.Dot(PointVelocity(w.A, w.ra, w.invMa, w.IA) - PointVelocity(w.B, w.rb, w.invMb, w.IB), w.t2) / w.kT2;
                        float a2 = Mathf.Max(-maxF, Mathf.Min(maxF, w.accT2 + l2));
                        l2 = a2 - w.accT2; w.accT2 = a2;
                        Apply(w.A, w.invMa, w.ra, w.t2 * l2);
                        Apply(w.B, w.invMb, w.rb, -(w.t2 * l2));
                    }
                }
            }

            // ---- correction de position : on écarte les corps qui se pénètrent
            for (int i = 0; i < n; i++)
            {
                Contact c = contacts[i];
                float invA = InvMass(c.A), invB = InvMass(c.B);
                float sum = invA + invB;
                if (sum == 0f) continue;
                float corr = Mathf.Max(c.penetration - Slop, 0f) * PositionCorrection / sum;
                if (invA > 0f) c.A.State.x += c.normal * (corr * invA);
                if (invB > 0f) c.B.State.x -= c.normal * (corr * invB);
            }
        }
    }
}
