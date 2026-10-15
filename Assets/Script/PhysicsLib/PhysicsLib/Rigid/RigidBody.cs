using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Corps rigide (Chapitre 2 partie 2). Équations de mouvement :
    ///   dx/dt = v = P/M      dR/dt = w* . R      dP/dt = F (somme des forces)      dL/dt = tau (somme des moments)
    ///   I(t)^-1 = R . Ibody^-1 . R^T            w(t) = I(t)^-1 . L(t)
    /// Aucune physique Unity : tout est calculé ici.
    /// </summary>
    public class RigidBody
    {
        public string Name = "RigidBody";
        public readonly float Mass;
        public readonly Shape Shape;
        public bool IsStatic;                                   // un corps statique ne bouge jamais
        public IntegrationMethod Method = IntegrationMethod.Euler;

        /// <summary>Forces s'appliquant au centre de masse (gravité, frottements...).</summary>
        public List<IForce> Forces = new List<IForce>();

        /// <summary>Forces ponctuelles (créent un moment si elles ne passent pas par le centre de masse).</summary>
        public List<ForceApplication> AppliedForces = new List<ForceApplication>();

        public RigidState State;

        readonly Mat3 Ibody;
        readonly Mat3 IbodyInv;

        public RigidBody(Shape shape, float mass, Vector3 position)
        {
            Shape = shape;
            Mass = mass;
            Ibody = shape.InertiaBody(mass);
            IbodyInv = Ibody.Inverse();

            State.x = position;
            State.R = Mat3.Identity;
            State.P = Vector3.zero;
            State.L = Vector3.zero;
        }

        // ----------------------------------------------------------------- lecture
        public Vector3 Position { get { return State.x; } }
        public Mat3 Orientation { get { return State.R; } }
        public Vector3 Velocity { get { return State.P / Mass; } }

        public Mat3 InertiaInvWorld { get { return State.R * IbodyInv * State.R.Transpose(); } }
        public Mat3 InertiaWorld { get { return State.R * Ibody * State.R.Transpose(); } }
        public Vector3 AngularVelocity { get { return InertiaInvWorld * State.L; } }

        public float KineticEnergy
        {
            get
            {
                Vector3 v = Velocity;
                return 0.5f * Mass * Vector3.Dot(v, v) + 0.5f * Vector3.Dot(AngularVelocity, State.L);
            }
        }

        /// <summary>Sommets du corps en repère monde.</summary>
        public Vector3[] WorldVertices()
        {
            Vector3[] lv = Shape.LocalVertices;
            Vector3[] w = new Vector3[lv.Length];
            for (int i = 0; i < lv.Length; i++) w[i] = State.R * lv[i] + State.x;
            return w;
        }

        /// <summary>Triangles du corps en repère monde (pour les tests de rayons).</summary>
        public Triangle3[] WorldTriangles()
        {
            Vector3[] w = WorldVertices();
            int[] t = Shape.Triangles;
            Triangle3[] res = new Triangle3[t.Length / 3];
            for (int i = 0; i < res.Length; i++)
                res[i] = new Triangle3(w[t[3 * i]], w[t[3 * i + 1]], w[t[3 * i + 2]]);
            return res;
        }

        /// <summary>Matrice 4x4 pour l'affichage (CustomCube.ApplyTransformation).</summary>
        public Matrix4x4 ModelMatrix() { return State.R.ToMatrix4x4(State.x, Shape.RenderScale); }

        // ----------------------------------------------------------------- commandes
        public void SetVelocity(Vector3 v) { State.P = v * Mass; }

        public void SetAngularVelocity(Vector3 w) { State.L = InertiaWorld * w; }

        /// <summary>Force au centre de masse pendant "duration" secondes.</summary>
        public void ApplyForce(Vector3 force, float duration = float.PositiveInfinity)
        { AppliedForces.Add(new ForceApplication(force, Vector3.zero, duration)); }

        /// <summary>Force en un point (repère de l'objet, relatif au centre de masse).</summary>
        public void ApplyForceAtPoint(Vector3 force, Vector3 localPoint, float duration = float.PositiveInfinity)
        { AppliedForces.Add(new ForceApplication(force, localPoint, duration)); }

        public void ClearAppliedForces() { AppliedForces.Clear(); }

        // ----------------------------------------------------------------- simulation
        /// <summary>Avance la simulation de dt secondes (Euler ou RK4 selon Method).</summary>
        public void Step(float dt)
        {
            if (IsStatic) return;

            if (Method == IntegrationMethod.RK4)
            {
                RigidState s = State;
                RigidState k1 = Derivative(s);
                RigidState k2 = Derivative(s + k1 * (dt * 0.5f));
                RigidState k3 = Derivative(s + k2 * (dt * 0.5f));
                RigidState k4 = Derivative(s + k3 * dt);
                State = s + (k1 + k2 * 2f + k3 * 2f + k4) * (dt / 6f);
            }
            else
            {
                State = State + Derivative(State) * dt;     // X(t+dt) = X(t) + dt . dX/dt   (slide 42)
            }

            State.R = State.R.Orthonormalize();             // évite la dérive numérique de R (slide 36)

            for (int i = AppliedForces.Count - 1; i >= 0; i--)
            {
                AppliedForces[i].Elapsed += dt;
                if (!AppliedForces[i].Active) AppliedForces.RemoveAt(i);
            }
        }

        /// <summary>Résultante F et moment tau pour un état donné (slide 37).</summary>
        void ComputeForces(RigidState s, out Vector3 F, out Vector3 tau)
        {
            F = Vector3.zero;
            tau = Vector3.zero;
            Vector3 v = s.P / Mass;

            for (int i = 0; i < Forces.Count; i++)
                F += Forces[i].Compute(s.x, v, Mass);

            for (int i = 0; i < AppliedForces.Count; i++)
            {
                ForceApplication fa = AppliedForces[i];
                if (!fa.Active) continue;
                Vector3 r = s.R * fa.LocalPoint;            // r_i - x(t) en repère global
                F += fa.Force;
                tau += Vector3.Cross(r, fa.Force);
            }
        }

        /// <summary>dX/dt = ( v(t) , w*(t).R(t) , F(t) , tau(t) )   (slides 38 et 41).</summary>
        RigidState Derivative(RigidState s)
        {
            Vector3 F, tau;
            ComputeForces(s, out F, out tau);

            Mat3 Iinv = s.R * IbodyInv * s.R.Transpose();
            Vector3 w = Iinv * s.L;

            RigidState d;
            d.x = s.P / Mass;
            d.R = Mat3.Star(w) * s.R;
            d.P = F;
            d.L = tau;
            return d;
        }
    }
}
