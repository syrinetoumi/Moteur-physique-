using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Corps rigide.
    ///
    /// X(t) = [x, R, P, L]
    ///
    /// dx/dt = P/M
    /// dR/dt = w*R
    /// dP/dt = F
    /// dL/dt = tau
    ///
    /// I(t)^-1 = R Ibody^-1 R^T
    ///
    /// Aucune physique Unity.
    /// </summary>
    public class RigidBody
    {
        public string Name = "RigidBody";

        public readonly float Mass;
        public readonly Shape Shape;

        public bool IsStatic;

        public IntegrationMethod Method =
            IntegrationMethod.Euler;

        public List<IForce> Forces =
            new List<IForce>();

        public List<ForceApplication> AppliedForces =
            new List<ForceApplication>();

        public RigidState State;

        readonly Mat3 Ibody;
        readonly Mat3 IbodyInv;

        public RigidBody(
            Shape shape,
            float mass,
            Vector3 position)
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

        public Vector3 Position
        {
            get
            {
                return State.x;
            }
        }

        public Mat3 Orientation
        {
            get
            {
                return State.R;
            }
        }

        public Vector3 Velocity
        {
            get
            {
                return State.P / Mass;
            }
        }

        public Mat3 InertiaInvWorld
        {
            get
            {
                return State.R *
                       IbodyInv *
                       State.R.Transpose();
            }
        }

        public Mat3 InertiaWorld
        {
            get
            {
                return State.R *
                       Ibody *
                       State.R.Transpose();
            }
        }

        public Vector3 AngularVelocity
        {
            get
            {
                return InertiaInvWorld * State.L;
            }
        }

        public float KineticEnergy
        {
            get
            {
                Vector3 velocity = Velocity;

                float linear =
                    0.5f * Mass *
                    Vec.Dot(velocity, velocity);

                float angular =
                    0.5f *
                    Vec.Dot(AngularVelocity, State.L);

                return linear + angular;
            }
        }

        /// <summary>
        /// Sommets en coordonnees monde.
        /// r_i = R r0_i + x
        /// </summary>
        public Vector3[] WorldVertices()
        {
            Vector3[] local =
                Shape.LocalVertices;

            Vector3[] world =
                new Vector3[local.Length];

            for (int i = 0; i < local.Length; i++)
            {
                world[i] =
                    State.R * local[i] +
                    State.x;
            }

            return world;
        }

        public Triangle3[] WorldTriangles()
        {
            Vector3[] vertices =
                WorldVertices();

            int[] indices =
                Shape.Triangles;

            Triangle3[] triangles =
                new Triangle3[indices.Length / 3];

            for (int i = 0; i < triangles.Length; i++)
            {
                triangles[i] = new Triangle3(
                    vertices[indices[3 * i]],
                    vertices[indices[3 * i + 1]],
                    vertices[indices[3 * i + 2]]
                );
            }

            return triangles;
        }

        public Matrix4x4 ModelMatrix()
        {
            return State.R.ToMatrix4x4(
                State.x,
                Shape.RenderScale
            );
        }

        public void SetVelocity(Vector3 velocity)
        {
            State.P = velocity * Mass;
        }

        public void SetAngularVelocity(
            Vector3 angularVelocity)
        {
            State.L =
                InertiaWorld * angularVelocity;
        }

        public void ApplyForce(
            Vector3 force,
            float duration = float.PositiveInfinity)
        {
            AppliedForces.Add(
                new ForceApplication(
                    force,
                    Vector3.zero,
                    duration
                )
            );
        }

        public void ApplyForceAtPoint(
            Vector3 force,
            Vector3 localPoint,
            float duration = float.PositiveInfinity)
        {
            AppliedForces.Add(
                new ForceApplication(
                    force,
                    localPoint,
                    duration
                )
            );
        }

        public void ClearAppliedForces()
        {
            AppliedForces.Clear();
        }

        public void Step(float dt)
        {
            if (IsStatic || dt <= 0f)
                return;

            if (Method == IntegrationMethod.RK4)
            {
                RigidState s = State;

                RigidState k1 =
                    Derivative(s);

                RigidState k2 =
                    Derivative(
                        s + k1 * (dt * 0.5f)
                    );

                RigidState k3 =
                    Derivative(
                        s + k2 * (dt * 0.5f)
                    );

                RigidState k4 =
                    Derivative(
                        s + k3 * dt
                    );

                State = s +
                    (
                        k1 +
                        k2 * 2f +
                        k3 * 2f +
                        k4
                    ) * (dt / 6f);
            }
            else
            {
                // Euler explicite.
                State =
                    State +
                    Derivative(State) * dt;
            }

            // Correction numerique de R.
            State.R =
                State.R.Orthonormalize();

            for (int i = AppliedForces.Count - 1;
                 i >= 0;
                 i--)
            {
                AppliedForces[i].Elapsed += dt;

                if (!AppliedForces[i].Active)
                {
                    AppliedForces.RemoveAt(i);
                }
            }
        }

        /// <summary>
        /// Resultante des forces et moments.
        /// </summary>
        void ComputeForces(
            RigidState state,
            out Vector3 F,
            out Vector3 tau)
        {
            F = Vector3.zero;
            tau = Vector3.zero;

            Vector3 velocity =
                state.P / Mass;

            for (int i = 0; i < Forces.Count; i++)
            {
                F += Forces[i].Compute(
                    state.x,
                    velocity,
                    Mass
                );
            }

            for (int i = 0;
                 i < AppliedForces.Count;
                 i++)
            {
                ForceApplication application =
                    AppliedForces[i];

                if (!application.Active)
                    continue;

                Vector3 r =
                    state.R *
                    application.LocalPoint;

                F += application.Force;

                tau += Vec.Cross(
                    r,
                    application.Force
                );
            }
        }

        /// <summary>
        /// Derivee de l'etat rigide.
        /// </summary>
        RigidState Derivative(RigidState state)
        {
            Vector3 F;
            Vector3 tau;

            ComputeForces(
                state,
                out F,
                out tau
            );

            Mat3 inverseInertia =
                state.R *
                IbodyInv *
                state.R.Transpose();

            Vector3 angularVelocity =
                inverseInertia * state.L;

            RigidState derivative;

            derivative.x =
                state.P / Mass;

            derivative.R =
                Mat3.Star(angularVelocity) *
                state.R;

            derivative.P = F;
            derivative.L = tau;

            return derivative;
        }
    }
}