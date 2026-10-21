using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Force appliquee a un point materiel.
    /// </summary>
    public interface IForce
    {
        Vector3 Compute(
            Vector3 position,
            Vector3 velocity,
            float mass
        );
    }

    /// <summary>
    /// Force entre plusieurs particules.
    /// </summary>
    public interface IPairForce
    {
        void Accumulate(
            Vector3[] x,
            Vector3[] v,
            Vector3[] f
        );
    }

    /// <summary>
    /// Gravite : F = m * g.
    /// </summary>
    public class GravityForce : IForce
    {
        public Vector3 g =
            new Vector3(0f, -9.81f, 0f);

        public GravityForce()
        {
        }

        public GravityForce(Vector3 gravity)
        {
            g = gravity;
        }

        public Vector3 Compute(
            Vector3 position,
            Vector3 velocity,
            float mass)
        {
            return g * mass;
        }
    }

    /// <summary>
    /// Frottement visqueux : F = -k * v.
    /// </summary>
    public class ViscousDragForce : IForce
    {
        public float k = 1f;

        public ViscousDragForce()
        {
        }

        public ViscousDragForce(float coefficient)
        {
            k = coefficient;
        }

        public Vector3 Compute(
            Vector3 position,
            Vector3 velocity,
            float mass)
        {
            return -k * velocity;
        }
    }

    /// <summary>
    /// Force constante.
    ///
    /// Utiliser PhysicsLib.ConstantForce
    /// si UnityEngine est egalement importe.
    /// </summary>
    public class ConstantForce : IForce
    {
        public Vector3 force;

        public ConstantForce(Vector3 force)
        {
            this.force = force;
        }

        public Vector3 Compute(
            Vector3 position,
            Vector3 velocity,
            float mass)
        {
            return force;
        }
    }

    /// <summary>
    /// Ressort amorti entre deux particules.
    ///
    /// F = k(L-L0)u + c(vrel.u)u
    ///
    /// Chapitre 2, Partie 1.
    /// </summary>
    public class SpringForce : IPairForce
    {
        public int i;
        public int j;

        public float k;
        public float restLength;
        public float damping;

        public SpringForce(
            int i,
            int j,
            float stiffness,
            float restLength,
            float damping)
        {
            this.i = i;
            this.j = j;

            k = stiffness;

            this.restLength = restLength;
            this.damping = damping;
        }

        public void Accumulate(
            Vector3[] x,
            Vector3[] v,
            Vector3[] f)
        {
            Vector3 d = x[j] - x[i];

            float length = Vec.Length(d);

            if (length < Vec.Epsilon)
                return;

            Vector3 direction = d / length;

            float stretch = length - restLength;

            Vector3 relativeVelocity =
                v[j] - v[i];

            float relativeSpeed =
                Vec.Dot(relativeVelocity, direction);

            Vector3 forceOnI =
                direction *
                (
                    k * stretch +
                    damping * relativeSpeed
                );

            f[i] += forceOnI;
            f[j] -= forceOnI;
        }
    }
}