using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Force qui ne dépend que de l'état d'UN point matériel (position, vitesse, masse).
    /// Pour ajouter une nouvelle force au projet : créer une classe qui implémente IForce,
    /// puis l'ajouter à la liste Forces d'un RigidBody / ParticleGroup / PhysicsWorld.
    /// (Principe du cours : "liste d'objets (forces) à invoquer".)
    /// </summary>
    public interface IForce
    {
        Vector3 Compute(Vector3 position, Vector3 velocity, float mass);
    }

    /// <summary>Force qui relie plusieurs particules entre elles (ressorts...). Elle ajoute ses forces dans f.</summary>
    public interface IPairForce
    {
        void Accumulate(Vector3[] x, Vector3[] v, Vector3[] f);
    }

    /// <summary>Poids : F = m.g</summary>
    public class GravityForce : IForce
    {
        public Vector3 g = new Vector3(0f, -9.81f, 0f);

        public GravityForce() { }
        public GravityForce(Vector3 g) { this.g = g; }

        public Vector3 Compute(Vector3 position, Vector3 velocity, float mass) => g * mass;
    }

    /// <summary>Frottement visqueux : F = -k.v   (TP 1).</summary>
    public class ViscousDragForce : IForce
    {
        public float k = 1f;

        public ViscousDragForce() { }

        public ViscousDragForce(float k) { this.k = k; }

        public Vector3 Compute(Vector3 position, Vector3 velocity, float mass) => -k * velocity;
    }

    /// <summary>Force constante (vent, poussée...).</summary>
    public class ConstantForce : IForce
    {
        public Vector3 force;

        public ConstantForce(Vector3 force) { this.force = force; }

        public Vector3 Compute(Vector3 position, Vector3 velocity, float mass) => force;
    }

    /// <summary>
    /// Ressort amorti entre les particules i et j :
    /// F = k.(L - L0).u + c.(vrel.u).u   (loi de Hooke + amortissement, Chapitre 2 partie 1).
    /// </summary>
    public class SpringForce : IPairForce
    {
        public int i, j;
        public float k;            // raideur
        public float restLength;   // longueur au repos L0
        public float damping;      // amortissement c

        public SpringForce(int i, int j, float k, float restLength, float damping)
        { this.i = i; this.j = j; this.k = k; this.restLength = restLength; this.damping = damping; }

        public void Accumulate(Vector3[] x, Vector3[] v, Vector3[] f)
        {
            Vector3 d = x[j] - x[i];
            float len = d.magnitude;
            if (len < 1e-8f) return;
            Vector3 dir = d / len;

            float stretch = len - restLength;
            float relVel = Vector3.Dot(v[j] - v[i], dir);
            Vector3 fi = dir * (k * stretch + damping * relVel);   // force sur i, dirigée vers j

            f[i] += fi;
            f[j] -= fi;
        }
    }
}
