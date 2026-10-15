using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Point matériel. Masse <= 0 : particule FIXE (épinglée), elle ne bouge jamais.
    /// "phase" sert à regrouper des particules (ex : toutes celles d'un même objet).
    /// </summary>
    public class Particle
    {
        public Vector3 position;
        public Vector3 velocity;
        public int phase;

        float mass;
        float invMass;

        public Particle(Vector3 position, float mass, int phase = 0)
        {
            this.position = position;
            this.phase = phase;
            Mass = mass;
        }

        public float Mass
        {
            get { return mass; }
            set { mass = value; invMass = value > 0f ? 1f / value : 0f; }
        }

        public float InvMass { get { return invMass; } }
        public bool IsFixed { get { return invMass == 0f; } }
    }
}
