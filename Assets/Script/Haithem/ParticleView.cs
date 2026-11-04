using PhysicsLib;
using UnityEngine;

namespace Haithem.ParticleBlock
{
    public class ParticleView : MonoBehaviour
    {
        Particle corps;
        Transform repere;

        public void Initialiser(Particle particule, Transform origine)
        {
            corps = particule;
            repere = origine;
        }

        void LateUpdate()
        {
            // La position vient du moteur physique.
            if (corps != null)
                transform.position = repere.TransformPoint(corps.position);
        }
    }
}
