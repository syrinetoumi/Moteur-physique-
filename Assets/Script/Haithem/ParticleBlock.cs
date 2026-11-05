using UnityEngine;

namespace Haithem.ParticleBlock
{
    public class ParticleBlock : MonoBehaviour
    {
        [Range(0.01f, 0.5f)] public float resistance = 0.04f;
        [Range(0f, 1f)] public float rebond = 0.45f;
        [Range(0f, 1f)] public float frottement = 0.45f;
        [SerializeField] Transform groupeParticules;
        [SerializeField] ParticleView[] particules;

        public BlockSimulation Simulation { get; private set; }

        void Awake()
        {
            // Le bloc est prêt avant le Start du projectile.
            Simulation = new BlockSimulation(resistance, rebond, frottement,
                groupeParticules.localScale.x);
            for (int i = 0; i < particules.Length; i++)
                particules[i].Initialiser(Simulation.Group.Particles[i], transform);
        }

        void Update()
        {
            Simulation.World.Update(Mathf.Min(Time.deltaTime, 0.1f));
        }
    }
}
