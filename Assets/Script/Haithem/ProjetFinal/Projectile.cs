using UnityEngine;

namespace Haithem.ProjetFinal
{
    [RequireComponent(typeof(Particule))]
    public class Projectile : MonoBehaviour
    {
        public BlocParticules bloc;
        [Min(0.1f)] public float masse = 500f;
        [Min(0f)] public float vitesse = 20f;
        public Vector3 cible = new Vector3(0f, 3.5f, 0f);

        void Start()
        {
            // Le tir démarre automatiquement.
            var corps = bloc.Simulation.LancerProjectile(
                bloc.transform.InverseTransformPoint(transform.position),
                cible, transform.localScale.x * 0.5f, masse, vitesse);
            GetComponent<Particule>().Initialiser(corps, bloc.transform);
        }
    }
}
