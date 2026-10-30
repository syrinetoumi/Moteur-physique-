using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Haithem.ProjetFinal
{
    public class BlocParticules : MonoBehaviour
    {
        [Range(1f, 500f)] public float masseProjectile = 500f;
        [Range(5f, 60f)] public float vitesseProjectile = 20f;
        [Range(0.01f, 0.5f)] public float resistance = 0.04f;
        [Range(0f, 1f)] public float rebond = 0.45f;
        [Range(0f, 1f)] public float frottement = 0.45f;
        [SerializeField] Mesh sphere;
        [SerializeField] Material materiau;
        [SerializeField] Transform particules;
        [SerializeField] Transform[] boules;

        public BlocSimulation Simulation { get; private set; }
        public bool EnPause { get; private set; }
        public int NombreBoules { get { return Simulation == null ? 0 : Simulation.Group.Particles.Count; } }

        void Start() { Reinitialiser(); }

        [ContextMenu("Reconstruire le bloc")]
        public void Reinitialiser()
        {
            if (sphere == null || materiau == null) return;
            Simulation = new BlocSimulation(resistance, rebond, frottement);
            bool reconstruire = boules == null || boules.Length != NombreBoules || particules == null;
            if (!reconstruire)
                foreach (Transform boule in boules)
                    if (boule == null) { reconstruire = true; break; }
            if (reconstruire) CreerBoules();
            Afficher();
        }

        void CreerBoules()
        {
            if (particules != null) Supprimer(particules.gameObject);
            if (boules != null && boules.Length > 0 && boules[boules.Length - 1] != null)
                Supprimer(boules[boules.Length - 1].gameObject);
            particules = new GameObject("Particules").transform;
            particules.SetParent(transform, false);
            var couches = new Transform[BlocSimulation.Height];
            for (int y = 0; y < couches.Length; y++)
            {
                couches[y] = new GameObject("Couche " + y).transform;
                couches[y].SetParent(particules, false);
            }
            boules = new Transform[NombreBoules];
            for (int i = 0; i < boules.Length; i++)
            {
                bool projectile = i == Simulation.ProjectileIndex;
                var objet = new GameObject(projectile ? "Projectile" : "Particule " + i,
                    typeof(MeshFilter), typeof(MeshRenderer));
                // Maillage natif, sans collider ni Rigidbody.
                objet.GetComponent<MeshFilter>().sharedMesh = sphere;
                objet.GetComponent<MeshRenderer>().sharedMaterial = materiau;
                objet.transform.SetParent(projectile ? transform
                    : couches[i / (BlocSimulation.Width * BlocSimulation.Depth)], false);
                objet.transform.localScale = Vector3.one * (Simulation.Group.Particles[i].Radius * 2f);
                boules[i] = objet.transform;
            }
        }

        static void Supprimer(GameObject objet)
        {
            objet.SetActive(false);
            if (Application.isPlaying) Destroy(objet);
            else DestroyImmediate(objet);
        }

        [ContextMenu("Tirer")]
        public void Tirer()
        {
            if (Simulation == null) return;
            Simulation.ProjectileMass = masseProjectile;
            Simulation.ProjectileSpeed = vitesseProjectile;
            Simulation.Fire();
        }

        public void ChangerPause() { EnPause = !EnPause; }

        public void Avancer(float dt)
        {
            if (Simulation == null || EnPause || dt <= 0f) return;
            Simulation.World.Update(Mathf.Min(dt, 0.1f));
            Afficher();
        }

        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard clavier = Keyboard.current;
            if (clavier != null)
            {
                if (clavier.spaceKey.wasPressedThisFrame) Tirer();
                if (clavier.rKey.wasPressedThisFrame) Reinitialiser();
                if (clavier.pKey.wasPressedThisFrame) ChangerPause();
            }
#else
            if (Input.GetKeyDown(KeyCode.Space)) Tirer();
            if (Input.GetKeyDown(KeyCode.R)) Reinitialiser();
            if (Input.GetKeyDown(KeyCode.P)) ChangerPause();
#endif
            Avancer(Time.deltaTime);
        }

        void Afficher()
        {
            for (int i = 0; i < boules.Length; i++)
                if (boules[i] != null) boules[i].localPosition = Simulation.Group.Particles[i].position;
        }
    }
}
