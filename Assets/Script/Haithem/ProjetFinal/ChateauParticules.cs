using PhysicsLib;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Haithem.ProjetFinal
{
    public class ChateauParticules : MonoBehaviour
    {
        [Range(1f, 500f)] public float masseProjectile = 500f;
        [Range(5f, 60f)] public float vitesseProjectile = 20f;
        [Range(0.01f, 0.5f)] public float resistance = 0.04f;
        [Range(0f, 1f)] public float rebond = 0.45f;
        [Range(0f, 1f)] public float frottement = 0.45f;

        public ChateauSimulation Simulation { get; private set; }
        public bool EnPause { get; private set; }
        [SerializeField] Transform[] boules;

        void Start() { Reinitialiser(); }

        [ContextMenu("Reconstruire le château")]
        public void Reinitialiser()
        {
            if (!Application.isPlaying) return;
            Simulation = new ChateauSimulation(resistance, rebond, frottement);
            if (boules == null || boules.Length != Simulation.Group.Particles.Count)
            {
                Nettoyer();
                boules = new Transform[Simulation.Group.Particles.Count];
            }
            for (int i = 0; i < boules.Length; i++)
            {
                if (boules[i] != null)
                {
                    boules[i].localScale = Vector3.one * (Simulation.Group.Particles[i].Radius * 2f);
                    continue;
                }
                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.name = i == Simulation.ProjectileIndex ? "Projectile" : "Boule " + i;
                sphere.transform.SetParent(transform, false);
                // Unity sert seulement à afficher les boules.
                Collider collider = sphere.GetComponent<Collider>();
                collider.enabled = false;
                Destroy(collider);
                float diameter = Simulation.Group.Particles[i].Radius * 2f;
                sphere.transform.localScale = Vector3.one * diameter;
                boules[i] = sphere.transform;
            }
            Afficher();
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
                boules[i].localPosition = Simulation.Group.Particles[i].position;
        }

        void Nettoyer()
        {
            if (boules == null) return;
            foreach (Transform boule in boules)
            {
                if (boule == null) continue;
                boule.gameObject.SetActive(false);
                Destroy(boule.gameObject);
            }
            boules = null;
        }

        void OnDestroy() { Nettoyer(); }
    }
}
