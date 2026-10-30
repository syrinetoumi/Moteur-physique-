using UnityEngine;
using UnityEngine.Rendering;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Haithem.ProjetFinal
{
    [ExecuteAlways]
    public class ChateauParticules : MonoBehaviour
    {
        [Range(1f, 500f)] public float masseProjectile = 500f;
        [Range(5f, 60f)] public float vitesseProjectile = 20f;
        [Range(0.01f, 0.5f)] public float resistance = 0.04f;
        [Range(0f, 1f)] public float rebond = 0.45f;
        [Range(0f, 1f)] public float frottement = 0.45f;
        [SerializeField] Mesh sphere;
        [SerializeField] Material materiau;

        public ChateauSimulation Simulation { get; private set; }
        public bool EnPause { get; private set; }
        public int NombreBoules { get { return Simulation == null ? 0 : Simulation.Group.Particles.Count; } }
        Material materiauInstancie;
        Matrix4x4[][] matrices;

        void OnEnable()
        {
            RenderPipelineManager.beginContextRendering += DessinerURP;
            Camera.onPreCull += DessinerStandard;
            Reinitialiser();
        }

        [ContextMenu("Reconstruire le château")]
        public void Reinitialiser()
        {
            if (sphere == null || materiau == null) return;
            if (materiauInstancie == null)
            {
                // Même matériau par défaut, avec le dessin par lots activé.
                materiauInstancie = new Material(materiau);
                materiauInstancie.enableInstancing = true;
                materiauInstancie.hideFlags = HideFlags.HideAndDontSave;
            }
            Simulation = new ChateauSimulation(resistance, rebond, frottement);
            int count = NombreBoules;
            matrices = new Matrix4x4[(count + 1022) / 1023][];
            for (int i = 0; i < matrices.Length; i++)
                matrices[i] = new Matrix4x4[Mathf.Min(1023, count - i * 1023)];
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
            if (!Application.isPlaying) return;
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
            Matrix4x4 local = transform.localToWorldMatrix;
            for (int i = 0; i < NombreBoules; i++)
            {
                var p = Simulation.Group.Particles[i];
                matrices[i / 1023][i % 1023] = local
                    * Matrix4x4.TRS(p.position, Quaternion.identity, Vector3.one * (p.Radius * 2f));
            }
        }

        void DessinerURP(ScriptableRenderContext context, System.Collections.Generic.List<Camera> cameras)
        {
            foreach (Camera camera in cameras) Dessiner(camera);
        }

        void DessinerStandard(Camera camera)
        {
            if (GraphicsSettings.currentRenderPipeline == null) Dessiner(camera);
        }

        void Dessiner(Camera camera)
        {
            if (matrices == null || materiauInstancie == null) return;
            foreach (Matrix4x4[] lot in matrices)
                Graphics.DrawMeshInstanced(sphere, 0, materiauInstancie, lot, lot.Length,
                    null, ShadowCastingMode.On, true, gameObject.layer, camera, LightProbeUsage.Off);
        }

        void OnDisable()
        {
            RenderPipelineManager.beginContextRendering -= DessinerURP;
            Camera.onPreCull -= DessinerStandard;
            if (materiauInstancie != null)
            {
                if (Application.isPlaying) Destroy(materiauInstancie);
                else DestroyImmediate(materiauInstancie);
            }
            materiauInstancie = null;
            matrices = null;
        }
    }
}
