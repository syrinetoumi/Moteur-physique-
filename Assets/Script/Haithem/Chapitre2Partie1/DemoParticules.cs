using System.Collections.Generic;
using PhysicsLib;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Haithem.Chapitre2Partie1
{
    public class DemoParticules : MonoBehaviour
    {
        public Transform sol;
        [Range(0.005f, 0.1f)] public float pas = 0.05f;
        [Min(0.1f)] public float masse = 1f;
        [Min(0f)] public float frottement = 1f;
        [Min(0.1f)] public float raideur = 20f;
        [Min(0f)] public float amortissement = 1.5f;

        PhysicsWorld monde;
        ParticleGroup euler, rk4, ressorts;
        readonly List<CustomCube> cubes = new List<CustomCube>();
        bool exempleRessorts;
        bool pause = true;
        float masseDepart, frottementDepart;
        const float hauteur = 8f;

        public PhysicsWorld Monde { get { return monde; } }
        public bool EnPause { get { return pause; } }

        void Start() { Reinitialiser(); }

        public void ChoisirExemple(bool avecRessorts)
        {
            exempleRessorts = avecRessorts;
            Reinitialiser();
        }

        public void Reinitialiser()
        {
            Nettoyer();
            masseDepart = Mathf.Max(0.1f, masse);
            frottementDepart = Mathf.Max(0f, frottement);
            monde = new PhysicsWorld();
            monde.FixedDt = Mathf.Clamp(pas, 0.005f, 0.1f);
            pause = true;

            if (exempleRessorts)
            {
                ressorts = ExperiencesParticules.CreerRessorts(masseDepart,
                    Mathf.Max(0.1f, raideur), Mathf.Max(0f, amortissement));
                monde.AddParticleGroup(ressorts);
                AjouterCube("Point fixe");
                AjouterCube("Masse 1");
                AjouterCube("Masse 2");
                AjouterCube("Ressort 1");
                AjouterCube("Ressort 2");
            }
            else
            {
                euler = ExperiencesParticules.CreerChute(new EulerExplicite(), -3f, hauteur, masseDepart, frottementDepart);
                rk4 = ExperiencesParticules.CreerChute(new RK4Integrator(), 0f, hauteur, masseDepart, frottementDepart);
                monde.AddParticleGroup(euler);
                monde.AddParticleGroup(rk4);
                AjouterCube("Euler");
                AjouterCube("RK4");
                AjouterCube("Solution exacte");
            }

            Camera camera = Camera.main;
            if (camera != null)
            {
                camera.orthographic = false;
                camera.fieldOfView = 60f;
                camera.transform.position = exempleRessorts ? new Vector3(0f, 7f, -10f) : new Vector3(0f, 10f, -23f);
                camera.transform.LookAt(new Vector3(0f, exempleRessorts ? 4f : -2f, 0f));
            }
            // Le sol reste sous les cubes, sans gérer de collision ici.
            if (sol != null) sol.position = new Vector3(0f, exempleRessorts ? 0f : -13f, 0f);
            Afficher();
        }

        void AjouterCube(string nom)
        {
            GameObject objet = new GameObject(nom);
            CustomCube cube = objet.AddComponent<CustomCube>();
            cubes.Add(cube);
        }

        public void ChangerPause() { pause = !pause; }

        public void Avancer(float dt)
        {
            if (pause || monde == null || dt <= 0f) return;

            // La chute s'arrête après 3 secondes pour rester visible.
            if (!exempleRessorts)
                dt = Mathf.Min(dt, Mathf.Max(0f, 3f - monde.Time));
            monde.Update(dt);
            if (!exempleRessorts && monde.Time >= 3f - monde.FixedDt * 0.5f)
                pause = true;
            Afficher();
        }

        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard clavier = Keyboard.current;
            if (clavier != null)
            {
                if (clavier.digit1Key.wasPressedThisFrame) ChoisirExemple(false);
                if (clavier.digit2Key.wasPressedThisFrame) ChoisirExemple(true);
                if (clavier.rKey.wasPressedThisFrame) Reinitialiser();
                if (clavier.spaceKey.wasPressedThisFrame) ChangerPause();
            }
#else
            if (Input.GetKeyDown(KeyCode.Alpha1)) ChoisirExemple(false);
            if (Input.GetKeyDown(KeyCode.Alpha2)) ChoisirExemple(true);
            if (Input.GetKeyDown(KeyCode.R)) Reinitialiser();
            if (Input.GetKeyDown(KeyCode.Space)) ChangerPause();
#endif
            Avancer(Time.deltaTime);
        }

        void AfficherCube(int index, Vector3 position, Vector3 taille)
        {
            cubes[index].ApplyTransformation(Mat3.Identity.ToMatrix4x4(position, taille));
        }

        void Afficher()
        {
            if (exempleRessorts)
            {
                for (int i = 0; i < 3; i++)
                    AfficherCube(i, ressorts.Particles[i].position, Vector3.one);
                for (int i = 0; i < 2; i++)
                {
                    Vector3 a = ressorts.Particles[i].position;
                    Vector3 b = ressorts.Particles[i + 1].position;
                    // Une barre fine représente chaque ressort vertical.
                    AfficherCube(i + 3, (a + b) * 0.5f, new Vector3(0.03f, Mathf.Abs(a.y - b.y), 0.03f));
                }
            }
            else
            {
                AfficherCube(0, euler.Particles[0].position, Vector3.one);
                AfficherCube(1, rk4.Particles[0].position, Vector3.one);
                float y = ExperiencesParticules.HauteurExacte(monde.Time, hauteur, masseDepart, frottementDepart);
                AfficherCube(2, new Vector3(3f, y, 0f), Vector3.one);
            }
        }

        void OnGUI()
        {
            GUI.Label(new Rect(10, 10, 800, 25), "1 : chute (Euler / RK4 / exacte) / 2 : ressorts / Espace : pause / R : reset");
        }

        void Nettoyer()
        {
            foreach (CustomCube cube in cubes)
            {
                if (cube == null) continue;
                cube.gameObject.SetActive(false);
                Destroy(cube.GetComponent<MeshFilter>().sharedMesh);
                Destroy(cube.GetComponent<MeshRenderer>().sharedMaterial);
                Destroy(cube.gameObject);
            }
            cubes.Clear();
        }

        void OnDestroy() { Nettoyer(); }
    }
}
