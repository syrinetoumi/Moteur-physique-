using System.Collections.Generic;
using PhysicsLib;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Haithem.Chapitre2Partie2
{
    public class DemoCorpsRigides : MonoBehaviour
    {
        public bool utiliserRK4 = true;
        PhysicsWorld monde;
        readonly List<Transform> cubes = new List<Transform>();
        int exemple;
        bool pause;
        Camera camera;

        public PhysicsWorld Monde { get { return monde; } }
        public bool EnPause { get { return pause; } }

        void Start() { Reinitialiser(); }

        public void ChoisirExemple(int choix)
        {
            exemple = Mathf.Clamp(choix, 0, 2);
            Reinitialiser();
        }

        public void Reinitialiser()
        {
            Nettoyer();
            monde = ExperiencesRigides.Creer(exemple, utiliserRK4);
            camera = Camera.main;
            for (int i = 0; i < monde.Bodies.Count; i++)
            {
                GameObject objet = GameObject.CreatePrimitive(PrimitiveType.Cube);
                objet.name = i == 0 ? "Corps gauche" : "Corps droit";
                Collider collider = objet.GetComponent<Collider>();
                collider.enabled = false;
                Destroy(collider);
                cubes.Add(objet.transform);
            }
            Afficher();
        }

        public void ChangerPause() { pause = !pause; }

        public void Avancer(float dt)
        {
            if (pause || monde == null || dt <= 0f) return;
            monde.Update(dt);
            Afficher();
        }

        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard clavier = Keyboard.current;
            if (clavier != null)
            {
                if (clavier.digit1Key.wasPressedThisFrame) ChoisirExemple(0);
                if (clavier.digit2Key.wasPressedThisFrame) ChoisirExemple(1);
                if (clavier.digit3Key.wasPressedThisFrame) ChoisirExemple(2);
                if (clavier.rKey.wasPressedThisFrame) Reinitialiser();
                if (clavier.spaceKey.wasPressedThisFrame) ChangerPause();
            }
#else
            if (Input.GetKeyDown(KeyCode.Alpha1)) ChoisirExemple(0);
            if (Input.GetKeyDown(KeyCode.Alpha2)) ChoisirExemple(1);
            if (Input.GetKeyDown(KeyCode.Alpha3)) ChoisirExemple(2);
            if (Input.GetKeyDown(KeyCode.R)) Reinitialiser();
            if (Input.GetKeyDown(KeyCode.Space)) ChangerPause();
#endif
            Avancer(Time.deltaTime);
        }

        void Afficher()
        {
            for (int i = 0; i < cubes.Count; i++)
            {
                RigidBody corps = monde.Bodies[i];
                cubes[i].position = corps.Position;
                // La rotation est calculée par le moteur, puis affichée par Unity.
                cubes[i].rotation = Quaternion.LookRotation(corps.Orientation.c2, corps.Orientation.c1);
                cubes[i].localScale = corps.Shape.RenderScale;
            }
            if (camera != null)
            {
                // Garder les deux corps visibles pendant la translation.
                Vector3 centre = (monde.Bodies[0].Position + monde.Bodies[1].Position) * 0.5f;
                camera.transform.position = centre + new Vector3(0f, 4f, -12f);
                camera.transform.LookAt(centre);
            }
        }

        void OnGUI()
        {
            GUI.Label(new Rect(10, 10, 1000, 25), "1 : force et couple / 2 : inertie / 3 : rotation libre / Espace : pause-reprendre / R : reset" + (pause ? " [PAUSE]" : ""));
            string explication = "Même force pendant 0,5 s : translation à gauche, translation et rotation à droite. Sans gravité.";
            if (exemple == 1) explication = "Même masse et même force décalée : la forme change l'inertie et la rotation. Sans gravité.";
            if (exemple == 2) explication = "Rotation libre : aucun effort extérieur, le centre reste immobile. Sans gravité.";
            GUI.Label(new Rect(10, 35, 1100, 25), explication);
            if (camera == null) return;
            for (int i = 0; i < cubes.Count; i++)
            {
                string nom = exemple == 0 ? (i == 0 ? "Force au centre" : "Force décalée") : (i == 0 ? "Cube" : "Pavé");
                Vector3 point = camera.WorldToScreenPoint(cubes[i].position + Vector3.up);
                if (point.z > 0f) GUI.Label(new Rect(point.x - 45f, Screen.height - point.y - 20f, 130f, 25f), nom);
            }
        }

        void Nettoyer()
        {
            foreach (Transform cube in cubes)
            {
                if (cube == null) continue;
                cube.gameObject.SetActive(false);
                Destroy(cube.gameObject);
            }
            cubes.Clear();
        }

        void OnDestroy() { Nettoyer(); }
    }
}
