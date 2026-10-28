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
        readonly List<CustomCube> cubes = new List<CustomCube>();
        int exemple;
        bool pause = true;

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
            pause = true;
            for (int i = 0; i < monde.Bodies.Count; i++)
            {
                GameObject objet = new GameObject(i == 0 ? "Corps gauche" : "Corps droit");
                CustomCube cube = objet.AddComponent<CustomCube>();
                cubes.Add(cube);
            }
            Afficher();
        }

        public void ChangerPause() { pause = !pause; }

        public void Avancer(float dt)
        {
            if (pause || monde == null || dt <= 0f) return;
            monde.Update(Mathf.Min(dt, Mathf.Max(0f, 3f - monde.Time)));
            if (monde.Time >= 3f - monde.FixedDt * 0.5f) pause = true;
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
                cubes[i].ApplyTransformation(monde.Bodies[i].ModelMatrix());
        }

        void OnGUI()
        {
            GUI.Label(new Rect(10, 10, 850, 25), "1 : force et couple / 2 : inertie / 3 : rotation libre / Espace : pause / R : reset");
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
