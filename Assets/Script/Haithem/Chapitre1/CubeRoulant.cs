using PhysicsLib;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Haithem.Chapitre1
{
    // Cube qui roule avec les quaternions de PhysicsLib.
    [RequireComponent(typeof(CustomCube))]
    public class CubeRoulant : MonoBehaviour
    {
        [Min(0.01f)] public float taille = 1f;
        [Min(0.01f)] public float dureeTour = 0.3f;

        public Vector3 Position { get; private set; }
        public Quat Orientation { get; private set; } = Quat.Identity;
        public bool EnMouvement { get; private set; }

        Vector3 depart, pivot, axe, destination;
        Quat orientationDepart;
        float progression;
        CustomCube cube;
        Mesh mesh;
        Material original;

        void Start()
        {
            cube = GetComponent<CustomCube>();
            mesh = GetComponent<MeshFilter>().sharedMesh;
            original = GetComponent<MeshRenderer>().sharedMaterial;
            Reinitialiser();
        }

        // Remet le cube au départ.
        public void Reinitialiser()
        {
            taille = Mathf.Max(0.01f, taille);
            dureeTour = Mathf.Max(0.01f, dureeTour);
            Position = new Vector3(0f, taille * 0.5f, 0f);
            Orientation = Quat.Identity;
            EnMouvement = false;
            progression = 0f;
            Afficher();
        }

        public bool Rouler(Vector3 direction)
        {
            if (EnMouvement) return false;
            if (direction != Vector3.right && direction != Vector3.left &&
                direction != Vector3.forward && direction != Vector3.back) return false;

            Commencer();
            // Le pivot est sur l'arête qui touche le sol.
            pivot = depart + (direction - Vector3.up) * (taille * 0.5f);
            axe = Quat.Cross(Vector3.up, direction);
            destination = depart + direction * taille;
            return true;
        }

        // Même rotation, mais autour du centre du cube.
        public bool TournerSurPlace()
        {
            if (EnMouvement) return false;
            Commencer();
            pivot = depart;
            axe = Vector3.up;
            destination = depart;
            return true;
        }

        void Commencer()
        {
            depart = Position;
            orientationDepart = Orientation;
            progression = 0f;
            EnMouvement = true;
        }

        public void Avancer(float dt)
        {
            if (!EnMouvement || dt <= 0f) return;

            progression = Mathf.Min(1f, progression + dt / Mathf.Max(0.01f, dureeTour));
            float angle = progression * Mathf.PI * 0.5f;
            Quat delta = Quat.FromAxisAngle(axe, angle);

            // On tourne la position autour du pivot.
            Position = pivot + delta.Rotate(depart - pivot);
            Orientation = (delta * orientationDepart).Normalized();

            if (progression >= 1f)
            {
                Position = destination;
                EnMouvement = false;
            }
            Afficher();
        }

        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard clavier = Keyboard.current;
            if (clavier != null)
            {
                if (clavier.rKey.wasPressedThisFrame)
                {
                    Reinitialiser();
                    return;
                }
                if (!EnMouvement)
                {
                    if (clavier.spaceKey.wasPressedThisFrame) TournerSurPlace();
                    else if (clavier.upArrowKey.isPressed) Rouler(Vector3.forward);
                    else if (clavier.downArrowKey.isPressed) Rouler(Vector3.back);
                    else if (clavier.leftArrowKey.isPressed) Rouler(Vector3.left);
                    else if (clavier.rightArrowKey.isPressed) Rouler(Vector3.right);
                }
            }
#else
            if (Input.GetKeyDown(KeyCode.R))
            {
                Reinitialiser();
                return;
            }
            if (!EnMouvement)
            {
                if (Input.GetKeyDown(KeyCode.Space)) TournerSurPlace();
                else if (Input.GetKey(KeyCode.UpArrow)) Rouler(Vector3.forward);
                else if (Input.GetKey(KeyCode.DownArrow)) Rouler(Vector3.back);
                else if (Input.GetKey(KeyCode.LeftArrow)) Rouler(Vector3.left);
                else if (Input.GetKey(KeyCode.RightArrow)) Rouler(Vector3.right);
            }
#endif
            Avancer(Time.deltaTime);
        }

        void Afficher()
        {
            if (cube != null)
                cube.ApplyTransformation(Orientation.ToMat3().ToMatrix4x4(Position, Vector3.one * taille));
        }

        void OnGUI()
        {
            GUI.Label(new Rect(10, 10, 500, 25), "Flèches : déplacer le cube / Espace : tourner / R : reset");
        }

        void OnDestroy()
        {
            if (original != null) Destroy(original);
            if (mesh != null) Destroy(mesh);
        }
    }
}
