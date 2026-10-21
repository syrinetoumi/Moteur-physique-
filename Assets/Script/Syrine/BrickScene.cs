using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
using PhysicsLib;

/// <summary>
/// Scène "Dense stacking" : un mur de briques est suspendu en l'air. Dès que la souris BOUGE,
/// les briques sont lâchées et tombent (gravité du moteur). Aucun clic, aucun objet qui les touche.
/// A glisser sur un GameObject VIDE d'une scène vide. Aucun Rigidbody / Collider / Physics d'Unity.
///
/// Collisions : sol (point-plan) + brique contre brique (sommet dans boîte), résolues par impulsions
/// (ImpulseResolver) : les briques s'empilent sans s'interpénétrer.
/// </summary>
public class BrickScene : MonoBehaviour
{
    [Header("Mur de briques")]
    public int columns = 4;
    public int rows = 5;
    public Vector3 brickSize = new Vector3(2f, 1f, 1f);
    public float gap = 0.05f;                 // petit espace pour qu'aucune brique ne se chevauche au départ
    public float startHeight = 6f;            // hauteur de départ au-dessus du sol
    public float brickMass = 1f;

    [Header("Souris")]
    public float moveThreshold = 2f;          // déplacement minimal de la souris (en pixels) pour lâcher les briques

    [Header("Simulation")]
    public bool useRK4 = true;
    public float fixedDt = 0.005f;            // pas de temps du moteur (plus petit = plus précis mais plus lent)
    public int solverIterations = 10;         // itérations du résolveur de contacts
    public float friction = 0.5f;             // friction brique/brique et brique/sol
    public float restitution = 0f;            // rebond (0 = pas de rebond)

    PhysicsWorld world;
    readonly List<RigidBody> bricks = new List<RigidBody>();
    readonly List<GameObject> spawned = new List<GameObject>();
    bool released;                            // faux : briques suspendues ; vrai : la simulation tourne
    Vector3 lastMouse;

    static readonly Color[] palette =
    {
        new Color(0.95f, 0.45f, 0.1f), new Color(0.2f, 0.6f, 0.9f), new Color(0.1f, 0.6f, 0.45f),
        new Color(0.95f, 0.8f, 0.2f), new Color(0.9f, 0.5f, 0.7f), new Color(0.2f, 0.4f, 0.8f)
    };

    void Start() { Build(); }

    void Build()
    {
        foreach (GameObject go in spawned) Destroy(go);
        spawned.Clear();
        bricks.Clear();
        released = false;
        lastMouse = MousePosition();

        world = new PhysicsWorld();
        world.FixedDt = fixedDt;
        world.AddGlobalForce(new GravityForce());

        // détection : sol (plan y = 0) + brique contre brique ; résolution : impulsions + friction
        world.Detector = new CompositeDetector(
            new PlaneContactDetector(Plane3.FromPointNormal(Vector3.zero, Vector3.up)),
            new BoxBoxContactDetector());
        world.Resolver = new ImpulseResolver
        {
            Iterations = solverIterations,
            Friction = friction,
            Restitution = restitution
        };

        // le sol, dessiné avec notre CustomCube (simple affichage : le moteur n'en dépend pas)
        GameObject floor = new GameObject("Sol");
        CustomCube floorCube = floor.AddComponent<CustomCube>();
        floorCube.SetColor(new Color(0.55f, 0.55f, 0.55f));
        floorCube.ApplyTransformation(Mat3.Identity.ToMatrix4x4(new Vector3(0f, -0.1f, 0f), new Vector3(24f, 0.2f, 12f)));
        spawned.Add(floor);

        IntegrationMethod method = useRK4 ? IntegrationMethod.RK4 : IntegrationMethod.Euler;
        int n = 0;
        for (int r = 0; r < rows; r++)
        {
            float shift = (r % 2 == 1) ? brickSize.x * 0.5f : 0f;                   // briques décalées d'une rangée à l'autre
            for (int c = 0; c < columns; c++)
            {
                float x = (c - (columns - 1) * 0.5f) * (brickSize.x + gap) + shift;
                float y = startHeight + brickSize.y * 0.5f + r * (brickSize.y + gap);
                SpawnBrick(new Vector3(x, y, 0f), palette[n % palette.Length], method);
                n++;
            }
        }

        if (Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(0f, 6f, -18f);
            Camera.main.transform.rotation = Quaternion.Euler(10f, 0f, 0f);
        }
    }

    void SpawnBrick(Vector3 position, Color color, IntegrationMethod method)
    {
        // 1) le corps de la simulation (notre moteur)
        RigidBody body = new RigidBody(new BoxShape(brickSize), brickMass, position);
        body.Method = method;
        world.AddBody(body);
        bricks.Add(body);

        // 2) l'affichage : cube construit dans le code + lien avec le corps
        GameObject go = new GameObject("Brick");
        CustomCube cube = go.AddComponent<CustomCube>();
        cube.SetColor(color);
        BodyView view = go.AddComponent<BodyView>();
        view.Body = body;
        spawned.Add(go);
    }

    void Update()
    {
        if (KeyPressed()) { Build(); return; }

        if (!released)
        {
            // la souris a-t-elle bougé ? alors on lâche les briques
            Vector3 m = MousePosition();
            if ((m - lastMouse).sqrMagnitude > moveThreshold * moveThreshold) released = true;
            lastMouse = m;
            return;                            // tant qu'elles ne sont pas lâchées, le moteur ne tourne pas
        }

        world.Update(Time.deltaTime);
    }

    static bool KeyPressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.R);
#endif
    }

    static Vector3 MousePosition()
    {
#if ENABLE_INPUT_SYSTEM
        if (Mouse.current == null) return Vector3.zero;
        Vector2 p = Mouse.current.position.ReadValue();
        return new Vector3(p.x, p.y, 0f);
#else
        return Input.mousePosition;
#endif
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 700, 100),
            (released ? "Briques lâchées" : "Bouge la souris pour lâcher les briques") + " | R = recommencer\n" +
            "Briques : " + bricks.Count + " | t = " + world.Time.ToString("F2") + " s | " +
            "contacts avec le sol : " + world.Contacts.Count);
    }
}