using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
using PhysicsLib;

/// <summary>
/// Scène "Dense stacking" : un mur de briques tient debout sur le sol. Quand tu BOUGES LA SOURIS, le sol vibre
/// (la vibration dépend de la vitesse de la souris) et les briques tombent les unes sur les autres.
/// A glisser sur un GameObject VIDE d'une scène vide. Aucun Rigidbody / Collider / Physics d'Unity.
///
/// Moteur : gravité + secousse (ShakeForce), collisions sol (point-plan) et brique/brique (axes séparateurs),
/// résolues par impulsions avec friction (ImpulseResolver), mise en sommeil des briques immobiles (BrickSleeper).
/// </summary>
public class BrickScene : MonoBehaviour
{
    [Header("Mur de briques")]
    public int columns = 4;
    public int rows = 5;
    public Vector3 brickSize = new Vector3(2f, 1f, 1f);
    public float gap = 0.01f;                 // petit espace entre briques
    public float density = 0.5f;              // masse = volume x densité (une brique 2x1x1 pèse 1)

    [Header("Souris -> secousse du sol")]
    public float speedForMaxShake = 1200f;    // vitesse de souris (pixels/s) qui donne la secousse maximale
    public float shakeDecay = 2.5f;           // la secousse retombe à cette vitesse quand la souris s'arrête
    public float maxShakeAcceleration = 35f;  // accélération maximale du sol (m/s²)
    public float shakeFrequency = 1f;         // fréquence de la vibration (Hz)

    [Header("Simulation")]
    public bool useRK4 = true;
    public float fixedDt = 0.005f;            // pas de temps du moteur
    public int solverIterations = 10;         // itérations du résolveur de contacts
    public float friction = 0.5f;             // friction brique/brique et brique/sol
    public float restitution = 0f;            // rebond (0 = pas de rebond)

    PhysicsWorld world;
    ShakeForce shake;
    BrickSleeper sleeper;
    readonly List<RigidBody> bricks = new List<RigidBody>();
    readonly List<GameObject> spawned = new List<GameObject>();
    Vector3 lastMouse;
    float intensity;                          // intensité de secousse actuelle (0..1)
    int colorIndex;

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
        intensity = 0f;
        colorIndex = 0;
        lastMouse = MousePosition();

        world = new PhysicsWorld();
        world.FixedDt = fixedDt;

        shake = new ShakeForce(world);
        shake.MaxAcceleration = maxShakeAcceleration;
        shake.Frequency = shakeFrequency;
        sleeper = new BrickSleeper();

        world.AddGlobalForce(new GravityForce());
        world.AddGlobalForce(shake);

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

        BuildWall();

        if (Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(0f, 4f, -16f);
            Camera.main.transform.rotation = Quaternion.Euler(8f, 0f, 0f);
        }
    }

    /// <summary>
    /// Mur en appareillage : rangées paires = 'columns' briques ; rangées impaires = 'columns - 1' briques décalées
    /// d'une demi-brique + une demi-brique à chaque bout (sinon les briques du bout seraient en équilibre sur le bord).
    /// </summary>
    void BuildWall()
    {
        float pitch = brickSize.x + gap;
        float wallHalf = columns * pitch * 0.5f - gap * 0.5f;
        Vector3 halfBrick = new Vector3(brickSize.x * 0.5f, brickSize.y, brickSize.z);

        for (int r = 0; r < rows; r++)
        {
            float y = brickSize.y * 0.5f + r * (brickSize.y + gap);
            if (r % 2 == 0)
            {
                for (int c = 0; c < columns; c++)
                    SpawnBrick(brickSize, new Vector3((c - (columns - 1) * 0.5f) * pitch, y, 0f));
            }
            else
            {
                for (int c = 0; c < columns - 1; c++)
                    SpawnBrick(brickSize, new Vector3((c - (columns - 2) * 0.5f) * pitch, y, 0f));
                float hx = wallHalf - halfBrick.x * 0.5f;
                SpawnBrick(halfBrick, new Vector3(-hx, y, 0f));
                SpawnBrick(halfBrick, new Vector3(hx, y, 0f));
            }
        }
    }

    void SpawnBrick(Vector3 size, Vector3 position)
    {
        // 1) le corps de la simulation (notre moteur)
        float mass = size.x * size.y * size.z * density;
        RigidBody body = new RigidBody(new BoxShape(size), mass, position);
        body.Method = useRK4 ? IntegrationMethod.RK4 : IntegrationMethod.Euler;
        world.AddBody(body);
        bricks.Add(body);

        // 2) l'affichage : cube construit dans le code + lien avec le corps
        GameObject go = new GameObject("Brick");
        CustomCube cube = go.AddComponent<CustomCube>();
        cube.SetColor(palette[colorIndex % palette.Length]);
        colorIndex++;
        BodyView view = go.AddComponent<BodyView>();
        view.Body = body;
        spawned.Add(go);
    }

    void Update()
    {
        if (KeyPressed()) { Build(); return; }

        // vitesse de la souris -> intensité de la secousse (monte tout de suite, retombe doucement)
        float dt = Mathf.Max(Time.deltaTime, 0.0001f);
        Vector3 m = MousePosition();
        float speed = (m - lastMouse).magnitude / dt;
        lastMouse = m;
        float target = Mathf.Min(speed / speedForMaxShake, 1f);
        intensity = Mathf.Max(target, intensity - shakeDecay * dt);
        shake.Intensity = intensity;

        world.Update(Time.deltaTime);
        sleeper.Update(bricks, Time.deltaTime, intensity);
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
        // pénétration maximale entre deux briques (0 = aucune brique n'entre dans une autre)
        float maxPen = 0f;
        for (int i = 0; i < bricks.Count; i++)
            for (int j = i + 1; j < bricks.Count; j++)
                maxPen = Mathf.Max(maxPen, BoxBoxContactDetector.PenetrationDepth(bricks[i], bricks[j]));

        GUI.Box(new Rect(10, 10, 600, 80),
            "Bouge la souris pour secouer le sol | R = recommencer\n" +
            "secousse : " + intensity.ToString("F2") + " | briques : " + bricks.Count +
            " (endormies : " + sleeper.SleepingCount + ") | contacts : " + world.Contacts.Count + "\n" +
            "pénétration max entre briques : " + maxPen.ToString("F3") + " m");
    }
}