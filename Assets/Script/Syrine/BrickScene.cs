using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
using PhysicsLib;

public enum BroadPhaseMode { BruteForce, SweepAndPrune, Grid }

/// <summary>
/// Simulation "Dense stacking" AUTONOME (comme la vidéo) : un grand mur de briques NON collées entre elles,
/// un peu désordonné, reçoit une légère poussée au lancement ; il s'effondre en tas et les briques se heurtent
/// pendant la chute. Aucune interaction : ni souris, ni clic.
/// A glisser sur un GameObject VIDE d'une scène vide.  Aucun Rigidbody / Collider / Physics d'Unity.
///
/// Boucle du moteur (Chapitre 4) :
///   intégration (RK4) -> mise à jour des volumes (AABB) -> phase large (force brute / SAP / grille)
///   -> phase étroite (SAT à 15 axes) -> contacts (p, n, δ) -> réponse (séparation + impulsion avec friction).
/// </summary>
public class BrickScene : MonoBehaviour
{
    [Header("Mur de briques (non collées)")]
    public int columns = 10;
    public int rows = 10;
    public Vector3 brickSize = new Vector3(2f, 1f, 1f);
    public float gap = 0.01f;                 // petit espace entre briques
    public float density = 0.5f;              // masse = volume x densité (une brique 2x1x1 pèse 1)

    [Header("Désordre initial du mur")]
    public float rowOffset = 0.3f;            // décalage aléatoire de chaque rangée (m)
    public float brickYaw = 3f;               // rotation aléatoire de chaque brique autour de la verticale (degrés)
    public float brickTilt = 2f;              // petite inclinaison aléatoire de chaque brique (degrés)
    public int seed = 7;                      // graine : même graine = même mur

    [Header("Poussée initiale (au lancement)")]
    public float pushAngularSpeed = 0.15f;    // le mur penche vers la caméra à cette vitesse (rad/s)

    [Header("Collisions")]
    public BroadPhaseMode broadPhase = BroadPhaseMode.SweepAndPrune;
    public int solverIterations = 10;         // itérations de la réponse aux contacts
    public float friction = 0.9f;             // friction brique/brique et brique/sol
    public float restitution = 0f;            // rebond (0 = pas de rebond)
    public float positionCorrection = 0.1f;   // part de la pénétration corrigée à chaque pas
    public float airDrag = 0.5f;              // frottement de l'air  F = -k.v  (évite que les briques soient projetées trop loin)

    [Header("Simulation")]
    public bool useRK4 = true;
    public float fixedDt = 0.005f;            // pas de temps du moteur

    PhysicsWorld world;
    BoxBoxContactDetector boxDetector;
    BrickSleeper sleeper;
    readonly List<RigidBody> bricks = new List<RigidBody>();
    readonly List<GameObject> spawned = new List<GameObject>();
    int colorIndex;
    double lastMs;
    System.Random rnd;

    static readonly Color[] palette =
    {
        new Color(0.95f, 0.45f, 0.1f), new Color(0.2f, 0.6f, 0.9f), new Color(0.1f, 0.6f, 0.45f),
        new Color(0.95f, 0.8f, 0.2f), new Color(0.9f, 0.5f, 0.7f), new Color(0.2f, 0.4f, 0.8f)
    };

    void Start() { Build(); }

    IBroadPhase CreateBroadPhase()
    {
        switch (broadPhase)
        {
            case BroadPhaseMode.BruteForce: return new BruteForceBroadPhase();
            case BroadPhaseMode.Grid: return new GridBroadPhase();
            default: return new SweepAndPruneBroadPhase();
        }
    }

    float Rand() { return (float)(rnd.NextDouble() * 2.0 - 1.0); }          // nombre aléatoire dans [-1, 1]

    void Build()
    {
        foreach (GameObject go in spawned) Destroy(go);
        spawned.Clear();
        bricks.Clear();
        colorIndex = 0;
        rnd = new System.Random(seed);

        world = new PhysicsWorld();
        world.FixedDt = fixedDt;
        world.AddGlobalForce(new GravityForce());
        if (airDrag > 0f) world.AddGlobalForce(new ViscousDragForce(airDrag));

        // détection : sol (sommets sous le plan y = 0) + brique contre brique (phase large + SAT) ; réponse : impulsions
        boxDetector = new BoxBoxContactDetector();
        boxDetector.BroadPhase = CreateBroadPhase();
        world.Detector = new CompositeDetector(
            new PlaneContactDetector(Plane3.FromPointNormal(Vector3.zero, Vector3.up)),
            boxDetector);
        world.Resolver = new ImpulseResolver
        {
            Iterations = solverIterations,
            Friction = friction,
            Restitution = restitution,
            PositionCorrection = positionCorrection
        };
        sleeper = new BrickSleeper();

        // le sol, dessiné avec notre CustomCube (simple affichage : le moteur n'en dépend pas)
        GameObject floor = new GameObject("Sol");
        CustomCube floorCube = floor.AddComponent<CustomCube>();
        floorCube.SetColor(new Color(0.55f, 0.55f, 0.55f));
        floorCube.ApplyTransformation(Mat3.Identity.ToMatrix4x4(new Vector3(0f, -0.1f, -10f), new Vector3(70f, 0.2f, 50f)));
        spawned.Add(floor);

        BuildWall();
        PushWall();

        if (Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(0f, 6f, -30f);
            Camera.main.transform.LookAt(new Vector3(0f, 2.5f, -3f));
        }
    }

    /// <summary>
    /// Mur en appareillage : rangées paires = 'columns' briques ; rangées impaires = 'columns - 1' briques décalées
    /// d'une demi-brique + une demi-brique à chaque bout. Chaque rangée est décalée au hasard (rowOffset) et chaque
    /// brique est légèrement tournée / inclinée : le mur n'est pas parfait, comme dans la vidéo.
    /// </summary>
    void BuildWall()
    {
        float pitch = brickSize.x + gap;
        float wallHalf = columns * pitch * 0.5f - gap * 0.5f;
        Vector3 halfBrick = new Vector3(brickSize.x * 0.5f, brickSize.y, brickSize.z);

        for (int r = 0; r < rows; r++)
        {
            float y = brickSize.y * 0.5f + r * (brickSize.y + gap);
            float jit = Rand() * rowOffset;
            if (r % 2 == 0)
            {
                for (int c = 0; c < columns; c++)
                    SpawnBrick(brickSize, new Vector3((c - (columns - 1) * 0.5f) * pitch + jit, y, 0f));
            }
            else
            {
                for (int c = 0; c < columns - 1; c++)
                    SpawnBrick(brickSize, new Vector3((c - (columns - 2) * 0.5f) * pitch + jit, y, 0f));
                float hx = wallHalf - halfBrick.x * 0.5f;
                SpawnBrick(halfBrick, new Vector3(-hx + jit, y, 0f));
                SpawnBrick(halfBrick, new Vector3(hx + jit, y, 0f));
            }
        }
    }

    /// <summary>
    /// Poussée : le mur reçoit une rotation initiale autour de son arête avant (axe x, z = -épaisseur/2, y = 0),
    /// vers la caméra. Chaque brique reçoit la vitesse d'un solide en rotation  v = ω ^ (x - pivot)  et la vitesse
    /// angulaire ω. Les briques n'étant pas collées, elles se séparent ensuite et chutent selon la gravité et les collisions.
    /// </summary>
    void PushWall()
    {
        Vector3 pivot = new Vector3(0f, 0f, -brickSize.z * 0.5f);
        Vector3 omega = new Vector3(-pushAngularSpeed, 0f, 0f);
        foreach (RigidBody b in bricks)
        {
            b.SetVelocity(Vector3.Cross(omega, b.Position - pivot));
            b.SetAngularVelocity(omega);
        }
    }

    void SpawnBrick(Vector3 size, Vector3 position)
    {
        // 1) le corps de la simulation (notre moteur), légèrement tourné / incliné au hasard
        float mass = size.x * size.y * size.z * density;
        RigidBody body = new RigidBody(new BoxShape(size), mass, position);
        body.Method = useRK4 ? IntegrationMethod.RK4 : IntegrationMethod.Euler;
        Quat yaw = Quat.FromAxisAngle(new Vector3(0f, 1f, 0f), Rand() * brickYaw * Mathf.PI / 180f);
        Quat tilt = Quat.FromAxisAngle(new Vector3(Rand(), 0f, Rand()), Mathf.Abs(Rand()) * brickTilt * Mathf.PI / 180f);
        body.State.R = (yaw * tilt).Normalized().ToMat3();
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

        float dt = Mathf.Min(Time.deltaTime, 0.033f);        // évite un énorme pas si une frame est très longue
        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
        world.Update(dt);
        lastMs = sw.Elapsed.TotalMilliseconds;
        sleeper.Update(bricks, dt, 0f);
    }

    static bool KeyPressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.R);
#endif
    }

    void OnGUI()
    {
        GUI.Box(new Rect(10, 10, 700, 80),
            "Simulation autonome | R = recommencer | phase large : " + broadPhase + "\n" +
            "t = " + world.Time.ToString("F2") + " s | briques : " + bricks.Count + " (endormies : " + sleeper.SleepingCount + ")\n" +
            "tests phase large : " + boxDetector.BroadPhase.TestCount + " | paires candidates : " + boxDetector.PairCount +
            " | contacts : " + world.Contacts.Count + " | calcul : " + lastMs.ToString("F1") + " ms");
    }
}