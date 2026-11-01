using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
using PhysicsLib;

public enum BroadPhaseMode { BruteForce, SweepAndPrune, Grid }

/// <summary>
/// Simulation "Dense stacking" : un grand mur de briques NON collées entre elles, un peu désordonné.
/// Le mur reste debout ; c'est le CLIC GAUCHE de la souris qui applique la force qui le fait tomber.
/// A glisser sur un GameObject VIDE d'une scène vide.  Aucun Rigidbody / Collider / Physics d'Unity.
///
/// Boucle du moteur (Chapitre 4) :
///   intégration (RK4) -> mise à jour des volumes (AABB) -> phase large (force brute / SAP / grille)
///   -> phase étroite (SAT à 15 axes) -> contacts (p, n, δ) -> réponse (séparation + impulsion avec friction).
///
/// Clic souris : rayon de la caméra, test rayon-boîte par la méthode des slabs (Chapitre 4, slide 64) dans le repère
/// local de chaque brique, puis force appliquée au point touché (moment tau = (r - x) ^ F, Chapitre 2).
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

    [Header("Clic souris (force qui fait tomber le mur)")]
    public float clickForce = 150f;           // intensité de la force (N)
    public float clickDuration = 0.1f;        // durée d'application de la force (s)
    public float pushRadius = 3f;             // rayon d'action autour du point cliqué (m) ; 0 = seulement la brique touchée
    public bool horizontalPush = true;        // pousse horizontalement (sans la composante verticale du rayon)

    [Header("Collisions")]
    public BroadPhaseMode broadPhase = BroadPhaseMode.SweepAndPrune;
    public int solverIterations = 10;         // itérations de la réponse aux contacts
    public float friction = 0.9f;             // friction brique/brique et brique/sol
    public float restitution = 0f;            // rebond (0 = pas de rebond)
    public float positionCorrection = 0.1f;   // part de la pénétration corrigée à chaque pas
    public float airDrag = 0.5f;              // frottement de l'air  F = -k.v

    [Header("Simulation")]
    public bool useRK4 = true;
    public float fixedDt = 0.002f;            // pas de temps du moteur
    public float timeScale = 1f;              // ACCÉLÉRATEUR : 1 = temps réel, 2 = deux fois plus vite, 0.5 = ralenti
    public float maxFrameTime = 0.1f;         // temps simulé maximal par frame (s)

    PhysicsWorld world;
    BoxBoxContactDetector boxDetector;
    BrickSleeper sleeper;
    readonly List<RigidBody> bricks = new List<RigidBody>();
    readonly List<GameObject> spawned = new List<GameObject>();
    int colorIndex;
    double lastMs;
    System.Random rnd;
    float smoothFrame = 0.016f;
    float speedRatio = 1f;
    string lastClick = "";

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
        lastClick = "";
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

        if (Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(0f, 6f, -30f);
            Camera.main.transform.LookAt(new Vector3(0f, 2.5f, -3f));
        }
    }

    /// <summary>
    /// Mur en appareillage : rangées paires = 'columns' briques ; rangées impaires = 'columns - 1' briques décalées
    /// d'une demi-brique + une demi-brique à chaque bout. Chaque rangée est décalée au hasard (rowOffset) et chaque
    /// brique est légèrement tournée / inclinée : le mur n'est pas parfait.
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

    // ------------------------------------------------------------------ clic souris -> force

    static float Comp(Vector3 v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

    /// <summary>
    /// Rayon - boîte orientée par la méthode des slabs (slide 64), dans le repère LOCAL de la brique
    /// où la boîte devient une AABB centrée à l'origine :  t1 = (min - o)/d,  t2 = (max - o)/d,
    /// t_entrée = max des t1,  t_sortie = min des t2,  touché si t_entrée <= t_sortie.
    /// Renvoie aussi le point touché en repère de l'objet (relatif au centre de masse).
    /// </summary>
    static bool RayBox(Vector3 o, Vector3 d, RigidBody b, out float tEnter, out Vector3 localHit)
    {
        tEnter = 0f;
        localHit = Vector3.zero;
        BoxShape box = b.Shape as BoxShape;
        if (box == null) return false;

        Mat3 Rt = b.State.R.Transpose();
        Vector3 oL = Rt * (o - b.Position);          // origine du rayon dans le repère de la brique
        Vector3 dL = Rt * d;                         // direction du rayon dans le repère de la brique
        Vector3 h = box.Size * 0.5f;
        float tExit = float.MaxValue;

        for (int i = 0; i < 3; i++)
        {
            float oi = Comp(oL, i), di = Comp(dL, i), hi = Comp(h, i);
            if (Mathf.Abs(di) < 1e-8f)
            {
                if (oi < -hi || oi > hi) return false;       // rayon parallèle à la dalle et en dehors
                continue;
            }
            float t1 = (-hi - oi) / di;
            float t2 = (hi - oi) / di;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            tEnter = Mathf.Max(tEnter, t1);
            tExit = Mathf.Min(tExit, t2);
            if (tEnter > tExit) return false;
        }
        localHit = oL + dL * tEnter;
        return true;
    }

    /// <summary>La brique la plus proche touchée par le rayon (plus petit t), ou null.</summary>
    RigidBody PickBrick(Vector3 o, Vector3 d, out float tHit, out Vector3 localHit)
    {
        RigidBody best = null;
        tHit = float.MaxValue;
        localHit = Vector3.zero;
        foreach (RigidBody b in bricks)
        {
            float t; Vector3 lp;
            if (RayBox(o, d, b, out t, out lp) && t < tHit) { tHit = t; localHit = lp; best = b; }
        }
        return best;
    }

    /// <summary>Clic : la brique touchée reçoit F au point cliqué ; les voisines (rayon pushRadius) une force décroissante.</summary>
    void ClickPush(Vector2 screenPos)
    {
        if (Camera.main == null) return;
        Ray ray = Camera.main.ScreenPointToRay(new Vector3(screenPos.x, screenPos.y, 0f));

        float t; Vector3 localHit;
        RigidBody hit = PickBrick(ray.origin, ray.direction, out t, out localHit);
        if (hit == null) { lastClick = "clic dans le vide"; return; }

        Vector3 dir = ray.direction;
        if (horizontalPush) dir.y = 0f;
        if (dir.sqrMagnitude < 1e-6f) return;
        dir = dir.normalized;

        Vector3 hitPoint = ray.origin + ray.direction * t;
        int pushed = 0;
        foreach (RigidBody b in bricks)
        {
            float k;
            Vector3 point;
            if (b == hit) { k = 1f; point = localHit; }                       // au point touché : crée aussi un moment
            else
            {
                if (pushRadius <= 0f) continue;
                float dist = (b.Position - hitPoint).magnitude;
                if (dist > pushRadius) continue;
                k = 1f - dist / pushRadius;                                    // décroît avec la distance
                point = Vector3.zero;                                          // au centre de masse
            }
            sleeper.Wake(bricks, b);
            b.ApplyForceAtPoint(dir * (clickForce * k), point, clickDuration);
            pushed++;
        }
        lastClick = "force sur " + pushed + " brique(s), F = " + clickForce.ToString("F0") + " N pendant " + clickDuration.ToString("F2") + " s";
    }

    static bool ClickPressed(out Vector2 pos)
    {
#if ENABLE_INPUT_SYSTEM
        Mouse m = Mouse.current;
        if (m != null && m.leftButton.wasPressedThisFrame) { pos = m.position.ReadValue(); return true; }
        pos = Vector2.zero;
        return false;
#else
        pos = Input.mousePosition;
        return Input.GetMouseButtonDown(0);
#endif
    }

    static bool KeyPressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.R);
#endif
    }

    // ------------------------------------------------------------------ boucle

    void Update()
    {
        if (KeyPressed()) { Build(); return; }

        Vector2 mouse;
        if (ClickPressed(out mouse)) ClickPush(mouse);

        float real = Mathf.Max(Time.deltaTime, 0.0001f);
        float dt = Mathf.Min(real, maxFrameTime) * timeScale;     // temps à simuler pendant cette frame
        float simBefore = world.Time;

        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
        world.Update(dt);
        lastMs = sw.Elapsed.TotalMilliseconds;
        sleeper.Update(bricks, dt, 0f);

        smoothFrame = smoothFrame * 0.95f + real * 0.05f;
        speedRatio = speedRatio * 0.95f + ((world.Time - simBefore) / real) * 0.05f;
    }

    void OnGUI()
    {
        GUI.Box(new Rect(10, 10, 760, 120),
            "CLIC GAUCHE = pousser le mur | R = recommencer | phase large : " + broadPhase + "\n" +
            "FPS : " + (1f / smoothFrame).ToString("F0") + " | vitesse de la simulation : x" + speedRatio.ToString("F2") +
            " (1.00 = temps réel) | accélérateur : x" + timeScale.ToString("F1") + "\n" +
            "t = " + world.Time.ToString("F2") + " s | briques : " + bricks.Count + " (endormies : " + sleeper.SleepingCount + ")\n" +
            "tests phase large : " + boxDetector.BroadPhase.TestCount + " | paires candidates : " + boxDetector.PairCount +
            " | contacts : " + world.Contacts.Count + " | calcul : " + lastMs.ToString("F1") + " ms\n" +
            "dernier clic : " + lastClick);
    }
}