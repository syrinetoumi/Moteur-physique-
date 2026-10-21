// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Scene de la video (article NVIDIA, Fig. 16) : un tissu vert, une tour de 4 cubes, de la fumee a gauche.
/// Chronologie : le tissu est lache et frappe le cube du haut -> la tour s'ecroule -> la fumee demarre
/// a gauche du tissu -> le tissu reste en l'air et brasse les particules de fumee.
/// Tout est fait de particules (spheres). Aucun Rigidbody, Collider ni Transform pour le mouvement.
/// Reglage conseille : Edit > Project Settings > Time > Fixed Timestep = 0.0166667.
/// Sol : plan invisible y = 0 (pas de mur, pas de sol affiche).
/// </summary>
public class SceneGenerator : MonoBehaviour
{
    [Header("Performance")]
    [Range(1, 4)]
    public int substeps = 2;

    [Range(1, 8)]
    public int solverIterations = 4;

    [Range(3, 6)]
    public int cubeResolution = 4;

    [Tooltip("Iterations de la contrainte de densite de la fumee par sous-pas (le plus gros cout apres les contacts).")]
    [Range(1, 8)]
    public int densityIterations = 2;

    [Header("Animation Timing")]
    public float clothReleaseTime = 0.25f;

    public float smokeStartTime = 1.4f;

    [Header("Cloth Impact")]
    public float clothSwingSpeed = 6f;

    [Range(5, 15)]
    public int clothColumns = 11;

    [Range(8, 30)]
    public int clothRows = 19;

    [Tooltip("Amortissement horizontal des particules au contact du sol (par sous-pas). Remplace la friction de sol trop faible.")]
    [Range(0f, 0.5f)]
    public float groundFriction = 0.10f;

    [Tooltip("Masse d'une particule de tissu (kg). Un tissu leger frappe moins fort qu'un tissu lourd.")]
    public float clothParticleMass = 0.05f;

    [Header("Cubes")]
    [Tooltip("Masse totale d'un cube (kg).")]
    public float cubeMass = 3f;

    [Header("Stabilite")]
    [Tooltip("Vitesse maximale d'une particule (m/s). Evite les cubes projetes a l'autre bout de la scene.")]
    public float maxSpeed = 7f;

    [Tooltip("Amortissement de l'air (1/s).")]
    public float airDamping = 0.8f;

    [Header("Smoke")]
    public int gasParticleCount = 450;

    public int smokeMarkerCount = 150;

    [Tooltip("Masse d'une particule de fumee (kg). Plus elle est grande, plus la fumee pousse le tissu.")]
    public float gasParticleMass = 0.1f;

    [Tooltip("Dessine aussi les marqueurs de fumee (eq. 28). Absents de la video.")]
    public bool showSmokeMarkers = false;

    // ============================================
    // OBJETS DE LA SIMULATION
    // ============================================

    ClothSimulation cloth;
    readonly List<RigidCubeSimulation> cubes = new List<RigidCubeSimulation>();
    GasSimulation gas;
    ParticleCollisionSolver contacts;

    readonly List<ParticleSceneRenderer> renderers = new List<ParticleSceneRenderer>();

    readonly List<Particle> allParticles = new List<Particle>();
    Vector3[] previousPositions;
    float[] radii;

    float simulationTime;
    bool clothReleased;

    const float GroundY = 0f;
    const int ClothPhase = 1;
    const int FirstCubePhase = 100;

    void Start()
    {
        GenerateScene();
    }

    void GenerateScene()
    {
        CreateCubes();
        CreateCloth();
        CreateSmoke();
        CollectParticles();
    }

    // ============================================
    // TOUR DE 4 CUBES (du bas vers le haut : orange, bleu, vert, jaune)
    // ============================================

    void CreateCubes()
    {
        float cubeSize = 1.2f;
        int resolution = Mathf.Max(3, cubeResolution);

        // Spheres qui se touchent : rayon = demi-espacement du reseau.
        float particleRadius = (cubeSize / (resolution - 1)) * 0.5f;

        // Les cubes s'empilent couche contre couche (reseaux alignes) : la tour tient debout.
        float cubeSpacing = cubeSize + particleRadius * 2f + 0.005f;
        float firstCenterY = cubeSize * 0.5f + particleRadius;

        Color[] colors =
        {
            new Color(1.00f, 0.55f, 0.10f),   // orange (bas)
            new Color(0.28f, 0.64f, 1.00f),   // bleu
            new Color(0.08f, 0.75f, 0.63f),   // vert
            new Color(0.90f, 0.78f, 0.12f)    // jaune (haut)
        };

        for (int i = 0; i < 4; i++)
        {
            Vector3 center = new Vector3(0f, firstCenterY + i * cubeSpacing, 0f);
            Vector3 size = new Vector3(cubeSize, cubeSize, cubeSize);

            RigidCubeSimulation cube = new RigidCubeSimulation(center, size, cubeMass, resolution, FirstCubePhase + i);
            cubes.Add(cube);

            renderers.Add(new ParticleSceneRenderer("Rigid Cube " + i, cube.Particles, cube.Radius, colors[i]));
        }
    }

    // ============================================
    // TISSU : accroche en haut a droite de la tour, il pend vers le bas
    // ============================================

    void CreateCloth()
    {
        cloth = new ClothSimulation(clothColumns, clothRows, 0.20f, new Vector3(-1.05f, 6.65f, 0f), clothParticleMass);

        renderers.Add(new ParticleSceneRenderer("Green Cloth", cloth.Particles, 0.065f, new Color(0.12f, 0.70f, 0.48f)));
    }

    // ============================================
    // FUMEE : a gauche, particules fluides cyan
    // ============================================

    void CreateSmoke()
    {
        gas = new GasSimulation(new Vector3(-2.3f, 0.20f, 0f), 0.30f, 3.0f,
                                Mathf.Max(1, gasParticleCount), Mathf.Max(1, smokeMarkerCount), gasParticleMass);

        renderers.Add(new ParticleSceneRenderer("Cyan Gas Particles", gas.GasParticles, gas.GasRadius, new Color(0.08f, 0.85f, 1.00f)));

        if (showSmokeMarkers)
            renderers.Add(new ParticleSceneRenderer("Smoke Markers", gas.SmokeMarkers, 0.05f, new Color(0.02f, 0.45f, 1.00f)));
    }

    void CollectParticles()
    {
        List<float> r = new List<float>();

        allParticles.AddRange(cloth.Particles);
        for (int i = 0; i < cloth.Particles.Count; i++) r.Add(cloth.Radius);

        for (int c = 0; c < cubes.Count; c++)
        {
            allParticles.AddRange(cubes[c].Particles);
            for (int i = 0; i < cubes[c].Particles.Count; i++) r.Add(cubes[c].Radius);
        }

        allParticles.AddRange(gas.GasParticles);
        for (int i = 0; i < gas.GasParticles.Count; i++) r.Add(gas.GasRadius);

        radii = r.ToArray();
        previousPositions = new Vector3[allParticles.Count];
        for (int i = 0; i < allParticles.Count; i++) previousPositions[i] = allParticles[i].position;

        contacts = new ParticleCollisionSolver(allParticles, radii, previousPositions);
        contacts.GroundY = GroundY;
    }

    // ============================================
    // LACHER DU TISSU
    // ============================================

    void ReleaseCloth()
    {
        if (clothReleased) return;

        // Le cote droit de la ligne du haut (pres de la tour) est libere, le cote gauche reste accroche :
        // le tissu reste en l'air apres le choc, comme dans la video.
        cloth.ReleaseTopRow(cloth.Width / 2, cloth.Width - 1);

        // Elan vers la tour : on change des vitesses, jamais un Transform.
        for (int i = 0; i < cloth.Particles.Count; i++)
        {
            Particle p = cloth.Particles[i];
            if (p.IsFixed) continue;
            p.velocity += new Vector3(clothSwingSpeed, 0f, 0f);
        }

        clothReleased = true;
    }

    // ============================================
    // BOUCLE PHYSIQUE (Algorithm 1 de l'article)
    // ============================================

    void FixedUpdate()
    {
        if (cloth == null) return;

        int steps = Mathf.Max(1, substeps);
        float dt = Time.fixedDeltaTime / steps;

        for (int sub = 0; sub < steps; sub++)
        {
            simulationTime += dt;

            if (!clothReleased && simulationTime >= clothReleaseTime) ReleaseCloth();
            if (!gas.IsActive && simulationTime >= smokeStartTime) gas.Activate();

            gas.UpdateEmission(dt);

            Predict(dt);

            contacts.BuildPairs();
            gas.BuildNeighbors();

            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                cloth.SolveConstraints();
                for (int i = 0; i < cubes.Count; i++) cubes[i].SolveShapeMatching(1f);
                if (iteration < densityIterations) gas.SolveDensity();
                contacts.SolvePairs(true);
                contacts.SolveGround(true);
            }

            UpdateVelocities(dt);
            contacts.ApplyGroundDrag(groundFriction);
            gas.ApplyDrag(dt);
        }

        // Les particules de fumee (visuelles) suivent le fluide : une fois par frame suffit.
        gas.AdvectMarkers(Time.fixedDeltaTime);
        RefreshInfoText();
    }

    void Predict(float dt)
    {
        Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        float damping = Mathf.Max(0f, 1f - airDamping * dt);

        for (int i = 0; i < allParticles.Count; i++)
        {
            Particle p = allParticles[i];
            previousPositions[i] = p.position;

            if (p.phase < 0) continue;
            if (p.IsFixed) { p.velocity = Vector3.zero; continue; }

            float g = (p.phase == gas.GasPhase) ? gas.GravityScale : 1f;
            p.velocity += gravity * (g * dt);
            p.velocity *= damping;
            p.position += p.velocity * dt;
        }
    }

    void UpdateVelocities(float dt)
    {
        for (int i = 0; i < allParticles.Count; i++)
        {
            Particle p = allParticles[i];
            if (p.phase < 0) continue;
            if (p.IsFixed) { p.velocity = Vector3.zero; continue; }
            Vector3 v = (p.position - previousPositions[i]) / dt;
            float speed = Vec.Length(v);
            if (speed > maxSpeed) v *= maxSpeed / speed;
            p.velocity = v;
        }
    }

    // ============================================
    // AFFICHAGE
    // ============================================

    void LateUpdate()
    {
        for (int i = 0; i < renderers.Count; i++) renderers[i].Update();
    }

    string infoText = "";
    int lastInfoTick = -1;

    // OnGUI est appele plusieurs fois par frame : fabriquer le texte la-bas genere du garbage (lag par GC).
    // On le reconstruit seulement 4 fois par seconde de simulation.
    void RefreshInfoText()
    {
        int tick = Mathf.FloorToInt(simulationTime * 4f);
        if (tick == lastInfoTick) return;
        lastInfoTick = tick;

        int cubeParticles = 0;
        for (int i = 0; i < cubes.Count; i++) cubeParticles += cubes[i].Particles.Count;

        infoText =
            "PARTICLE PHYSICS ENGINE\n" +
            "Time: " + simulationTime.ToString("F2") + " s\n" +
            "Cloth: " + cloth.Particles.Count + "\n" +
            "Cube particles: " + cubeParticles + "\n" +
            "Gas (alive): " + gas.AliveCount() + " / " + gas.GasParticles.Count + "\n" +
            "Substeps: " + substeps + "  Iterations: " + solverIterations;

    }

    void OnGUI()
    {
        if (cloth == null) return;
        GUI.Label(new Rect(10, 10, 400, 220), infoText);
    }
}