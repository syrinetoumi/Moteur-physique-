// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Scene de la video (article NVIDIA, Fig. 16) : tissu vert, tour de 4 cubes, fumee a gauche.
/// Les valeurs par defaut viennent de MESURES faites sur la video (1 m = 36,5 px, cube de 1,6 m) :
///  - tour de 4 cubes 5x5x5 spheres, cube du haut jaune ; sol y = 0 (invisible, pas de mur) ;
///  - tissu = bande etroite (5 colonnes) de 7,2 m accrochee en haut (x = 1,4 ; y = 12,2) ;
///  - fumee = jet dense qui part de (-3,4 ; 3,3) vers la droite puis monte.
/// Chronologie : le tissu frappe le cube du haut -> la tour s'ecroule -> la fumee demarre a gauche
/// -> le tissu reste en l'air et brasse les particules de fumee.
/// Aucun Rigidbody, Collider ni Transform pour le mouvement.
/// Reglage conseille : Edit > Project Settings > Time > Fixed Timestep = 0.0166667.
/// Camera conseillee : position (1, 5, -22), regarde (1, 5.5, 0), champ de vision 35.
/// </summary>
public class SceneGenerator : MonoBehaviour
{
    [Header("Preset")]
    [Tooltip("COCHE : au demarrage, tous les reglages ci-dessous sont remplaces par le preset mesure sur la video. " +
             "Decocher pour utiliser vos propres valeurs. (Unity garde les anciennes valeurs de l'Inspector " +
             "quand le script change : cette case evite ce piege.)")]
    public bool useVideoPreset = true;

    [Header("Elements (decocher pour travailler un seul element)")]
    public bool enableCubes = true;
    public bool enableCloth = true;
    public bool enableSmoke = true;

    [Header("Physique")]
    [Tooltip("Gravite (m/s2 dans les unites de la scene). La video tombe et oscille ~4,5x plus vite qu'a 9,81 : mesure = ~44.")]
    public float gravity = 44f;

    [Header("Murs invisibles (collision seulement, rien n'est dessine)")]
    public bool enableWalls = true;
    public float wallMinX = -3.5f;
    public float wallMaxX = 12f;
    public float wallMinZ = -10f;
    public float wallMaxZ = 4f;

    [Header("Phase large (Chapitre 4)")]
    [Tooltip("Tissu - fumee : paires trouvees par le Sweep and Prune de la bibliotheque (BroadPhase.cs) au lieu de la grille.")]
    public bool useSAPForClothSmoke = true;

    [Header("Performance")]
    [Range(1, 4)]
    public int substeps = 2;

    [Range(1, 8)]
    public int solverIterations = 4;

    [Tooltip("Iterations de la contrainte de densite de la fumee par sous-pas.")]
    [Range(1, 8)]
    public int densityIterations = 2;

    [Header("Cubes")]
    [Tooltip("Particules par arete d'un cube (la video : 5).")]
    [Range(3, 6)]
    public int cubeResolution = 5;

    [Tooltip("Arete exterieure d'un cube (m).")]
    public float cubeEdge = 1.6f;

    [Tooltip("Masse totale d'un cube (kg).")]
    public float cubeMass = 3.048f;

    [Tooltip("Decalage lateral (m) des cubes impairs : une tour parfaitement alignee est trop stable pour s'ecrouler.")]
    public float cubeStackOffset = 0.053f;

    [Tooltip("Friction entre particules (statique, cinetique) : plus bas, les cubes glissent plus facilement.")]
    [Range(0f, 1f)]
    public float contactFriction = 0.632f;

    [Header("Cloth")]
    [Range(3, 12)]
    public int clothColumns = 5;

    [Range(10, 45)]
    public int clothRows = 36;

    [Tooltip("Position de l'accroche (x, y). Le tissu pend de ce point.")]
    public float clothPinX = 1.4f;
    public float clothTopY = 12.2f;

    [Tooltip("Nombre de colonnes accrochees au centre de la ligne du haut (1 = un seul point).")]
    [Range(1, 5)]
    public int clothPinnedColumns = 1;

    [Tooltip("Angle de depart (degres) : le bas du tissu est ecarte vers +X, comme au debut de la video.")]
    public float clothInitialAngle = 85.473f;

    [Tooltip("Vitesse angulaire de depart (rad/s). Negatif : le tissu balaye vers la tour (-X).")]
    public float clothAngularSpeed = 0f;

    [Tooltip("Vitesse laterale (z) du tissu : > 0 l'ecarte de la camera, < 0 le rapproche.")]
    public float clothSideSpeed = -0.369f;

    [Tooltip("Masse d'une particule de tissu (kg).")]
    public float clothParticleMass = 0.149f;

    [Header("Smoke")]
    [Tooltip("Centre du disque d'emission du jet.")]
    public Vector3 smokePosition = new Vector3(-3.4f, 3.35f, 0f);

    [Tooltip("Direction du jet (normalisee par le code).")]
    public Vector3 smokeDirection = new Vector3(1f, 0.1095485f, 0f);

    public float smokeSpeed = 8.840787f;
    public float smokeEmitRadius = 0.5149912f;

    [Tooltip("Secondes entre deux couches de particules (0 = automatique : espacement / vitesse).")]
    public float smokeEmitInterval = 0.05132967f;
    public float smokeStartTime = 1.35f;
    public float gasParticleRadius = 0.16f;
    public int gasParticleCount = 600;

    [Tooltip("Masse d'une particule de fumee (kg) : plus elle est grande, plus la fumee pousse le tissu.")]
    public float gasParticleMass = 0.02f;

    [Tooltip("< 0 : la fumee monte (gravite reduite, article 7.2.1).")]
    public float smokeBuoyancy = -0.1059023f;

    [Tooltip("k de la trainee (article eq. 29).")]
    public float smokeDrag = 9.170113f;

    [Tooltip("Duree de vie d'une particule de fumee (s).")]
    public float smokeLifetime = 9.139285f;

    public int smokeMarkerCount = 100;

    [Tooltip("Dessine aussi les marqueurs de fumee (eq. 28). Absents de la video.")]
    public bool showSmokeMarkers = false;

    [Header("Stabilite")]
    [Tooltip("Vitesse maximale d'une particule (m/s).")]
    public float maxSpeed = 40f;

    [Tooltip("Amortissement de l'air (1/s).")]
    public float airDamping = 0.8f;

    [Tooltip("Amortissement de l'air applique au tissu (1/s). 0 : il oscille longtemps comme dans la video.")]
    public float clothAirDamping = 0f;

    [Tooltip("Passes de resolution des contraintes du tissu par iteration. Plus de passes = moins d'energie perdue, tissu qui oscille plus longtemps.")]
    [Range(1, 4)]
    public int clothConstraintPasses = 2;

    [Tooltip("Amortissement horizontal au contact du sol (par sous-pas).")]
    [Range(0f, 0.5f)]
    public float groundFriction = 0.236f;

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
    bool smokeStarted;

    const float GroundY = 0f;
    const int FirstCubePhase = 100;
    const int GasPhaseId = 500;
    const int ClothPhaseId = 1;   // phase donnee par ClothSimulation


    // ============================================
    // PRESET DE LA VIDEO (valeurs mesurees / ajustees sur VIDEO.mp4)
    // Les cases Enable* et showSmokeMarkers ne sont pas touchees.
    // ============================================

    [ContextMenu("Appliquer le preset de la video")]
    public void ApplyVideoPreset()
    {
        gravity = 44f;
        enableWalls = true; wallMinX = -3.5f; wallMaxX = 12f; wallMinZ = -10f; wallMaxZ = 4f;

        substeps = 2; solverIterations = 4; densityIterations = 2;

        cubeResolution = 5; cubeEdge = 1.6f; cubeMass = 3.048f;
        cubeStackOffset = 0.053f; contactFriction = 0.632f;

        clothColumns = 5; clothRows = 36; clothPinX = 1.4f; clothTopY = 12.2f; clothPinnedColumns = 1;
        clothInitialAngle = 85.473f; clothAngularSpeed = 0f; clothSideSpeed = -0.369f; clothParticleMass = 0.149f;
        clothAirDamping = 0f; clothConstraintPasses = 2;

        smokePosition = new Vector3(-3.4f, 3.35f, 0f);
        smokeDirection = new Vector3(1f, 0.1095485f, 0f);
        smokeSpeed = 8.840787f; smokeEmitRadius = 0.5149912f; smokeEmitInterval = 0.05132967f;
        smokeStartTime = 1.35f; gasParticleRadius = 0.16f; gasParticleCount = 600; gasParticleMass = 0.02f;
        smokeBuoyancy = -0.1059023f; smokeDrag = 9.170113f; smokeLifetime = 9.139285f; smokeMarkerCount = 100;

        maxSpeed = 40f; airDamping = 0.8f; groundFriction = 0.236f;
    }

    // Appele par Unity quand le composant est ajoute ou reinitialise (menu du composant > Reset).
    void Reset()
    {
        ApplyVideoPreset();
    }

    void Start()
    {
        if (useVideoPreset)
        {
            ApplyVideoPreset();

            // Le resultat de l'ecroulement depend du pas de temps : le preset a ete regle a 1/60 s.
            Time.fixedDeltaTime = 1f / 60f;
        }
        GenerateScene();
    }

    void GenerateScene()
    {
        if (enableCubes) CreateCubes();
        if (enableCloth) CreateCloth();
        if (enableSmoke) CreateSmoke();
        CollectParticles();
    }

    // ============================================
    // TOUR DE 4 CUBES (du bas vers le haut : orange, bleu, vert, jaune)
    // ============================================

    void CreateCubes()
    {
        int resolution = Mathf.Max(3, cubeResolution);

        // Spheres qui se touchent : espacement = arete / resolution, rayon = demi-espacement.
        float spacing = cubeEdge / resolution;
        float lattice = cubeEdge - spacing;               // distance entre les centres des spheres extremes
        Vector3 size = new Vector3(lattice, lattice, lattice);

        // Cubes empiles couche contre couche : la tour tient debout.
        float cubeSpacing = cubeEdge + 0.005f;
        float firstCenterY = cubeEdge * 0.5f;

        Color[] colors =
        {
            new Color(1.00f, 0.55f, 0.10f),   // orange (bas)
            new Color(0.28f, 0.64f, 1.00f),   // bleu
            new Color(0.08f, 0.75f, 0.63f),   // vert
            new Color(0.90f, 0.78f, 0.12f)    // jaune (haut)
        };

        for (int i = 0; i < 4; i++)
        {
            float dx = (i % 2 == 1) ? cubeStackOffset : 0f;
            Vector3 center = new Vector3(dx, firstCenterY + i * cubeSpacing, 0f);

            RigidCubeSimulation cube = new RigidCubeSimulation(center, size, cubeMass, resolution, FirstCubePhase + i);
            cubes.Add(cube);

            renderers.Add(new ParticleSceneRenderer("Rigid Cube " + i, cube.Particles, cube.Radius, colors[i]));
        }
    }

    // ============================================
    // TISSU : bande etroite accrochee en haut, lancee vers la tour
    // ============================================

    void CreateCloth()
    {
        float spacing = 0.20f;

        // La ligne du haut est centree sur l'accroche.
        Vector3 topRight = new Vector3(clothPinX + (clothColumns - 1) * 0.5f * spacing, clothTopY, 0f);
        cloth = new ClothSimulation(clothColumns, clothRows, spacing, topRight, clothParticleMass);
        cloth.Radius = 0.10f;

        int mid = clothColumns / 2;
        int half = (clothPinnedColumns - 1) / 2;
        cloth.PinOnly(mid - half, mid + half);

        // Pose de depart de la video : tissu ecarte vers la droite, puis lance vers la tour.
        cloth.SetInitialSwing(new Vector3(clothPinX, clothTopY, 0f), clothInitialAngle, clothAngularSpeed);

        if (Mathf.Abs(clothSideSpeed) > 0f)
        {
            for (int i = 0; i < cloth.Particles.Count; i++)
            {
                Particle p = cloth.Particles[i];
                if (p.IsFixed) continue;
                p.velocity += new Vector3(0f, 0f, clothSideSpeed);
            }
        }

        renderers.Add(new ParticleSceneRenderer("Green Cloth", cloth.Particles, cloth.Radius, new Color(0.12f, 0.70f, 0.48f)));
    }

    // ============================================
    // FUMEE : jet de particules fluides cyan a gauche
    // ============================================

    void CreateSmoke()
    {
        gas = new GasSimulation(smokePosition, smokeEmitRadius, smokeSpeed,
                                Mathf.Max(1, gasParticleCount), Mathf.Max(1, smokeMarkerCount),
                                gasParticleMass, gasParticleRadius);
        gas.GasPhase = GasPhaseId;
        gas.EmitDirection = smokeDirection;
        gas.EmitInterval = smokeEmitInterval;
        gas.GravityScale = smokeBuoyancy;
        gas.DragK = smokeDrag;
        gas.Lifetime = smokeLifetime;
        gas.EnvironmentVelocity = Vector3.zero;

        renderers.Add(new ParticleSceneRenderer("Cyan Gas Particles", gas.GasParticles, gas.GasRadius, new Color(0.08f, 0.85f, 1.00f)));

        if (showSmokeMarkers)
            renderers.Add(new ParticleSceneRenderer("Smoke Markers", gas.SmokeMarkers, 0.05f, new Color(0.02f, 0.45f, 1.00f)));
    }

    void CollectParticles()
    {
        List<float> r = new List<float>();

        if (cloth != null)
        {
            clothFirst = allParticles.Count;
            allParticles.AddRange(cloth.Particles);
            for (int i = 0; i < cloth.Particles.Count; i++) r.Add(cloth.Radius);
        }

        for (int c = 0; c < cubes.Count; c++)
        {
            allParticles.AddRange(cubes[c].Particles);
            for (int i = 0; i < cubes[c].Particles.Count; i++) r.Add(cubes[c].Radius);
        }

        if (gas != null)
        {
            gasFirst = allParticles.Count;
            allParticles.AddRange(gas.GasParticles);
            for (int i = 0; i < gas.GasParticles.Count; i++) r.Add(gas.GasRadius);
        }

        radii = r.ToArray();
        previousPositions = new Vector3[allParticles.Count];
        for (int i = 0; i < allParticles.Count; i++) previousPositions[i] = allParticles[i].position;

        if (allParticles.Count > 0)
        {
            contacts = new ParticleCollisionSolver(allParticles, radii, previousPositions);
            contacts.GroundY = GroundY;
            contacts.EnableWalls = enableWalls;
            contacts.WallMinX = wallMinX; contacts.WallMaxX = wallMaxX;
            contacts.WallMinZ = wallMinZ; contacts.WallMaxZ = wallMaxZ;
            contacts.MuStatic = contactFriction;
            contacts.MuKinetic = contactFriction * 0.6f;

            if (useSAPForClothSmoke && cloth != null && gas != null)
                contacts.ClothSmokeBroadPhase = BuildClothSmokeBridge();
        }
    }

    int clothFirst = -1;
    int gasFirst = -1;
    ParticleSAPBridge sapBridge;

    /// <summary>Indices des particules de tissu et de fumee dans allParticles -> adaptateur du SAP de la bibliotheque.</summary>
    ParticleSAPBridge BuildClothSmokeBridge()
    {
        int[] clothIdx = new int[cloth.Particles.Count];
        for (int i = 0; i < clothIdx.Length; i++) clothIdx[i] = clothFirst + i;

        int[] gasIdx = new int[gas.GasParticles.Count];
        for (int i = 0; i < gasIdx.Length; i++) gasIdx[i] = gasFirst + i;

        sapBridge = new ParticleSAPBridge(allParticles, radii, clothIdx, gasIdx, ClothPhaseId, GasPhaseId);
        return sapBridge;
    }

    // ============================================
    // BOUCLE PHYSIQUE (Algorithm 1 de l'article)
    // ============================================

    void FixedUpdate()
    {
        if (contacts == null) return;

        int steps = Mathf.Max(1, substeps);
        float dt = Time.fixedDeltaTime / steps;

        for (int sub = 0; sub < steps; sub++)
        {
            simulationTime += dt;

            if (gas != null)
            {
                if (!smokeStarted && simulationTime >= smokeStartTime)
                {
                    gas.Activate();
                    smokeStarted = true;
                }
                gas.UpdateEmission(dt);
            }

            Predict(dt);

            contacts.BuildPairs();
            if (gas != null) gas.BuildNeighbors();

            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                if (cloth != null)
                    for (int pass = 0; pass < clothConstraintPasses; pass++) cloth.SolveConstraints();
                for (int i = 0; i < cubes.Count; i++) cubes[i].SolveShapeMatching(1f);
                if (gas != null && iteration < densityIterations) gas.SolveDensity();
                contacts.SolvePairs(true);
                contacts.SolveGround(true);
                contacts.SolveWalls();
            }

            UpdateVelocities(dt);
            contacts.ApplyGroundDrag(groundFriction);
            if (gas != null) gas.ApplyDrag(dt);
        }

        // Les particules de fumee (visuelles) suivent le fluide : une fois par frame suffit.
        if (gas != null) gas.AdvectMarkers(Time.fixedDeltaTime);
        RefreshInfoText();
    }

    void Predict(float dt)
    {
        Vector3 gravityVector = new Vector3(0f, -gravity, 0f);
        float damping = Mathf.Max(0f, 1f - airDamping * dt);
        float clothDamping = Mathf.Max(0f, 1f - clothAirDamping * dt);

        for (int i = 0; i < allParticles.Count; i++)
        {
            Particle p = allParticles[i];
            previousPositions[i] = p.position;

            if (p.phase < 0) continue;
            if (p.IsFixed) { p.velocity = Vector3.zero; continue; }

            float g = (p.phase == GasPhaseId && gas != null) ? gas.GravityScale : 1f;
            p.velocity += gravityVector * (g * dt);
            // La fumee n'est freinee que par la trainee de surface (eq. 29), pas par l'amortissement de l'air.
            if (p.phase == ClothPhaseId) p.velocity *= clothDamping;
            else if (p.phase != GasPhaseId) p.velocity *= damping;
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
            "Cloth: " + (cloth != null ? cloth.Particles.Count : 0) + "\n" +
            "Cube particles: " + cubeParticles + "\n" +
            (sapBridge != null
                ? "SAP tissu-fumee: " + sapBridge.LastTestCount + " tests, " + sapBridge.LastCandidatePairs + " paires (force brute: " + sapBridge.BruteForceTests + ")\n"
                : "Phase large: grille\n") +
            "Gas (alive): " + (gas != null ? gas.AliveCount() : 0) + " / " + (gas != null ? gas.GasParticles.Count : 0) + "\n" +
            "Substeps: " + substeps + "  Iterations: " + solverIterations;
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 400, 220), infoText);
    }
}
