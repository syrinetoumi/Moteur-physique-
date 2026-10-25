// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Fumee en frontiere ouverte (article NVIDIA, sec. 7.2) :
///  - GasParticles : particules FLUIDES (les spheres cyan de la video). Elles portent la physique :
///    contrainte de densite unilaterale (eq. 26), gravite reduite (elles montent), trainee (eq. 29).
///  - SmokeMarkers : particules de fumee VISUELLES, deplacees passivement par la vitesse des
///    particules fluides (eq. 28). Elles ne sont pas dessinees dans la video.
/// Le couplage avec le tissu et les cubes se fait dans ParticleCollisionSolver : meme grille, memes contacts.
/// Une particule de phase -1 est inactive (morte ou pas encore emise).
/// Non implemente : turbulence barocline (eq. 30-32).
/// </summary>
public class GasSimulation
{
    public readonly List<Particle> GasParticles = new List<Particle>();
    public readonly List<Particle> SmokeMarkers = new List<Particle>();

    public bool IsActive = false;

    public int GasPhase = 500;
    public const int MarkerPhase = 501;
    public float GasRadius = 0.1f;
    public Vector3 EmitDirection = new Vector3(1f, 0.15f, 0f);   // direction du jet
    public float GasMass = 0.03f;

    public float GravityScale = -0.3f;       // < 0 : le gaz monte (sec. 7.2.1)
    public float Lifetime = 5f;               // secondes avant recyclage d une particule fluide
    public float EmitInterval = 0f;           // secondes entre deux couches (0 = automatique : espacement / vitesse)
    public float InnerSpeed;                  // le coeur est emis plus vite que le bord (sec. 7.2.2)
    public float OuterSpeed;
    public float DragK = 4f;                  // k de l eq. 29
    public Vector3 EnvironmentVelocity = new Vector3(0.8f, 0f, 0f);    // v_env de l eq. 29 (vent leger vers la tour)
    public bool RecycleOldest = true;        // pool plein : on recycle la plus ancienne particule, l'emission ne s'arrete jamais
    public bool NozzleWall = true;           // les particules ne repassent pas derriere le disque d'emission
    public float CfmEpsilon = 5f;             // relaxation de lambda
    public int MarkersPerParticle = 1;

    readonly Vector3 source;
    readonly float sourceRadius;

    // noyau SPH
    float H, h2, poly6K, spikyK, restDensity;
    readonly SpatialHashGrid grid;
    readonly List<int> tmp = new List<int>();
    readonly List<int> nbList = new List<int>();
    Vector3[] nbGrad = new Vector3[1];
    readonly int[] nbStart, nbCount;
    readonly float[] age, rho, lambda;
    readonly Vector3[] dp;
    readonly float[] markerAge;

    float emitTimer;
    int nextGas, nextMarker;
    uint randomState = 123456789u;

    static readonly Vector3 Parked = new Vector3(0f, -1000f, 0f);

    float Random01()
    {
        randomState = randomState * 1664525u + 1013904223u;
        return (randomState & 0x00FFFFFFu) / 16777216f;
    }

    public GasSimulation(Vector3 smokeSource, float radius, float speed, int gasCount, int markerCount, float gasMass = 0.03f, float gasRadius = 0.1f)
    {
        GasMass = gasMass;
        GasRadius = gasRadius;
        source = smokeSource;
        sourceRadius = radius;
        InnerSpeed = speed;
        OuterSpeed = speed * 0.65f;

        H = 3f * GasRadius;
        h2 = H * H;
        poly6K = 315f / (64f * Mathf.PI * Mathf.Pow(H, 9f));
        spikyK = -45f / (Mathf.PI * Mathf.Pow(H, 6f));
        restDensity = ComputeRestDensity(2f * GasRadius);

        for (int i = 0; i < gasCount; i++) GasParticles.Add(new Particle(Parked, GasMass, -1));
        for (int i = 0; i < markerCount; i++) SmokeMarkers.Add(new Particle(Parked, 0.001f, -1));

        grid = new SpatialHashGrid(H, gasCount);
        nbStart = new int[gasCount]; nbCount = new int[gasCount];
        age = new float[gasCount]; rho = new float[gasCount]; lambda = new float[gasCount];
        dp = new Vector3[gasCount];
        markerAge = new float[markerCount];
    }

    public void Activate() { IsActive = true; }

    // ============================================
    // NOYAUX SPH (Mueller 2003 / Macklin et Mueller 2013, cites par l article)
    // ============================================

    float Poly6(float r2)
    {
        if (r2 >= h2) return 0f;
        float t = h2 - r2;
        return poly6K * t * t * t;
    }

    Vector3 SpikyGrad(Vector3 r)
    {
        float len = Vec.Length(r);
        if (len < 1e-6f || len >= H) return Vector3.zero;
        float t = H - len;
        return r * (spikyK * t * t / len);
    }

    /// <summary>rho_0 : densite au centre d un reseau regulier de pas "spacing" (sec. 7.1.1).</summary>
    float ComputeRestDensity(float spacing)
    {
        int m = Mathf.CeilToInt(H / spacing);
        float sum = 0f;
        for (int a = -m; a <= m; a++)
            for (int b = -m; b <= m; b++)
                for (int c = -m; c <= m; c++)
                    sum += Poly6((a * a + b * b + c * c) * spacing * spacing);
        return sum;
    }

    // ============================================
    // EMISSION ET DUREE DE VIE (sec. 7.2.2)
    // ============================================

    public void UpdateEmission(float dt)
    {
        if (!IsActive) return;

        for (int i = 0; i < GasParticles.Count; i++)
        {
            Particle p = GasParticles[i];
            if (p.phase < 0) continue;
            age[i] += dt;
            if (age[i] > Lifetime) { p.phase = -1; p.position = Parked; p.velocity = Vector3.zero; }
        }
        for (int i = 0; i < SmokeMarkers.Count; i++)
        {
            Particle m = SmokeMarkers[i];
            if (m.phase < 0) continue;
            markerAge[i] += dt;
            if (markerAge[i] > Lifetime) { m.phase = -1; m.position = Parked; }
        }

        float interval = EmitInterval > 0f ? EmitInterval : (2f * GasRadius) / Mathf.Max(0.1f, InnerSpeed);
        emitTimer += dt;
        while (emitTimer >= interval)
        {
            emitTimer -= interval;
            EmitLayer();
        }
    }

    int FreeGas()
    {
        for (int s = 0; s < GasParticles.Count; s++)
        {
            int i = (nextGas + s) % GasParticles.Count;
            if (GasParticles[i].phase < 0) { nextGas = (i + 1) % GasParticles.Count; return i; }
        }
        if (!RecycleOldest) return -1;

        // Plus de place : la fumee doit couler EN CONTINU comme dans la video, on reutilise la plus vieille.
        int oldest = -1;
        float oldestAge = -1f;
        for (int i = 0; i < GasParticles.Count; i++)
            if (age[i] > oldestAge) { oldestAge = age[i]; oldest = i; }
        return oldest;
    }

    int FreeMarker()
    {
        for (int s = 0; s < SmokeMarkers.Count; s++)
        {
            int i = (nextMarker + s) % SmokeMarkers.Count;
            if (SmokeMarkers[i].phase < 0) { nextMarker = (i + 1) % SmokeMarkers.Count; return i; }
        }
        return -1;
    }

    /// <summary>
    /// Une couche de particules fluides en disque PERPENDICULAIRE a la direction du jet ;
    /// le coeur part plus vite que le bord (sec. 7.2.2).
    /// </summary>
    void EmitLayer()
    {
        float spacing = 2f * GasRadius;
        Vector3 dir = Vec.Normalize(EmitDirection);
        if (Vec.LengthSquared(dir) < 0.5f) dir = new Vector3(0f, 1f, 0f);
        Vector3 helper = Mathf.Abs(dir.y) < 0.9f ? new Vector3(0f, 1f, 0f) : new Vector3(1f, 0f, 0f);
        Vector3 u = Vec.Normalize(Vec.Cross(dir, helper));
        Vector3 w = Vec.Cross(u, dir);

        int k = Mathf.CeilToInt(sourceRadius / spacing);
        for (int a = -k; a <= k; a++)
            for (int b = -k; b <= k; b++)
            {
                Vector3 off = u * (a * spacing) + w * (b * spacing);
                float dist = Vec.Length(off);
                if (dist > sourceRadius + 1e-4f) continue;
                int i = FreeGas();
                if (i < 0) return;
                bool inner = dist <= sourceRadius - spacing * 0.5f;
                Particle p = GasParticles[i];
                p.position = source + off;
                p.velocity = dir * (inner ? InnerSpeed : OuterSpeed)
                           + new Vector3(Random01() - 0.5f, Random01() - 0.5f, Random01() - 0.5f) * 0.1f;
                p.phase = GasPhase;
                age[i] = 0f;
                if (!inner) continue;
                for (int q = 0; q < MarkersPerParticle; q++)
                {
                    int m = FreeMarker();
                    if (m < 0) break;
                    Particle mk = SmokeMarkers[m];
                    mk.position = p.position + new Vector3(Random01() - 0.5f, Random01() - 0.5f, Random01() - 0.5f) * spacing;
                    mk.velocity = p.velocity;
                    mk.phase = MarkerPhase;
                    markerAge[m] = 0f;
                }
            }
    }

    // ============================================
    // CONTRAINTE DE DENSITE (eq. 26, Jacobi)
    // ============================================

    /// <summary>Sec. 9 : voisins fluides, calcules une fois par sous-pas (positions predites).</summary>
    public void BuildNeighbors()
    {
        nbList.Clear();
        grid.Clear();
        for (int i = 0; i < GasParticles.Count; i++)
            if (GasParticles[i].phase >= 0) grid.Insert(i, GasParticles[i].position);
        for (int i = 0; i < GasParticles.Count; i++)
        {
            nbStart[i] = nbList.Count; nbCount[i] = 0;
            Particle p = GasParticles[i];
            if (p.phase < 0) continue;
            grid.Query(p.position, H, tmp);
            for (int k = 0; k < tmp.Count; k++)
            {
                int j = tmp[k];
                if (Vec.LengthSquared(p.position - GasParticles[j].position) < h2) nbList.Add(j);
            }
            nbCount[i] = nbList.Count - nbStart[i];
        }
        if (nbGrad.Length < nbList.Count) nbGrad = new Vector3[nbList.Count * 2];
    }

    /// <summary>
    /// C = rho_i / rho_0 - 1 &lt;= 0 (unilaterale : elle ne fait que separer).
    /// lambda_i = -C / (somme |grad C|^2 + eps) ; dx_i = 1/rho_0 * somme (lambda_i + lambda_j) grad W.
    /// </summary>
    public void SolveDensity()
    {
        int n = GasParticles.Count;
        for (int i = 0; i < n; i++)
        {
            lambda[i] = 0f; rho[i] = 0f;
            Particle p = GasParticles[i];
            if (p.phase < 0) continue;
            float dens = 0f;
            Vector3 gradI = Vector3.zero;
            float sumGrad2 = 0f;
            for (int k = 0; k < nbCount[i]; k++)
            {
                int j = nbList[nbStart[i] + k];
                Vector3 r = p.position - GasParticles[j].position;
                dens += Poly6(Vec.LengthSquared(r));
                if (j == i) continue;
                Vector3 sg = SpikyGrad(r);
                nbGrad[nbStart[i] + k] = sg;
                Vector3 gj = sg * (1f / restDensity);
                gradI += gj;
                sumGrad2 += Vec.LengthSquared(gj);
            }
            sumGrad2 += Vec.LengthSquared(gradI);
            rho[i] = dens;
            float C = dens / restDensity - 1f;
            lambda[i] = (C > 0f) ? -C / (sumGrad2 + CfmEpsilon) : 0f;
        }
        for (int i = 0; i < n; i++)
        {
            dp[i] = Vector3.zero;
            if (GasParticles[i].phase < 0) continue;
            Vector3 d = Vector3.zero;
            for (int k = 0; k < nbCount[i]; k++)
            {
                int j = nbList[nbStart[i] + k];
                if (j == i) continue;
                d += nbGrad[nbStart[i] + k] * (lambda[i] + lambda[j]);
            }
            dp[i] = d * (1f / restDensity);
        }
        for (int i = 0; i < n; i++)
            if (GasParticles[i].phase >= 0) GasParticles[i].position += dp[i];

        // Buse : un jet sort d'un orifice, le fluide ne peut pas refluer derriere le disque d'emission
        // (dans la video le bord gauche de la fumee reste fixe a l'emetteur).
        if (NozzleWall)
        {
            Vector3 dir = Vec.Normalize(EmitDirection);
            for (int i = 0; i < n; i++)
            {
                Particle p = GasParticles[i];
                if (p.phase < 0) continue;
                float s = Vec.Dot(p.position - source, dir);
                if (s < 0f) p.position -= dir * s;
            }
        }
    }

    /// <summary>Trainee eq. 29 : f = -k (v - v_env)(1 - rho_i/rho_0). Seules les particules proches de la surface sont freinees.</summary>
    public void ApplyDrag(float dt)
    {
        for (int i = 0; i < GasParticles.Count; i++)
        {
            Particle p = GasParticles[i];
            if (p.phase < 0) continue;
            float fac = 1f - rho[i] / restDensity;
            if (fac < 0f) fac = 0f; else if (fac > 1f) fac = 1f;
            p.velocity += (EnvironmentVelocity - p.velocity) * (DragK * fac * dt);
        }
    }

    // ============================================
    // PARTICULES DE FUMEE (eq. 28)
    // ============================================

    /// <summary>v(xs) = somme(vj W(xs - xj)) / somme(W(xs - xj)). Sans voisin fluide : mouvement balistique amorti.</summary>
    public void AdvectMarkers(float dt)
    {
        if (!IsActive) return;
        for (int m = 0; m < SmokeMarkers.Count; m++)
        {
            Particle mk = SmokeMarkers[m];
            if (mk.phase < 0) continue;
            grid.Query(mk.position, H, tmp);
            Vector3 sumWV = Vector3.zero;
            float sumW = 0f;
            for (int k = 0; k < tmp.Count; k++)
            {
                Particle g = GasParticles[tmp[k]];
                float w = Poly6(Vec.LengthSquared(mk.position - g.position));
                if (w <= 0f) continue;
                sumWV += g.velocity * w; sumW += w;
            }
            if (sumW > 1e-12f) mk.velocity = sumWV * (1f / sumW);
            else mk.velocity = mk.velocity * (1f - 0.5f * dt);
            mk.position += mk.velocity * dt;
        }
    }

    public int AliveCount()
    {
        int c = 0;
        for (int i = 0; i < GasParticles.Count; i++) if (GasParticles[i].phase >= 0) c++;
        return c;
    }
}