// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Contacts entre particules (spheres de rayons differents) et avec le sol.
/// Article NVIDIA : contrainte de contact C = |xi - xj| - (ri + rj) >= 0 (eq. 22),
/// friction au niveau des positions (eq. 23-24), recherche de voisins par grille (sec. 9).
/// Une seule grille et une seule liste de paires par sous-pas, pour TOUTES les particules
/// (tissu, cubes, gaz) : c est ce qui permet le couplage tissu - cubes - fumee.
/// Les particules d une meme phase ne se repoussent pas (cube rigide, tissu, fluide).
/// Une particule de phase negative est "morte" (ignoree).
/// </summary>
public sealed class ParticleCollisionSolver
{
    public float GroundY = 0f;
    public float MuStatic = 0.5f;
    public float MuKinetic = 0.3f;
    public float ContactMargin = 1.15f;

    readonly List<Particle> particles;
    readonly float[] radii;
    readonly Vector3[] previous;      // positions au debut du sous-pas (pour la friction)
    readonly SpatialHashGrid grid;
    readonly float maxRadius;
    readonly List<int> pairs = new List<int>();
    readonly List<int> tmp = new List<int>();

    public ParticleCollisionSolver(List<Particle> all, float[] radiusPerParticle, Vector3[] previousPositions)
    {
        particles = all;
        radii = radiusPerParticle;
        previous = previousPositions;
        float m = 0f;
        for (int i = 0; i < radii.Length; i++) if (radii[i] > m) m = radii[i];
        maxRadius = m;
        grid = new SpatialHashGrid(2f * maxRadius * ContactMargin, all.Count);
    }

    /// <summary>Sec. 9 : voisins calcules une fois par sous-pas avec les positions predites.</summary>
    public void BuildPairs()
    {
        pairs.Clear();
        grid.Clear();
        for (int i = 0; i < particles.Count; i++)
            if (particles[i].phase >= 0) grid.Insert(i, particles[i].position);

        for (int i = 0; i < particles.Count; i++)
        {
            Particle a = particles[i];
            if (a.phase < 0) continue;
            grid.Query(a.position, (radii[i] + maxRadius) * ContactMargin, tmp);
            for (int k = 0; k < tmp.Count; k++)
            {
                int j = tmp[k];
                if (j <= i) continue;
                Particle b = particles[j];
                if (a.IsFixed && b.IsFixed) continue;
                if (a.phase == b.phase) continue;
                float rr = (radii[i] + radii[j]) * ContactMargin;
                if (Vec.LengthSquared(a.position - b.position) < rr * rr) { pairs.Add(i); pairs.Add(j); }
            }
        }
    }

    /// <summary>Un passage de Gauss-Seidel sur les contacts particule - particule.</summary>
    public void SolvePairs(bool friction)
    {
        for (int k = 0; k < pairs.Count; k += 2)
        {
            int i = pairs[k], j = pairs[k + 1];
            Particle a = particles[i], b = particles[j];
            float minDist = radii[i] + radii[j];
            Vector3 d = a.position - b.position;
            float s2 = Vec.LengthSquared(d);
            if (s2 >= minDist * minDist || s2 < 1e-12f) continue;

            float wa = a.InvMass, wb = b.InvMass, ws = wa + wb;
            if (ws <= 0f) continue;
            float dist = Mathf.Sqrt(s2);
            Vector3 n = d * (1f / dist);
            float pen = minDist - dist;
            float ca = wa / ws, cb = wb / ws;

            a.position += n * (pen * ca);
            b.position -= n * (pen * cb);

            if (friction)
            {
                // deplacement tangentiel relatif pendant ce pas (eq. 23)
                Vector3 rel = (a.position - previous[i]) - (b.position - previous[j]);
                Vector3 relT = rel - n * Vec.Dot(rel, n);
                float l2 = Vec.LengthSquared(relT);
                if (l2 > 1e-14f)
                {
                    float l = Mathf.Sqrt(l2);
                    // statique : on annule le glissement ; cinetique : Coulomb borne par la penetration (eq. 24)
                    float f = (l < MuStatic * pen) ? 1f : Mathf.Min(MuKinetic * pen / l, 1f);
                    Vector3 dx = relT * (-f);      // la friction s oppose au glissement
                    a.position += dx * ca;
                    b.position -= dx * cb;
                }
            }
        }
    }

    /// <summary>Sol invisible y = GroundY : contact + friction (masse infinie cote sol).</summary>
    public void SolveGround(bool friction)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Particle p = particles[i];
            if (p.phase < 0 || p.IsFixed) continue;
            float pen = GroundY + radii[i] - p.position.y;
            if (pen <= 0f) continue;
            Vector3 pos = p.position;
            pos.y += pen;
            if (friction)
            {
                Vector3 rel = pos - previous[i];
                Vector3 relT = new Vector3(rel.x, 0f, rel.z);
                float l2 = Vec.LengthSquared(relT);
                if (l2 > 1e-14f)
                {
                    float l = Mathf.Sqrt(l2);
                    float f = (l < MuStatic * pen) ? 1f : Mathf.Min(MuKinetic * pen / l, 1f);
                    pos -= relT * f;
                }
            }
            p.position = pos;
        }
    }

    /// <summary>
    /// Dissipation au contact du sol : la friction de position (eq. 24) est bornee par la profondeur de penetration,
    /// donc tres faible avec de petits pas de temps (limite signalee par l'article, sec. 6.1).
    /// On amortit en plus la vitesse horizontale des particules qui touchent le sol.
    /// </summary>
    public void ApplyGroundDrag(float factor)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Particle p = particles[i];
            if (p.phase < 0 || p.IsFixed) continue;
            if (p.position.y > GroundY + radii[i] + 0.02f) continue;
            Vector3 v = p.velocity;
            v.x *= 1f - factor;
            v.z *= 1f - factor;
            if (v.y < 0f) v.y = 0f;
            p.velocity = v;
        }
    }
}

// ====================================================================================
// Grille de hachage spatiale (article NVIDIA, section 9) - dans ce fichier pour qu'il
// n'y ait qu'UN seul fichier a copier. Ne pas ajouter de SpatialHashGrid.cs en plus.
// Liste chainee par cellule (head / next) : aucune allocation par frame.
// Table de taille 2^n (masque binaire) et dedoublonnage O(1) par horodatage.
// Contrainte : rayon de requete <= 2 x cellSize.
// ====================================================================================
public sealed class SpatialHashGrid
{
    readonly float inv;
    readonly int mask;
    readonly int[] head;
    readonly int[] next;
    readonly int[] stamp;
    int queryId;

    public SpatialHashGrid(float cellSize, int capacity, int tableSizePowerOfTwo = 4096)
    {
        inv = 1f / cellSize;
        mask = tableSizePowerOfTwo - 1;
        head = new int[tableSizePowerOfTwo];
        stamp = new int[tableSizePowerOfTwo];
        next = new int[capacity];
        Clear();
    }

    public void Clear()
    {
        for (int i = 0; i < head.Length; i++) head[i] = -1;
    }

    int Hash(int ix, int iy, int iz)
    {
        unchecked
        {
            return ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) & mask;
        }
    }

    public void Insert(int idx, Vector3 p)
    {
        int b = Hash(Mathf.FloorToInt(p.x * inv), Mathf.FloorToInt(p.y * inv), Mathf.FloorToInt(p.z * inv));
        next[idx] = head[b];
        head[b] = idx;
    }

    /// <summary>Candidats proches de p (a verifier avec la vraie distance). Pas de doublons.</summary>
    public void Query(Vector3 p, float radius, List<int> result)
    {
        result.Clear();
        queryId++;
        int x0 = Mathf.FloorToInt((p.x - radius) * inv), x1 = Mathf.FloorToInt((p.x + radius) * inv);
        int y0 = Mathf.FloorToInt((p.y - radius) * inv), y1 = Mathf.FloorToInt((p.y + radius) * inv);
        int z0 = Mathf.FloorToInt((p.z - radius) * inv), z1 = Mathf.FloorToInt((p.z + radius) * inv);
        for (int ix = x0; ix <= x1; ix++)
            for (int iy = y0; iy <= y1; iy++)
                for (int iz = z0; iz <= z1; iz++)
                {
                    int b = Hash(ix, iy, iz);
                    if (stamp[b] == queryId) continue;
                    stamp[b] = queryId;
                    for (int e = head[b]; e >= 0; e = next[e]) result.Add(e);
                }
    }
}