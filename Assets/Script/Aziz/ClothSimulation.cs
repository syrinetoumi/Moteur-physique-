// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Tissu = grille de particules reliees par des contraintes de distance (article NVIDIA, sec. 8 ;
/// Ch.2 partie 1, diapo 29 : ressorts de structure, de cisaillement et de flexion).
/// Chaque famille a sa raideur : un tissu dont la flexion est aussi raide que l etirement
/// se comporte comme une plaque rigide au lieu de plier comme dans la video.
/// </summary>
public class ClothSimulation
{
    public struct DistanceConstraint
    {
        public int A;
        public int B;
        public float RestLength;
        public float Stiffness;

        public DistanceConstraint(int a, int b, float r, float k)
        {
            A = a;
            B = b;
            RestLength = r;
            Stiffness = k;
        }
    }

    public readonly List<Particle> Particles = new List<Particle>();
    public readonly List<DistanceConstraint> Constraints = new List<DistanceConstraint>();

    public int Width { get; private set; }
    public int Height { get; private set; }
    public float Radius = 0.08f;
    public float ParticleMass = 0.05f;

    public float StructuralStiffness = 1.0f;
    public float ShearStiffness = 0.6f;
    public float BendStiffness = 0.08f;

    // Copies en tableaux pour la boucle chaude (plus rapide que List<struct>).
    int[] cA, cB;
    float[] cRest, cK;
    Particle[] parts;

    public ClothSimulation(int width, int height, float spacing, Vector3 topRightCorner, float particleMass = 0.05f)
    {
        ParticleMass = particleMass;
        Width = width;
        Height = height;

        // Tissu accroche par sa ligne du haut ; il pend vers -Y et s etend vers -X.
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Vector3 position = new Vector3(
                    topRightCorner.x - (width - 1 - x) * spacing,
                    topRightCorner.y - y * spacing,
                    topRightCorner.z);

                bool pinned = (y == 0);
                // phase 1 = tissu
                Particles.Add(new Particle(position, pinned ? 0f : ParticleMass, 1));
            }
        }

        BuildConstraints();
        CacheConstraintArrays();
    }

    public int Index(int x, int y)
    {
        return y * Width + x;
    }

    void Connect(int a, int b, float stiffness)
    {
        float rest = Vec.Distance(Particles[a].position, Particles[b].position);
        Constraints.Add(new DistanceConstraint(a, b, rest, stiffness));
    }

    void BuildConstraints()
    {
        for (int y = 0; y < Height; y++)
        {
            for (int x = 0; x < Width; x++)
            {
                int idx = Index(x, y);

                // Structure (etirement).
                if (x + 1 < Width) Connect(idx, Index(x + 1, y), StructuralStiffness);
                if (y + 1 < Height) Connect(idx, Index(x, y + 1), StructuralStiffness);

                // Cisaillement.
                if (x + 1 < Width && y + 1 < Height)
                {
                    Connect(idx, Index(x + 1, y + 1), ShearStiffness);
                    Connect(Index(x + 1, y), Index(x, y + 1), ShearStiffness);
                }

                // Flexion (une raideur faible laisse le tissu plier).
                if (x + 2 < Width) Connect(idx, Index(x + 2, y), BendStiffness);
                if (y + 2 < Height) Connect(idx, Index(x, y + 2), BendStiffness);
            }
        }
    }

    void CacheConstraintArrays()
    {
        int n = Constraints.Count;
        cA = new int[n]; cB = new int[n]; cRest = new float[n]; cK = new float[n];
        for (int i = 0; i < n; i++)
        {
            cA[i] = Constraints[i].A; cB[i] = Constraints[i].B;
            cRest[i] = Constraints[i].RestLength; cK[i] = Constraints[i].Stiffness;
        }
        parts = Particles.ToArray();
    }

    /// <summary>Lache les particules fixees de la ligne du haut entre startX et endX (indices inclus).</summary>
    public void ReleaseTopRow(int startX, int endX)
    {
        for (int x = startX; x <= endX && x < Width; x++)
        {
            Particle p = Particles[Index(x, 0)];
            if (p.IsFixed) p.Mass = ParticleMass;
        }
    }

    /// <summary>Un passage de Gauss-Seidel sur les contraintes de distance (eq. 3-5 de l article).</summary>
    public void SolveConstraints()
    {
        for (int i = 0; i < cA.Length; i++)
        {
            Particle a = parts[cA[i]];
            Particle b = parts[cB[i]];

            Vector3 delta = b.position - a.position;
            float dist = Vec.Length(delta);
            if (dist < Vec.Epsilon) continue;

            float wA = a.InvMass;
            float wB = b.InvMass;
            float sum = wA + wB;
            if (sum <= 0f) continue;

            float error = dist - cRest[i];
            Vector3 correction = delta * (cK[i] * error / (dist * sum));

            a.position += correction * wA;
            b.position -= correction * wB;
        }
    }
}