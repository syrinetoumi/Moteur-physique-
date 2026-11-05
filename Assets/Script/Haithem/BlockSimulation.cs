using System.Collections.Generic;
using PhysicsLib;
using UnityEngine;

namespace Haithem.ParticleBlock
{
    public class BlockSimulation
    {
        public const int Width = 24, Height = 20, Depth = 10;
        public readonly PhysicsWorld World = new PhysicsWorld();
        public readonly ParticleGroup Group = new ParticleGroup();
        public readonly ParticleSolver Solver = new ParticleSolver();
        public readonly List<Vector3> InitialPositions = new List<Vector3>();
        public BlockSimulation(float strength = 0.04f, float bounce = 0.45f,
            float friction = 0.45f, float taille = 1f)
        {
            World.FixedDt = 1f / 60f;
            World.MaxStepsPerUpdate = 2;
            Solver.Iterations = 2;
            Group.Integrator = new EulerIntegrator();
            Group.Solver = Solver;
            Solver.BreakStrain = Mathf.Max(0.01f, strength);
            Solver.Restitution = Mathf.Clamp01(bounce);
            Solver.Friction = Mathf.Clamp01(friction);
            Group.Forces.Add(new GravityForce());
            Group.Forces.Add(new ViscousDragForce(0.02f));
            World.AddParticleGroup(Group);

            var points = new Dictionary<Vector3Int, int>();
            float spacing = Mathf.Max(0.01f, taille) / 6f;
            for (int y = 0; y < Height; y++)
            for (int x = 0; x < Width; x++)
            for (int z = 0; z < Depth; z++)
            {
                Vector3 position = new Vector3((x - (Width - 1) * 0.5f) * spacing,
                    spacing * 0.5f + y * spacing, (z - (Depth - 1) * 0.5f) * spacing);
                // La première rangée forme les fondations.
                points.Add(new Vector3Int(x, y, z), Add(position, y == 0 ? 0f : 1f, spacing * 0.5f));
            }

            // On répartit la masse entre les petites boules.
            foreach (Particle p in Group.Particles)
                if (!p.IsFixed) p.Mass = 451f / Group.Particles.Count;

            // Les diagonales renforcent le bloc.
            foreach (var point in points)
            for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
            for (int z = -1; z <= 1; z++)
            {
                int other;
                if (points.TryGetValue(point.Key + new Vector3Int(x, y, z), out other)
                    && other > point.Value)
                    Solver.Connect(Group.Particles, point.Value, other);
            }
        }

        int Add(Vector3 position, float mass, float radius)
        {
            InitialPositions.Add(position);
            return Group.Add(new Particle(position, mass) { Radius = radius });
        }

        public Particle LancerProjectile(Vector3 position, Vector3 cible,
            float rayon, float masse, float vitesse)
        {
            int index = Add(position, Mathf.Max(0.1f, masse), Mathf.Max(0.01f, rayon));
            Particle boule = Group.Particles[index];
            boule.velocity = (cible - position).normalized * Mathf.Max(0f, vitesse);
            return boule;
        }
    }
}
