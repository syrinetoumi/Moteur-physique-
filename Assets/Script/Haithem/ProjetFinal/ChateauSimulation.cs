using System.Collections.Generic;
using PhysicsLib;
using UnityEngine;

namespace Haithem.ProjetFinal
{
    public class ChateauSimulation
    {
        public readonly PhysicsWorld World = new PhysicsWorld();
        public readonly ParticleGroup Group = new ParticleGroup();
        public readonly ParticleSolver Solver = new ParticleSolver();
        public readonly List<Vector3> InitialPositions = new List<Vector3>();
        public int ProjectileIndex { get; private set; }
        public bool Fired { get; private set; }
        public float ProjectileMass = 500f;
        public float ProjectileSpeed = 20f;

        public ChateauSimulation(float strength = 0.04f, float bounce = 0.45f, float friction = 0.45f)
        {
            World.FixedDt = 1f / 120f;
            World.MaxStepsPerUpdate = 12;
            Group.Integrator = new EulerIntegrator();
            Group.Solver = Solver;
            // Le château attend au repos, puis se réveille au tir.
            Group.IsSleeping = true;
            Solver.BreakStrain = Mathf.Max(0.01f, strength);
            Solver.Restitution = Mathf.Clamp01(bounce);
            Solver.Friction = Mathf.Clamp01(friction);
            Group.Forces.Add(new GravityForce());
            Group.Forces.Add(new ViscousDragForce(0.02f));
            World.AddParticleGroup(Group);

            var points = new Dictionary<Vector3Int, int>();
            for (int y = 0; y < 9; y++)
            for (int x = -7; x <= 7; x++)
            for (int z = -5; z <= 5; z++)
            {
                bool tower = Mathf.Abs(x) >= 5 && Mathf.Abs(z) >= 3;
                bool wall = (Mathf.Abs(x) == 7 || Mathf.Abs(z) == 5) && y < 5;
                bool battlement = y == 5 && (Mathf.Abs(x) == 7 || Mathf.Abs(z) == 5)
                    && (x + z) % 2 == 0;
                bool door = z == -5 && Mathf.Abs(x) <= 1 && y < 3;
                bool towerTop = y == 8 && Mathf.Abs(x) != 7 && Mathf.Abs(z) != 5;
                if ((!tower && !wall && !battlement) || door || towerTop) continue;
                Vector3 position = new Vector3(x * 0.5f, 0.25f + y * 0.5f, z * 0.5f);
                // La première rangée forme les fondations.
                points.Add(new Vector3Int(x, y, z), Add(position, y == 0 ? 0f : 1f, 0.25f));
            }

            // Les diagonales empêchent les murs de se plier tout seuls.
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
            ProjectileIndex = Add(new Vector3(0f, 3.2f, -11f), 0f, 2.8f);
        }

        int Add(Vector3 position, float mass, float radius)
        {
            InitialPositions.Add(position);
            return Group.Add(new Particle(position, mass) { Radius = radius });
        }

        public void Fire()
        {
            if (Fired) return;
            Fired = true;
            Group.IsSleeping = false;
            Particle ball = Group.Particles[ProjectileIndex];
            ball.Mass = Mathf.Clamp(ProjectileMass, 1f, 500f);
            Vector3 aim = new Vector3(0f, 5f, 2f);
            ball.velocity = (aim - ball.position).normalized * Mathf.Clamp(ProjectileSpeed, 5f, 60f);
        }
    }
}
