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
            World.FixedDt = 1f / 60f;
            World.MaxStepsPerUpdate = 2;
            Solver.Iterations = 2;
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
            const int detail = 3;
            const float spacing = 0.5f / detail;
            for (int y = 0; y < 9 * detail; y++)
            for (int x = -7 * detail; x <= 7 * detail; x++)
            for (int z = -5 * detail; z <= 5 * detail; z++)
            {
                int ax = Mathf.Abs(x), az = Mathf.Abs(z);
                // Les tours sont creuses pour éviter les particules cachées.
                bool tower = ax >= 5 * detail && az >= 3 * detail
                    && (ax == 5 * detail || ax == 7 * detail || az == 3 * detail || az == 5 * detail);
                bool edge = ax == 7 * detail || az == 5 * detail;
                bool wall = edge && y < 5 * detail;
                bool battlement = y >= 5 * detail && y < 6 * detail && edge
                    && ((x + 7 * detail) / detail + (z + 5 * detail) / detail) % 2 == 0;
                bool door = z == -5 * detail && ax <= detail && y < 3 * detail;
                bool towerTop = y >= 8 * detail && ax != 7 * detail && az != 5 * detail;
                if ((!tower && !wall && !battlement) || door || towerTop) continue;
                Vector3 position = new Vector3(x * spacing, spacing * 0.5f + y * spacing, z * spacing);
                // La première rangée forme les fondations.
                points.Add(new Vector3Int(x, y, z), Add(position, y == 0 ? 0f : 1f, spacing * 0.5f));
            }

            // La masse totale reste proche de celle du premier château.
            foreach (Particle p in Group.Particles)
                if (!p.IsFixed) p.Mass = 451f / Group.Particles.Count;

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
