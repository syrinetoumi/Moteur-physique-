using System;
using System.Diagnostics;
using PhysicsLib;
using UnityEditor;
using UnityEngine;

namespace Haithem.ProjetFinal
{
    public static class ChateauVerification
    {
        [MenuItem("Haithem/Projet final/Vérifier les calculs")]
        public static void Verifier()
        {
            ParticleGroup pair = Group();
            pair.Particles.Add(new Particle(new Vector3(-0.51f, 5f, 0f), 1f)
                { velocity = Vector3.right * 2f });
            pair.Particles.Add(new Particle(new Vector3(0.51f, 5f, 0f), 1f)
                { velocity = Vector3.left * 2f });
            pair.Step(0.01f);
            Check(Mathf.Abs(pair.Particles[0].velocity.x + 2f) < 0.001f,
                "Collision élastique : échange des vitesses");
            Check((pair.Particles[0].velocity + pair.Particles[1].velocity).magnitude < 0.001f,
                "Quantité de mouvement conservée");

            ParticleGroup floor = Group();
            floor.Particles.Add(new Particle(new Vector3(0f, 0.51f, 0f), 1f)
                { velocity = Vector3.down * 4f });
            floor.Step(0.01f);
            Check(Mathf.Abs(floor.Particles[0].velocity.y - 4f) < 0.001f,
                "Rebond sur le sol");
            floor.Forces.Add(new GravityForce());
            floor.Solver.Restitution = 0.4f;
            for (int i = 0; i < 1200; i++) floor.Step(1f / 120f);
            Check(Mathf.Abs(floor.Particles[0].position.y - 0.5f) < 0.002f
                && floor.Particles[0].velocity.magnitude < 0.05f, "Repos au sol");

            ParticleGroup link = Group();
            link.Particles.Add(new Particle(new Vector3(0f, 5f, 0f), 0f));
            link.Particles.Add(new Particle(new Vector3(1f, 5f, 0f), 1f));
            link.Solver.Connect(link.Particles, 0, 1);
            link.Particles[1].velocity = Vector3.right * 20f;
            link.Step(0.01f);
            Check(link.Solver.BrokenCount == 1, "Rupture par étirement");
            Check(link.Particles[0].position == new Vector3(0f, 5f, 0f), "Point fixe");

            ParticleGroup large = Group();
            large.Particles.Add(new Particle(new Vector3(-2.5f, 5f, -2f), 0f)
                { Radius = 2f });
            large.Particles.Add(new Particle(new Vector3(-0.1f, 5f, -2f), 1f));
            large.Step(0.01f);
            Check(Vector3.Distance(large.Particles[0].position, large.Particles[1].position) >= 2.499f,
                "Grille : grosse boule et coordonnées négatives");
            UnityEngine.Debug.Log("Projet final : collisions, repos et rupture OK");
        }

        static ParticleGroup Group()
        {
            return new ParticleGroup { Integrator = new EulerIntegrator(),
                Solver = new ParticleSolver { Restitution = 1f, Friction = 0f } };
        }

        [MenuItem("Haithem/Projet final/Vérifier l'effondrement")]
        public static void VerifierChateau()
        {
            UnityEngine.Debug.Log(Mesurer());
        }

        public static string Mesurer()
        {
            var watch = Stopwatch.StartNew();
            var castle = new ChateauSimulation();
            float dt = castle.World.FixedDt;
            for (int i = 0; i < Mathf.RoundToInt(3f / dt); i++) castle.World.Step(dt);
            float drift = Drift(castle);
            int before = castle.Solver.BrokenCount;
            castle.Fire();
            for (int i = 0; i < Mathf.RoundToInt(8f / dt); i++) castle.World.Step(dt);
            watch.Stop();
            int moved = 0, low = 0, high = 0;
            for (int i = 0; i < castle.ProjectileIndex; i++)
            {
                Particle p = castle.Group.Particles[i];
                Check(!float.IsNaN(p.position.sqrMagnitude) && !float.IsInfinity(p.position.sqrMagnitude)
                    && !float.IsNaN(p.velocity.sqrMagnitude) && !float.IsInfinity(p.velocity.sqrMagnitude),
                    "Positions et vitesses finies");
                Check(p.position.y >= p.Radius - 0.002f, "Pas de traversée du sol");
                if (Vector3.Distance(p.position, castle.InitialPositions[i]) > 0.75f) moved++;
                if (castle.InitialPositions[i].y > 2f)
                {
                    high++;
                    if (p.position.y < 1.25f) low++;
                }
            }
            Check(before == 0 && drift < 0.2f, "Château stable avant le tir");
            Check(moved > castle.ProjectileIndex * 0.65f && low > high * 0.65f,
                "Le tir doit faire tomber la majorité du château");
            return "Particules=" + castle.ProjectileIndex + " liens=" + castle.Solver.Links.Count
                + " dérive=" + drift + " ruptures avant=" + before
                + " après=" + castle.Solver.BrokenCount + " déplacées=" + moved
                + " haut tombées=" + low + "/" + high + " durée ms=" + watch.ElapsedMilliseconds;
        }

        [MenuItem("Haithem/Projet final/Vérifier la scène (Play Mode)")]
        public static void VerifierScene()
        {
            Check(EditorApplication.isPlaying, "Lancer la scène avant ce test");
            var demo = UnityEngine.Object.FindFirstObjectByType<ChateauParticules>();
            Check(demo != null, "Château présent");
            demo.Reinitialiser();
            int count = demo.NombreBoules;
            Check(count > 3500, "Château dense");
            Check(demo.GetComponentsInChildren<Collider>().Length == 0, "Aucun collider Unity");
            Check(demo.GetComponentsInChildren<Rigidbody>().Length == 0, "Aucun Rigidbody Unity");
            demo.ChangerPause();
            float time = demo.Simulation.World.Time;
            demo.Avancer(0.1f);
            Check(demo.Simulation.World.Time == time, "Pause respectée");
            demo.Tirer();
            demo.Tirer();
            Check(demo.Simulation.Fired, "Tir unique");
            demo.Reinitialiser();
            Check(!demo.Simulation.Fired && demo.Simulation.Solver.BrokenCount == 0,
                "Reset du projectile et des liaisons");
            Check(demo.NombreBoules == count && demo.transform.childCount == 0,
                "Reset sans doublons");
            Check(demo.EnPause, "Reset conserve la pause");
            demo.ChangerPause();
            demo.Avancer(0.1f);
            Check(demo.Simulation.World.Time > time, "Reprise");
            demo.Reinitialiser();
            UnityEngine.Debug.Log("Projet final : scène, pause, tir et reset OK");
        }

        static float Drift(ChateauSimulation castle)
        {
            float result = 0f;
            for (int i = 0; i < castle.ProjectileIndex; i++)
                result = Mathf.Max(result, Vector3.Distance(castle.Group.Particles[i].position,
                    castle.InitialPositions[i]));
            return result;
        }

        static void Check(bool condition, string message)
        {
            if (!condition) throw new Exception(message);
        }
    }
}
