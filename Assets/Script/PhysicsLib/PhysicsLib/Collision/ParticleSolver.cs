using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    public class ParticleSolver
    {
        public class Link
        {
            public int A, B;
            public float Length;
            public bool Broken;
        }

        struct Pair
        {
            public int A, B;
            public Vector3 Normal;
        }

        public readonly List<Link> Links = new List<Link>();
        public int Iterations = 12;
        public float BreakStrain = 0.12f;
        public float Restitution = 0.45f;
        public float Friction = 0.45f;
        public float GroundHeight;
        public int BrokenCount { get; private set; }
        public int PairChecks { get; private set; }

        readonly Dictionary<Vector3Int, int> cells = new Dictionary<Vector3Int, int>();
        readonly List<Pair> contacts = new List<Pair>();
        readonly List<Pair> candidates = new List<Pair>();
        Vector3[] previous = new Vector3[0], incoming = new Vector3[0];
        int[] next = new int[0];
        float cellSize;

        public void Connect(List<Particle> particles, int a, int b)
        {
            Links.Add(new Link { A = a, B = b,
                Length = Vector3.Distance(particles[a].position, particles[b].position) });
        }

        public void Prepare(List<Particle> particles)
        {
            if (previous.Length != particles.Count)
            {
                previous = new Vector3[particles.Count];
                incoming = new Vector3[particles.Count];
                next = new int[particles.Count];
            }
            cellSize = float.MaxValue;
            for (int i = 0; i < particles.Count; i++)
            {
                previous[i] = particles[i].position;
                cellSize = Mathf.Min(cellSize, Mathf.Max(0.001f, 2f * particles[i].Radius));
            }
        }

        public void Resolve(List<Particle> particles, float dt)
        {
            if (dt <= 0f) return;
            PairChecks = 0;
            for (int i = 0; i < particles.Count; i++) incoming[i] = particles[i].velocity;
            BuildPairs(particles);
            for (int pass = 0; pass < Mathf.Max(1, Iterations); pass++)
            {
                SolveLinks(particles);
                SolveContacts(particles);
                foreach (Particle p in particles)
                    if (!p.IsFixed && p.position.y < GroundHeight + p.Radius)
                        p.position.y = GroundHeight + p.Radius;
            }

            // La correction de position modifie aussi la vitesse.
            for (int i = 0; i < particles.Count; i++)
                particles[i].velocity = particles[i].IsFixed ? Vector3.zero
                    : (particles[i].position - previous[i]) / dt;

            foreach (Pair pair in contacts)
            {
                Particle a = particles[pair.A], b = particles[pair.B];
                float approach = Vector3.Dot(incoming[pair.A] - incoming[pair.B], pair.Normal);
                Impulse(a, b, pair.Normal, approach);
            }
            for (int i = 0; i < particles.Count; i++)
            {
                Particle p = particles[i];
                if (!p.IsFixed && p.position.y <= GroundHeight + p.Radius + 0.001f)
                    Impulse(p, null, Vector3.up, incoming[i].y);
            }
        }

        void SolveLinks(List<Particle> particles)
        {
            foreach (Link link in Links)
            {
                if (link.Broken) continue;
                Particle a = particles[link.A], b = particles[link.B];
                Vector3 delta = b.position - a.position;
                float length = delta.magnitude;
                if (length > link.Length * (1f + BreakStrain))
                {
                    link.Broken = true;
                    BrokenCount++;
                    continue;
                }
                float weight = a.InvMass + b.InvMass;
                if (length < 0.000001f || weight == 0f) continue;
                Vector3 correction = delta * ((length - link.Length) / (length * weight));
                a.position += correction * a.InvMass;
                b.position -= correction * b.InvMass;
            }
        }

        Vector3Int Cell(Vector3 p)
        {
            return new Vector3Int(Mathf.FloorToInt(p.x / cellSize),
                Mathf.FloorToInt(p.y / cellSize), Mathf.FloorToInt(p.z / cellSize));
        }

        void SolveContacts(List<Particle> particles)
        {
            contacts.Clear();
            foreach (Pair pair in candidates) SolvePair(particles, pair.A, pair.B);
        }

        void BuildPairs(List<Particle> particles)
        {
            cells.Clear();
            candidates.Clear();
            for (int i = 0; i < particles.Count; i++)
            {
                if (particles[i].Radius * 2f > cellSize + 0.001f) continue;
                Vector3Int cell = Cell(particles[i].position);
                int head;
                next[i] = cells.TryGetValue(cell, out head) ? head : -1;
                cells[cell] = i;
            }
            // Les grosses boules sont testées à part pour garder une grille fine.
            for (int i = 0; i < particles.Count; i++)
            {
                Particle a = particles[i];
                if (a.Radius * 2f > cellSize + 0.001f)
                {
                    for (int j = 0; j < particles.Count; j++)
                        if (j != i && (particles[j].Radius * 2f <= cellSize + 0.001f || j > i))
                            candidates.Add(new Pair { A = i, B = j });
                    continue;
                }
                Vector3Int cell = Cell(a.position);
                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    int j;
                    if (!cells.TryGetValue(cell + new Vector3Int(x, y, z), out j)) continue;
                    for (; j >= 0; j = next[j])
                    {
                        if (j <= i) continue;
                        candidates.Add(new Pair { A = i, B = j });
                    }
                }
            }
        }

        void SolvePair(List<Particle> particles, int i, int j)
        {
            Particle a = particles[i], b = particles[j];
            float weight = a.InvMass + b.InvMass;
            if (weight == 0f) return;
            PairChecks++;
            Vector3 delta = a.position - b.position;
            float radius = a.Radius + b.Radius;
            if (delta.sqrMagnitude > (radius + 0.001f) * (radius + 0.001f)) return;
            float length = delta.magnitude;
            Vector3 normal = length > 0.000001f ? delta / length : Vector3.right;
            contacts.Add(new Pair { A = i, B = j, Normal = normal });
            Vector3 correction = normal * (Mathf.Max(0f, radius - length) / weight);
            a.position += correction * a.InvMass;
            b.position -= correction * b.InvMass;
        }

        void Impulse(Particle a, Particle b, Vector3 normal, float approach)
        {
            float invB = b == null ? 0f : b.InvMass;
            float weight = a.InvMass + invB;
            if (weight == 0f) return;
            Vector3 relative = a.velocity - (b == null ? Vector3.zero : b.velocity);
            // Les petits chocs restent au repos.
            float target = approach < -1f ? -approach * Restitution : 0f;
            float change = Mathf.Max(0f, target - Vector3.Dot(relative, normal));
            Vector3 impulse = normal * (change / weight);
            Vector3 tangent = relative - normal * Vector3.Dot(relative, normal);
            float speed = tangent.magnitude;
            if (speed > 0.000001f)
                impulse -= tangent / speed * (Mathf.Min(speed,
                    Friction * Mathf.Max(change, -approach)) / weight);
            a.velocity += impulse * a.InvMass;
            if (b != null) b.velocity -= impulse * invB;
        }
    }
}
