using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Système de N particules (Chapitre 2 partie 1) : tissu, cordes, fumée, sable...
    /// Forces : liste d'IForce (une par particule) + liste d'IPairForce (ressorts).
    /// </summary>
    public class ParticleGroup
    {
        public List<Particle> Particles = new List<Particle>();
        public List<IForce> Forces = new List<IForce>();
        public List<IPairForce> PairForces = new List<IPairForce>();
        public IIntegrator Integrator = new RK4Integrator();

        Vector3[] x = new Vector3[0], v = new Vector3[0], f = new Vector3[0];
        AccelerationFunc accelFunc;

        public ParticleGroup() { accelFunc = Accelerations; }

        public int Add(Particle p) { Particles.Add(p); return Particles.Count - 1; }

        public void Step(float dt)
        {
            int n = Particles.Count;
            if (n == 0) return;
            if (x.Length != n) { x = new Vector3[n]; v = new Vector3[n]; f = new Vector3[n]; }

            for (int i = 0; i < n; i++) { x[i] = Particles[i].position; v[i] = Particles[i].velocity; }

            Integrator.Step(x, v, dt, accelFunc);

            for (int i = 0; i < n; i++) { Particles[i].position = x[i]; Particles[i].velocity = v[i]; }
        }

        void Accelerations(Vector3[] xs, Vector3[] vs, Vector3[] aOut)
        {
            int n = xs.Length;
            for (int i = 0; i < n; i++)
            {
                Vector3 total = Vector3.zero;
                Particle p = Particles[i];
                if (!p.IsFixed)
                    for (int k = 0; k < Forces.Count; k++)
                        total += Forces[k].Compute(xs[i], vs[i], p.Mass);
                f[i] = total;
            }

            for (int k = 0; k < PairForces.Count; k++) PairForces[k].Accumulate(xs, vs, f);

            for (int i = 0; i < n; i++)
                aOut[i] = Particles[i].IsFixed ? Vector3.zero : f[i] * Particles[i].InvMass;
        }
    }
}
