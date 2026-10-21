using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Systeme de particules.
    ///
    /// Chapitre 2 Partie 1 :
    /// systemes ponctuels et forces.
    ///
    /// Les particules fixes ne sont pas integrees.
    /// </summary>
    public class ParticleGroup
    {
        public List<Particle> Particles =
            new List<Particle>();

        public List<IForce> Forces =
            new List<IForce>();

        public List<IPairForce> PairForces =
            new List<IPairForce>();

        public IIntegrator Integrator =
            new RK4Integrator();

        Vector3[] x = new Vector3[0];
        Vector3[] v = new Vector3[0];
        Vector3[] f = new Vector3[0];

        AccelerationFunc accelFunc;

        public ParticleGroup()
        {
            accelFunc = Accelerations;
        }

        public int Add(Particle particle)
        {
            Particles.Add(particle);

            return Particles.Count - 1;
        }

        public void Step(float dt)
        {
            int n = Particles.Count;

            if (n == 0 || dt <= 0f)
                return;

            if (x.Length != n)
            {
                x = new Vector3[n];
                v = new Vector3[n];
                f = new Vector3[n];
            }

            for (int i = 0; i < n; i++)
            {
                Particle particle = Particles[i];

                x[i] = particle.position;

                v[i] = particle.IsFixed
                    ? Vector3.zero
                    : particle.velocity;
            }

            Integrator.Step(
                x,
                v,
                dt,
                accelFunc
            );

            for (int i = 0; i < n; i++)
            {
                Particle particle = Particles[i];

                if (particle.IsFixed)
                {
                    particle.velocity = Vector3.zero;
                    continue;
                }

                particle.position = x[i];
                particle.velocity = v[i];
            }
        }

        void Accelerations(
            Vector3[] xs,
            Vector3[] vs,
            Vector3[] aOut)
        {
            int n = xs.Length;

            for (int i = 0; i < n; i++)
            {
                Vector3 total = Vector3.zero;

                Particle particle = Particles[i];

                if (!particle.IsFixed)
                {
                    for (int k = 0; k < Forces.Count; k++)
                    {
                        total += Forces[k].Compute(
                            xs[i],
                            vs[i],
                            particle.Mass
                        );
                    }
                }

                f[i] = total;
            }

            for (int k = 0; k < PairForces.Count; k++)
            {
                PairForces[k].Accumulate(
                    xs,
                    vs,
                    f
                );
            }

            for (int i = 0; i < n; i++)
            {
                Particle particle = Particles[i];

                if (particle.IsFixed)
                {
                    aOut[i] = Vector3.zero;
                }
                else
                {
                    aOut[i] =
                        f[i] * particle.InvMass;
                }
            }
        }
    }
}