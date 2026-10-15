using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Le "chef d'orchestre" du moteur : contient les corps et groupes de particules et les fait avancer
    /// avec un pas de temps FIXE (dt = 0.002 comme le cours), indépendamment du framerate.
    /// </summary>
    public class PhysicsWorld
    {
        public float FixedDt = 0.002f;
        public int MaxStepsPerUpdate = 250;                // protège contre la "spirale de la mort"

        public List<RigidBody> Bodies = new List<RigidBody>();
        public List<ParticleGroup> ParticleGroups = new List<ParticleGroup>();

        public ICollisionDetector Detector;
        public ICollisionResolver Resolver;
        public List<Contact> Contacts = new List<Contact>();   // contacts du dernier pas (utile pour debug)

        public float Time { get; private set; }

        readonly List<IForce> globalForces = new List<IForce>();
        float accumulator;

        /// <summary>Ajoute une force (ex : gravité) à tous les corps et groupes, présents et futurs.</summary>
        public void AddGlobalForce(IForce force)
        {
            globalForces.Add(force);
            foreach (RigidBody b in Bodies) b.Forces.Add(force);
            foreach (ParticleGroup g in ParticleGroups) g.Forces.Add(force);
        }

        public RigidBody AddBody(RigidBody body)
        {
            Bodies.Add(body);
            foreach (IForce f in globalForces) body.Forces.Add(f);
            return body;
        }

        public ParticleGroup AddParticleGroup(ParticleGroup group)
        {
            ParticleGroups.Add(group);
            foreach (IForce f in globalForces) group.Forces.Add(f);
            return group;
        }

        /// <summary>À appeler à chaque frame avec le temps écoulé (Time.deltaTime).</summary>
        public void Update(float frameDt)
        {
            accumulator += frameDt;
            int steps = 0;
            while (accumulator >= FixedDt && steps < MaxStepsPerUpdate)
            {
                Step(FixedDt);
                accumulator -= FixedDt;
                steps++;
            }
            if (steps == MaxStepsPerUpdate) accumulator = 0f;
        }

        /// <summary>Un pas de simulation : intégration, détection, résolution.</summary>
        public void Step(float dt)
        {
            for (int i = 0; i < Bodies.Count; i++) Bodies[i].Step(dt);
            for (int i = 0; i < ParticleGroups.Count; i++) ParticleGroups[i].Step(dt);

            Contacts.Clear();
            if (Detector != null)
            {
                Detector.Detect(Bodies, Contacts);
                if (Resolver != null && Contacts.Count > 0) Resolver.Resolve(Contacts, dt);
            }
            Time += dt;
        }
    }
}
