using System.Collections.Generic;

namespace PhysicsLib
{
    /// <summary>
    /// Regroupe plusieurs détecteurs en un seul (ex : sol + brique/brique).
    /// PhysicsWorld n'accepte qu'un détecteur : on lui donne celui-ci.
    /// </summary>
    public class CompositeDetector : ICollisionDetector
    {
        public List<ICollisionDetector> Detectors = new List<ICollisionDetector>();

        public CompositeDetector(params ICollisionDetector[] detectors) { Detectors.AddRange(detectors); }

        public void Detect(List<RigidBody> bodies, List<Contact> contacts)
        {
            for (int i = 0; i < Detectors.Count; i++) Detectors[i].Detect(bodies, contacts);
        }
    }
}
