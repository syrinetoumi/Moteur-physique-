using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>Un point de contact détecté entre un corps A et un corps B (B = null : plan fixe).</summary>
    public struct Contact
    {
        public RigidBody A;
        public RigidBody B;
        public Vector3 point;        // point de contact (repère monde)
        public Vector3 normal;       // normale du contact (dirigée vers A)
        public float penetration;    // profondeur de pénétration (>= 0)
    }

    /// <summary>Détection : remplit la liste de contacts. Une classe par type de détection.</summary>
    public interface ICollisionDetector
    {
        void Detect(List<RigidBody> bodies, List<Contact> contacts);
    }

    /// <summary>Résolution : corrige les corps à partir des contacts (à compléter avec le cours sur les collisions).</summary>
    public interface ICollisionResolver
    {
        void Resolve(List<Contact> contacts, float dt);
    }
}
