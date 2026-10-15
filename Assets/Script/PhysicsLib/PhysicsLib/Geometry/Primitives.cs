using UnityEngine;

namespace PhysicsLib
{
    /// <summary>Demi-droite (rayon) : P(t) = S + t.V, t >= 0 (Chapitre 3).</summary>
    public struct Ray3
    {
        public Vector3 S;   // point de départ
        public Vector3 V;   // direction

        public Ray3(Vector3 start, Vector3 direction) { S = start; V = direction; }

        public Vector3 PointAt(float t) => S + t * V;
    }

    /// <summary>Plan : n.P + D = 0, avec n unitaire et D = -n.P0 (slide 23).</summary>
    public struct Plane3
    {
        public Vector3 n;
        public float D;

        public Plane3(Vector3 normal, float d) { n = normal.normalized; D = d; }

        public static Plane3 FromPointNormal(Vector3 point, Vector3 normal)
        {
            Vector3 nn = normal.normalized;
            return new Plane3(nn, -Vector3.Dot(nn, point));
        }

        public static Plane3 FromPoints(Vector3 P0, Vector3 P1, Vector3 P2)
        {
            Vector3 nn = Vector3.Cross(P1 - P0, P2 - P0).normalized;
            return new Plane3(nn, -Vector3.Dot(nn, P0));
        }

        /// <summary>Distance signée d'un point au plan (positive du côté de la normale).</summary>
        public float SignedDistance(Vector3 P) => Vector3.Dot(n, P) + D;
    }

    /// <summary>Triangle (P0, P1, P2). Normale = (P1-P0) ^ (P2-P0).</summary>
    public struct Triangle3
    {
        public Vector3 P0, P1, P2;

        public Triangle3(Vector3 p0, Vector3 p1, Vector3 p2) { P0 = p0; P1 = p1; P2 = p2; }

        public Vector3 Normal() => Vector3.Cross(P1 - P0, P2 - P0).normalized;

        public Plane3 Plane() => Plane3.FromPoints(P0, P1, P2);

        public Vector3 Centroid() => (P0 + P1 + P2) / 3f;
    }
}
