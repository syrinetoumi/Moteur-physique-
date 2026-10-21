using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Rayon : P(t) = S + tV, t >= 0.
    /// </summary>
    public struct Ray3
    {
        public Vector3 S;
        public Vector3 V;

        public Ray3(
            Vector3 start,
            Vector3 direction)
        {
            S = start;
            V = direction;
        }

        public Vector3 PointAt(float t)
        {
            return S + t * V;
        }
    }

    /// <summary>
    /// Plan : n.P + D = 0.
    /// La normale est unitaire.
    /// </summary>
    public struct Plane3
    {
        public Vector3 n;
        public float D;

        public Plane3(Vector3 normal, float d)
        {
            float length = Vec.Length(normal);

            if (length < Vec.Epsilon)
            {
                n = Vector3.up;
                D = 0f;
                return;
            }

            n = normal / length;

            // Si la normale change d'echelle,
            // D doit changer de la meme maniere.
            D = d / length;
        }

        public static Plane3 FromPointNormal(
            Vector3 point,
            Vector3 normal)
        {
            Vector3 nn = Vec.Normalize(normal);

            return new Plane3(
                nn,
                -Vec.Dot(nn, point)
            );
        }

        public static Plane3 FromPoints(
            Vector3 P0,
            Vector3 P1,
            Vector3 P2)
        {
            Vector3 edge1 = P1 - P0;
            Vector3 edge2 = P2 - P0;

            Vector3 nn = Vec.Normalize(
                Vec.Cross(edge1, edge2)
            );

            return new Plane3(
                nn,
                -Vec.Dot(nn, P0)
            );
        }

        public float SignedDistance(Vector3 point)
        {
            return Vec.Dot(n, point) + D;
        }
    }

    /// <summary>
    /// Triangle defini par trois sommets.
    /// </summary>
    public struct Triangle3
    {
        public Vector3 P0;
        public Vector3 P1;
        public Vector3 P2;

        public Triangle3(
            Vector3 p0,
            Vector3 p1,
            Vector3 p2)
        {
            P0 = p0;
            P1 = p1;
            P2 = p2;
        }

        public Vector3 Normal()
        {
            return Vec.Normalize(
                Vec.Cross(
                    P1 - P0,
                    P2 - P0
                )
            );
        }

        public Plane3 Plane()
        {
            return Plane3.FromPoints(
                P0,
                P1,
                P2
            );
        }

        public Vector3 Centroid()
        {
            return (P0 + P1 + P2) / 3f;
        }
    }
}