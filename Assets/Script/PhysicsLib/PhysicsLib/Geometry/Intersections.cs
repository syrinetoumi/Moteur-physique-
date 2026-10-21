using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Intersections geometriques implementees manuellement.
    /// Chapitre 3 : lancer de rayons.
    /// </summary>
    public static class Intersections
    {
        const float Eps = 1e-6f;

        /// <summary>
        /// Intersection rayon-plan.
        /// t = -(D + n.S) / (n.V)
        /// </summary>
        public static bool RayPlane(
            Ray3 ray,
            Plane3 plane,
            out float t)
        {
            t = 0f;

            float nV = Vec.Dot(
                plane.n,
                ray.V
            );

            if (Mathf.Abs(nV) < Eps)
                return false;

            t = -(
                plane.D +
                Vec.Dot(plane.n, ray.S)
            ) / nV;

            return t >= 0f;
        }

        /// <summary>
        /// Coordonnees barycentriques.
        /// </summary>
        public static bool Barycentric(
            Vector3 P,
            Triangle3 tri,
            out float w0,
            out float w1,
            out float w2)
        {
            Vector3 R = P - tri.P0;

            Vector3 Q1 = tri.P1 - tri.P0;
            Vector3 Q2 = tri.P2 - tri.P0;

            float q11 = Vec.Dot(Q1, Q1);
            float q12 = Vec.Dot(Q1, Q2);
            float q22 = Vec.Dot(Q2, Q2);

            float rq1 = Vec.Dot(R, Q1);
            float rq2 = Vec.Dot(R, Q2);

            float det =
                q11 * q22 - q12 * q12;

            if (Mathf.Abs(det) < 1e-12f)
            {
                w0 = 0f;
                w1 = 0f;
                w2 = 0f;

                return false;
            }

            w1 =
                (q22 * rq1 - q12 * rq2) / det;

            w2 =
                (-q12 * rq1 + q11 * rq2) / det;

            w0 = 1f - w1 - w2;

            return true;
        }

        /// <summary>
        /// Test d'appartenance a un triangle.
        /// Le point doit etre dans le plan du triangle.
        /// </summary>
        public static bool PointInTriangle(
            Vector3 P,
            Triangle3 tri)
        {
            float w0;
            float w1;
            float w2;

            if (!Barycentric(
                P,
                tri,
                out w0,
                out w1,
                out w2))
            {
                return false;
            }

            return w0 >= -Eps
                && w1 >= -Eps
                && w2 >= -Eps;
        }

        /// <summary>
        /// Intersection rayon-triangle.
        /// </summary>
        public static bool RayTriangle(
            Ray3 ray,
            Triangle3 tri,
            out float t,
            out Vector3 point)
        {
            point = Vector3.zero;

            if (!RayPlane(
                ray,
                tri.Plane(),
                out t))
            {
                return false;
            }

            point = ray.PointAt(t);

            return PointInTriangle(point, tri);
        }

        /// <summary>
        /// Intersection rayon-sphere.
        /// </summary>
        public static bool RaySphere(
            Ray3 ray,
            Vector3 center,
            float radius,
            out float t)
        {
            t = 0f;

            Vector3 m = ray.S - center;

            float a = Vec.Dot(ray.V, ray.V);
            float b = Vec.Dot(m, ray.V);

            float c =
                Vec.Dot(m, m) -
                radius * radius;

            float discriminant =
                b * b - a * c;

            if (a < Eps || discriminant < 0f)
                return false;

            float root =
                Mathf.Sqrt(discriminant);

            t = (-b - root) / a;

            if (t < 0f)
                t = (-b + root) / a;

            return t >= 0f;
        }

        /// <summary>
        /// Distance entre un point et une droite.
        /// </summary>
        public static float PointLineDistance(
            Vector3 P,
            Vector3 A,
            Vector3 B)
        {
            Vector3 u = B - A;

            float length = Vec.Length(u);

            if (length < Eps)
                return Vec.Distance(P, A);

            Vector3 cross =
                Vec.Cross(P - A, u);

            return Vec.Length(cross) / length;
        }

        /// <summary>
        /// Distance minimale entre deux droites.
        /// </summary>
        public static float LineLineDistance(
            Vector3 A,
            Vector3 B,
            Vector3 C,
            Vector3 D,
            out float t1,
            out float t2)
        {
            Vector3 u = B - A;
            Vector3 v = D - C;
            Vector3 w = A - C;

            float a = Vec.Dot(u, u);
            float b = Vec.Dot(u, v);
            float c = Vec.Dot(v, v);

            float d = Vec.Dot(u, w);
            float e = Vec.Dot(v, w);

            float denominator =
                a * c - b * b;

            if (denominator < 1e-10f)
            {
                t1 = 0f;

                t2 = c > Eps
                    ? e / c
                    : 0f;
            }
            else
            {
                t1 =
                    (b * e - c * d) /
                    denominator;

                t2 =
                    (a * e - b * d) /
                    denominator;
            }

            Vector3 P = A + t1 * u;
            Vector3 Q = C + t2 * v;

            return Vec.Distance(P, Q);
        }
    }
}