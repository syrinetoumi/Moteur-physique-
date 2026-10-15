using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Outils du Chapitre 3 (lancer de rayons) : intersections et distances.
    /// Tout est statique et sans état : n'importe quel membre du groupe peut les appeler.
    /// </summary>
    public static class Intersections
    {
        const float Eps = 1e-6f;

        /// <summary>Intersection rayon-plan : t = -(D + n.S) / (n.V). Faux si n.V = 0 ou t &lt; 0.</summary>
        public static bool RayPlane(Ray3 ray, Plane3 plane, out float t)
        {
            t = 0f;
            float nV = Vector3.Dot(plane.n, ray.V);
            if (Mathf.Abs(nV) < Eps) return false;
            t = -(plane.D + Vector3.Dot(plane.n, ray.S)) / nV;
            return t >= 0f;
        }

        /// <summary>Coordonnées barycentriques (w0, w1, w2) de P par rapport au triangle (slides 34-35).</summary>
        public static bool Barycentric(Vector3 P, Triangle3 tri, out float w0, out float w1, out float w2)
        {
            Vector3 R = P - tri.P0;
            Vector3 Q1 = tri.P1 - tri.P0;
            Vector3 Q2 = tri.P2 - tri.P0;

            float q11 = Vector3.Dot(Q1, Q1);
            float q12 = Vector3.Dot(Q1, Q2);
            float q22 = Vector3.Dot(Q2, Q2);
            float rq1 = Vector3.Dot(R, Q1);
            float rq2 = Vector3.Dot(R, Q2);

            float det = q11 * q22 - q12 * q12;
            if (Mathf.Abs(det) < 1e-12f) { w0 = w1 = w2 = 0f; return false; }   // triangle dégénéré

            w1 = (q22 * rq1 - q12 * rq2) / det;
            w2 = (-q12 * rq1 + q11 * rq2) / det;
            w0 = 1f - w1 - w2;
            return true;
        }

        /// <summary>P (supposé dans le plan du triangle) est-il dans le triangle ? (tous les wi >= 0)</summary>
        public static bool PointInTriangle(Vector3 P, Triangle3 tri)
        {
            float w0, w1, w2;
            if (!Barycentric(P, tri, out w0, out w1, out w2)) return false;
            return w0 >= -Eps && w1 >= -Eps && w2 >= -Eps;
        }

        /// <summary>Rayon-triangle : intersection avec le plan, puis test barycentrique.</summary>
        public static bool RayTriangle(Ray3 ray, Triangle3 tri, out float t, out Vector3 point)
        {
            point = Vector3.zero;
            if (!RayPlane(ray, tri.Plane(), out t)) return false;
            point = ray.PointAt(t);
            return PointInTriangle(point, tri);
        }

        /// <summary>Rayon-sphère : résolution de |S + tV - C|^2 = r^2. Renvoie le premier t >= 0.</summary>
        public static bool RaySphere(Ray3 ray, Vector3 center, float radius, out float t)
        {
            t = 0f;
            Vector3 m = ray.S - center;
            float a = Vector3.Dot(ray.V, ray.V);
            float b = Vector3.Dot(m, ray.V);
            float c = Vector3.Dot(m, m) - radius * radius;
            float disc = b * b - a * c;
            if (a < Eps || disc < 0f) return false;
            float sq = Mathf.Sqrt(disc);
            t = (-b - sq) / a;
            if (t < 0f) t = (-b + sq) / a;
            return t >= 0f;
        }

        /// <summary>Distance du point P à la droite (AB) : |(P-A) ^ (B-A)| / |B-A|.</summary>
        public static float PointLineDistance(Vector3 P, Vector3 A, Vector3 B)
        {
            Vector3 u = B - A;
            float len = u.magnitude;
            if (len < Eps) return (P - A).magnitude;
            return Vector3.Cross(P - A, u).magnitude / len;
        }

        /// <summary>Distance minimale entre les droites (AB) et (CD) ; renvoie aussi t1 et t2 optimaux.</summary>
        public static float LineLineDistance(Vector3 A, Vector3 B, Vector3 C, Vector3 D,
                                             out float t1, out float t2)
        {
            Vector3 u = B - A, v = D - C, w = A - C;
            float a = Vector3.Dot(u, u), b = Vector3.Dot(u, v), c = Vector3.Dot(v, v);
            float d = Vector3.Dot(u, w), e = Vector3.Dot(v, w);
            float den = a * c - b * b;

            if (den < 1e-10f)            // droites parallèles
            {
                t1 = 0f;
                t2 = c > Eps ? e / c : 0f;
            }
            else
            {
                t1 = (b * e - c * d) / den;
                t2 = (a * e - b * d) / den;
            }
            Vector3 P = A + t1 * u;
            Vector3 Q = C + t2 * v;
            return (P - Q).magnitude;
        }
    }
}
