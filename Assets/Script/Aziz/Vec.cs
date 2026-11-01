using UnityEngine;

namespace PhysicsLib
{
    public static class Vec
    {
        public const float Epsilon = 0.000001f;

        public static float Dot(Vector3 a, Vector3 b)
        {
            return a.x * b.x
                 + a.y * b.y
                 + a.z * b.z;
        }

        public static Vector3 Cross(Vector3 a, Vector3 b)
        {
            return new Vector3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }

        public static float LengthSquared(Vector3 v)
        {
            return Dot(v, v);
        }

        public static float Length(Vector3 v)
        {
            return Mathf.Sqrt(Dot(v, v));
        }

        public static Vector3 Normalize(Vector3 v)
        {
            float length = Length(v);

            if (length < Epsilon)
                return Vector3.zero;

            return v / length;
        }

        public static float Distance(Vector3 a, Vector3 b)
        {
            return Length(b - a);
        }

        public static Vector3 Project(Vector3 a, Vector3 b)
        {
            float denominator = Dot(b, b);

            if (denominator < Epsilon)
                return Vector3.zero;

            return b * (Dot(a, b) / denominator);
        }

        public static Vector3 Reject(Vector3 a, Vector3 b)
        {
            return a - Project(a, b);
        }
    }
}