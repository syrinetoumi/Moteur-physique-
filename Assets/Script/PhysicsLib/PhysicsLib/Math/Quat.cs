using UnityEngine;

namespace PhysicsLib
{
    /// <summary>Quaternion maison : q = (w, x, y, z), w = partie réelle (Chapitre 1).</summary>
    public struct Quat
    {
        public float w, x, y, z;

        public Quat(float w, float x, float y, float z)
        { this.w = w; this.x = x; this.y = y; this.z = z; }

        public static Quat Identity => new Quat(1, 0, 0, 0);

        /// <summary>q = (cos(theta/2), v.sin(theta/2)), v unitaire.</summary>
        public static Quat FromAxisAngle(Vector3 axis, float angleRad)
        {
            float len = Mathf.Sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (len < 1e-8f) return Identity;
            float half = angleRad * 0.5f;
            float s = Mathf.Sin(half) / len;
            return new Quat(Mathf.Cos(half), axis.x * s, axis.y * s, axis.z * s);
        }

        /// <summary>Produit p * q.</summary>
        public static Quat operator *(Quat p, Quat q)
        {
            return new Quat(
                p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
                p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
                p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
                p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w);
        }

        public Quat Conjugate() => new Quat(w, -x, -y, -z);

        public Quat Normalized()
        {
            float n = Mathf.Sqrt(w * w + x * x + y * y + z * z);
            return new Quat(w / n, x / n, y / n, z / n);
        }

        /// <summary>P' = q . P . q^-1 avec P = (0, v).</summary>
        public Vector3 Rotate(Vector3 v)
        {
            Quat p = new Quat(0, v.x, v.y, v.z);
            Quat r = this * p * Conjugate();
            return new Vector3(r.x, r.y, r.z);
        }

        /// <summary>Matrice de rotation 3x3 équivalente (quaternion unitaire).</summary>
        public Mat3 ToMat3()
        {
            return new Mat3(
                new Vector3(1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
                new Vector3(2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
                new Vector3(2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)));
        }

        /// <summary>Produit vectoriel écrit à la main.</summary>
        public static Vector3 Cross(Vector3 u, Vector3 v)
        {
            return new Vector3(
                u.y * v.z - u.z * v.y,
                u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x);
        }

        public override string ToString() => "(" + w + ", " + x + ", " + y + ", " + z + ")";
    }
}
