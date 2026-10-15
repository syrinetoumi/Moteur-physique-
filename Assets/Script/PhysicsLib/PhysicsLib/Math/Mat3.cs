using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Matrice 3x3 stockée par colonnes (c0, c1, c2).
    /// Utilisée pour l'orientation R(t) et le tenseur d'inertie I(t) (Chapitre 2, partie 2).
    /// </summary>
    public struct Mat3
    {
        public Vector3 c0, c1, c2;

        public Mat3(Vector3 c0, Vector3 c1, Vector3 c2)
        { this.c0 = c0; this.c1 = c1; this.c2 = c2; }

        public static Mat3 Identity =>
            new Mat3(new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1));

        public static Mat3 Zero => new Mat3(Vector3.zero, Vector3.zero, Vector3.zero);

        public static Mat3 Diag(float a, float b, float c) =>
            new Mat3(new Vector3(a, 0, 0), new Vector3(0, b, 0), new Vector3(0, 0, c));

        /// <summary>Matrice antisymétrique w* telle que (w*).v = w ^ v.</summary>
        public static Mat3 Star(Vector3 w) =>
            new Mat3(new Vector3(0, w.z, -w.y),
                     new Vector3(-w.z, 0, w.x),
                     new Vector3(w.y, -w.x, 0));

        public Vector3 Mul(Vector3 v) => c0 * v.x + c1 * v.y + c2 * v.z;

        public Mat3 Transpose() =>
            new Mat3(new Vector3(c0.x, c1.x, c2.x),
                     new Vector3(c0.y, c1.y, c2.y),
                     new Vector3(c0.z, c1.z, c2.z));

        public float Determinant() => Vector3.Dot(c0, Vector3.Cross(c1, c2));

        /// <summary>Inverse générale (via produits vectoriels). Pour une matrice orthonormale, préférer Transpose().</summary>
        public Mat3 Inverse()
        {
            float det = Determinant();
            if (Mathf.Abs(det) < 1e-12f) return Identity;   // matrice singulière : on évite NaN
            Vector3 r0 = Vector3.Cross(c1, c2) / det;
            Vector3 r1 = Vector3.Cross(c2, c0) / det;
            Vector3 r2 = Vector3.Cross(c0, c1) / det;
            // r0, r1, r2 sont les LIGNES de l'inverse -> on les range en colonnes
            return new Mat3(new Vector3(r0.x, r1.x, r2.x),
                            new Vector3(r0.y, r1.y, r2.y),
                            new Vector3(r0.z, r1.z, r2.z));
        }

        /// <summary>Gram-Schmidt sur les colonnes T, N, B (slide 36 du cours).</summary>
        public Mat3 Orthonormalize()
        {
            Vector3 T = c0.normalized;
            Vector3 N = c1 - T * Vector3.Dot(c1, T);
            N = N.normalized;
            Vector3 B = Vector3.Cross(T, N);
            return new Mat3(T, N, B);
        }

        public static Mat3 operator *(Mat3 A, Mat3 B) =>
            new Mat3(A.Mul(B.c0), A.Mul(B.c1), A.Mul(B.c2));

        public static Vector3 operator *(Mat3 A, Vector3 v) => A.Mul(v);

        public static Mat3 operator *(Mat3 A, float s) =>
            new Mat3(A.c0 * s, A.c1 * s, A.c2 * s);

        public static Mat3 operator +(Mat3 A, Mat3 B) =>
            new Mat3(A.c0 + B.c0, A.c1 + B.c1, A.c2 + B.c2);

        /// <summary>
        /// Matrice homogène 4x4 [ R.S | t ; 0 0 0 1 ] (S = échelle par axe).
        /// Prête à être passée à CustomCube.ApplyTransformation.
        /// </summary>
        public Matrix4x4 ToMatrix4x4(Vector3 translation, Vector3 scale)
        {
            Matrix4x4 m = Matrix4x4.identity;
            m.m00 = c0.x * scale.x; m.m10 = c0.y * scale.x; m.m20 = c0.z * scale.x;
            m.m01 = c1.x * scale.y; m.m11 = c1.y * scale.y; m.m21 = c1.z * scale.y;
            m.m02 = c2.x * scale.z; m.m12 = c2.y * scale.z; m.m22 = c2.z * scale.z;
            m.m03 = translation.x;  m.m13 = translation.y;  m.m23 = translation.z;
            return m;
        }

        public override string ToString() => "[" + c0 + " | " + c1 + " | " + c2 + "]";
    }
}
