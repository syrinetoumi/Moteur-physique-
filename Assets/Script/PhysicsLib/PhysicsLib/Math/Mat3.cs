using System;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Matrice 3x3 stockee par colonnes.
    /// </summary>
    public struct Mat3
    {
        public Vector3 c0;
        public Vector3 c1;
        public Vector3 c2;

        public Mat3(
            Vector3 column0,
            Vector3 column1,
            Vector3 column2)
        {
            c0 = column0;
            c1 = column1;
            c2 = column2;
        }

        public static Mat3 Identity
        {
            get
            {
                return new Mat3(
                    new Vector3(1f, 0f, 0f),
                    new Vector3(0f, 1f, 0f),
                    new Vector3(0f, 0f, 1f)
                );
            }
        }

        public static Mat3 Zero
        {
            get
            {
                return new Mat3(
                    Vector3.zero,
                    Vector3.zero,
                    Vector3.zero
                );
            }
        }

        public static Mat3 Diag(
            float a,
            float b,
            float c)
        {
            return new Mat3(
                new Vector3(a, 0f, 0f),
                new Vector3(0f, b, 0f),
                new Vector3(0f, 0f, c)
            );
        }

        /// <summary>
        /// Matrice antisymetrique :
        /// Star(w) * v = w x v.
        /// </summary>
        public static Mat3 Star(Vector3 w)
        {
            return new Mat3(
                new Vector3(0f, w.z, -w.y),
                new Vector3(-w.z, 0f, w.x),
                new Vector3(w.y, -w.x, 0f)
            );
        }

        public Vector3 Mul(Vector3 v)
        {
            return c0 * v.x
                 + c1 * v.y
                 + c2 * v.z;
        }

        public Mat3 Transpose()
        {
            return new Mat3(
                new Vector3(c0.x, c1.x, c2.x),
                new Vector3(c0.y, c1.y, c2.y),
                new Vector3(c0.z, c1.z, c2.z)
            );
        }

        public float Determinant()
        {
            return Vec.Dot(
                c0,
                Vec.Cross(c1, c2)
            );
        }

        /// <summary>
        /// Essaie de calculer l'inverse.
        /// Retourne false si la matrice est singuliere.
        /// </summary>
        public bool TryInverse(out Mat3 inverse)
        {
            float det = Determinant();

            if (Mathf.Abs(det) < 1e-12f)
            {
                inverse = Zero;
                return false;
            }

            Vector3 r0 =
                Vec.Cross(c1, c2) / det;

            Vector3 r1 =
                Vec.Cross(c2, c0) / det;

            Vector3 r2 =
                Vec.Cross(c0, c1) / det;

            inverse = new Mat3(
                new Vector3(r0.x, r1.x, r2.x),
                new Vector3(r0.y, r1.y, r2.y),
                new Vector3(r0.z, r1.z, r2.z)
            );

            return true;
        }

        /// <summary>
        /// Inverse generale.
        /// Une matrice singuliere provoque une exception.
        /// </summary>
        public Mat3 Inverse()
        {
            Mat3 inverse;

            if (!TryInverse(out inverse))
            {
                throw new InvalidOperationException(
                    "Mat3 : matrice singuliere, inverse impossible."
                );
            }

            return inverse;
        }

        /// <summary>
        /// Orthonormalisation Gram-Schmidt.
        /// Chapitre 2 Partie 2, page 36.
        /// </summary>
        public Mat3 Orthonormalize()
        {
            Vector3 T = Vec.Normalize(c0);

            if (Vec.LengthSquared(T) < Vec.Epsilon)
                return Identity;

            Vector3 N =
                c1 - T * Vec.Dot(c1, T);

            N = Vec.Normalize(N);

            if (Vec.LengthSquared(N) < Vec.Epsilon)
                return Identity;

            Vector3 B = Vec.Cross(T, N);

            return new Mat3(T, N, B);
        }

        public static Mat3 operator *(
            Mat3 A,
            Mat3 B)
        {
            return new Mat3(
                A.Mul(B.c0),
                A.Mul(B.c1),
                A.Mul(B.c2)
            );
        }

        public static Vector3 operator *(
            Mat3 A,
            Vector3 v)
        {
            return A.Mul(v);
        }

        public static Mat3 operator *(
            Mat3 A,
            float s)
        {
            return new Mat3(
                A.c0 * s,
                A.c1 * s,
                A.c2 * s
            );
        }

        public static Mat3 operator +(
            Mat3 A,
            Mat3 B)
        {
            return new Mat3(
                A.c0 + B.c0,
                A.c1 + B.c1,
                A.c2 + B.c2
            );
        }

        /// <summary>
        /// Construit manuellement une matrice homogene 4x4.
        ///
        /// Aucune fonction TRS de Unity.
        /// Matrix4x4 sert uniquement au stockage.
        /// </summary>
        public Matrix4x4 ToMatrix4x4(
            Vector3 translation,
            Vector3 scale)
        {
            Matrix4x4 m = new Matrix4x4();

            m.m00 = c0.x * scale.x;
            m.m10 = c0.y * scale.x;
            m.m20 = c0.z * scale.x;
            m.m30 = 0f;

            m.m01 = c1.x * scale.y;
            m.m11 = c1.y * scale.y;
            m.m21 = c1.z * scale.y;
            m.m31 = 0f;

            m.m02 = c2.x * scale.z;
            m.m12 = c2.y * scale.z;
            m.m22 = c2.z * scale.z;
            m.m32 = 0f;

            m.m03 = translation.x;
            m.m13 = translation.y;
            m.m23 = translation.z;
            m.m33 = 1f;

            return m;
        }

        public override string ToString()
        {
            return "[" + c0 + " | "
                       + c1 + " | "
                       + c2 + "]";
        }
    }
}