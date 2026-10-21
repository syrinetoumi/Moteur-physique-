using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Geometrie d'un corps rigide.
    /// </summary>
    public abstract class Shape
    {
        public abstract Mat3 InertiaBody(float mass);

        public abstract Vector3[] LocalVertices
        {
            get;
        }

        public abstract int[] Triangles
        {
            get;
        }

        public abstract Vector3 RenderScale
        {
            get;
        }
    }

    /// <summary>
    /// Parallelepipede centre a l'origine.
    /// </summary>
    public class BoxShape : Shape
    {
        public readonly Vector3 Size;

        readonly Vector3[] verts;
        readonly int[] tris;

        public BoxShape(Vector3 size)
        {
            Size = size;

            Vector3 half = size * 0.5f;

            verts = new Vector3[8];

            for (int i = 0; i < 8; i++)
            {
                verts[i] = new Vector3(
                    (i & 1) != 0
                        ? half.x
                        : -half.x,

                    (i & 2) != 0
                        ? half.y
                        : -half.y,

                    (i & 4) != 0
                        ? half.z
                        : -half.z
                );
            }

            int[][] quads =
            {
                new[] { 1, 3, 7, 5 },
                new[] { 0, 2, 6, 4 },

                new[] { 2, 3, 7, 6 },
                new[] { 0, 1, 5, 4 },

                new[] { 4, 5, 7, 6 },
                new[] { 0, 1, 3, 2 }
            };

            Vector3[] outward =
            {
                new Vector3(1f, 0f, 0f),
                new Vector3(-1f, 0f, 0f),

                new Vector3(0f, 1f, 0f),
                new Vector3(0f, -1f, 0f),

                new Vector3(0f, 0f, 1f),
                new Vector3(0f, 0f, -1f)
            };

            tris = new int[36];

            for (int face = 0; face < 6; face++)
            {
                int a = quads[face][0];
                int b = quads[face][1];
                int c = quads[face][2];
                int d = quads[face][3];

                Vector3 normal = Vec.Cross(
                    verts[b] - verts[a],
                    verts[c] - verts[a]
                );

                if (Vec.Dot(normal, outward[face]) < 0f)
                {
                    int temp = b;
                    b = d;
                    d = temp;
                }

                int t = face * 6;

                tris[t + 0] = a;
                tris[t + 1] = b;
                tris[t + 2] = c;

                tris[t + 3] = a;
                tris[t + 4] = c;
                tris[t + 5] = d;
            }
        }

        /// <summary>
        /// Tenseur d'inertie d'un parallelepipede.
        ///
        /// Ixx = m(b²+c²)/12
        /// Iyy = m(a²+c²)/12
        /// Izz = m(a²+b²)/12
        /// </summary>
        public override Mat3 InertiaBody(float mass)
        {
            float a = Size.x;
            float b = Size.y;
            float c = Size.z;

            return Mat3.Diag(
                mass * (b * b + c * c) / 12f,
                mass * (a * a + c * c) / 12f,
                mass * (a * a + b * b) / 12f
            );
        }

        public override Vector3[] LocalVertices
        {
            get
            {
                return verts;
            }
        }

        public override int[] Triangles
        {
            get
            {
                return tris;
            }
        }

        public override Vector3 RenderScale
        {
            get
            {
                return Size;
            }
        }
    }

    /// <summary>
    /// Sphere pleine.
    ///
    /// I = 2/5 * m * r²
    ///
    /// Attention :
    /// cette classe ne fournit pas encore
    /// un maillage de sphere.
    /// </summary>
    public class SphereShape : Shape
    {
        public readonly float Radius;

        static readonly Vector3[] noVertices =
            new Vector3[0];

        static readonly int[] noTriangles =
            new int[0];

        public SphereShape(float radius)
        {
            Radius = radius;
        }

        public override Mat3 InertiaBody(float mass)
        {
            float inertia =
                0.4f * mass * Radius * Radius;

            return Mat3.Diag(
                inertia,
                inertia,
                inertia
            );
        }

        public override Vector3[] LocalVertices
        {
            get
            {
                return noVertices;
            }
        }

        public override int[] Triangles
        {
            get
            {
                return noTriangles;
            }
        }

        public override Vector3 RenderScale
        {
            get
            {
                return Vector3.one * (2f * Radius);
            }
        }
    }
}