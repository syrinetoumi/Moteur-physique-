using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Géométrie d'un corps rigide dans son repère local (centre de masse à l'origine) :
    /// sommets, triangles, tenseur d'inertie I_body, échelle pour l'affichage.
    /// Pour ajouter une forme : hériter de Shape.
    /// </summary>
    public abstract class Shape
    {
        /// <summary>Tenseur d'inertie I_body (matrice diagonale pour les formes simples).</summary>
        public abstract Mat3 InertiaBody(float mass);

        /// <summary>Sommets en repère local.</summary>
        public abstract Vector3[] LocalVertices { get; }

        /// <summary>Triangles (indices dans LocalVertices, orientés vers l'extérieur).</summary>
        public abstract int[] Triangles { get; }

        /// <summary>Échelle à appliquer au cube unitaire de CustomCube pour afficher la forme.</summary>
        public abstract Vector3 RenderScale { get; }
    }

    /// <summary>Parallélépipède a x b x c centré à l'origine.</summary>
    public class BoxShape : Shape
    {
        public readonly Vector3 Size;
        readonly Vector3[] verts;
        readonly int[] tris;

        public BoxShape(Vector3 size)
        {
            Size = size;
            Vector3 h = size * 0.5f;

            // sommet i : bit0 -> x, bit1 -> y, bit2 -> z
            verts = new Vector3[8];
            for (int i = 0; i < 8; i++)
                verts[i] = new Vector3(((i & 1) != 0) ? h.x : -h.x,
                                       ((i & 2) != 0) ? h.y : -h.y,
                                       ((i & 4) != 0) ? h.z : -h.z);

            // 6 faces : (a, b, c, d) en cycle + normale sortante
            int[][] quads =
            {
                new[] { 1, 3, 7, 5 }, new[] { 0, 2, 6, 4 },   // +X, -X
                new[] { 2, 3, 7, 6 }, new[] { 0, 1, 5, 4 },   // +Y, -Y
                new[] { 4, 5, 7, 6 }, new[] { 0, 1, 3, 2 }    // +Z, -Z
            };
            Vector3[] outward =
            {
                new Vector3(1, 0, 0), new Vector3(-1, 0, 0),
                new Vector3(0, 1, 0), new Vector3(0, -1, 0),
                new Vector3(0, 0, 1), new Vector3(0, 0, -1)
            };

            tris = new int[36];
            for (int f = 0; f < 6; f++)
            {
                int a = quads[f][0], b = quads[f][1], c = quads[f][2], d = quads[f][3];
                Vector3 nrm = Vector3.Cross(verts[b] - verts[a], verts[c] - verts[a]);
                if (Vector3.Dot(nrm, outward[f]) < 0f) { int tmp = b; b = d; d = tmp; }   // inverse l'orientation
                int t = f * 6;
                tris[t + 0] = a; tris[t + 1] = b; tris[t + 2] = c;
                tris[t + 3] = a; tris[t + 4] = c; tris[t + 5] = d;
            }
        }

        /// <summary>I_body = m/12 . diag(b²+c², a²+c², a²+b²)   (slide 33).</summary>
        public override Mat3 InertiaBody(float mass)
        {
            float a = Size.x, b = Size.y, c = Size.z;
            return Mat3.Diag(mass / 12f * (b * b + c * c),
                             mass / 12f * (a * a + c * c),
                             mass / 12f * (a * a + b * b));
        }

        public override Vector3[] LocalVertices { get { return verts; } }
        public override int[] Triangles { get { return tris; } }
        public override Vector3 RenderScale { get { return Size; } }
    }

    /// <summary>Sphère pleine de rayon r : I = 2/5 m r² (même valeur sur les 3 axes).</summary>
    public class SphereShape : Shape
    {
        public readonly float Radius;
        static readonly Vector3[] none = new Vector3[0];
        static readonly int[] noTris = new int[0];

        public SphereShape(float radius) { Radius = radius; }

        public override Mat3 InertiaBody(float mass)
        {
            float i = 0.4f * mass * Radius * Radius;
            return Mat3.Diag(i, i, i);
        }

        public override Vector3[] LocalVertices { get { return none; } }
        public override int[] Triangles { get { return noTris; } }
        public override Vector3 RenderScale { get { return Vector3.one * (2f * Radius); } }
    }
}
