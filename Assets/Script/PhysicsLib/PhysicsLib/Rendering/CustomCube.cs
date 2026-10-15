using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Construit un cube (mesh) unitaire dans le code, sur un GameObject vide, puis permet de lui
    /// appliquer une matrice 4x4 à la main (aucun Rigidbody, aucun Collider, aucune physique Unity).
    /// </summary>
    public class CustomCube : MonoBehaviour
    {
        public Color color = new Color(0.85f, 0.35f, 0.3f);
        public Material material;            // optionnel : sinon créé automatiquement

        Mesh mesh;
        Vector3[] originalVertices;
        Vector3[] transformedVertices;
        Material runtimeMaterial;

        void Awake()
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            if (mf == null) mf = gameObject.AddComponent<MeshFilter>();

            MeshRenderer mr = GetComponent<MeshRenderer>();
            if (mr == null) mr = gameObject.AddComponent<MeshRenderer>();

            mesh = BuildCubeMesh();
            mf.mesh = mesh;
            runtimeMaterial = material != null ? material : CreateMaterial();
            mr.material = runtimeMaterial;

            originalVertices = mesh.vertices;
            transformedVertices = new Vector3[originalVertices.Length];
        }

        public void SetColor(Color c)
        {
            color = c;
            if (runtimeMaterial != null) runtimeMaterial.color = c;
        }

        // Cube unitaire centré à l'origine : 6 faces x 4 sommets = 24 sommets, 12 triangles
        Mesh BuildCubeMesh()
        {
            Vector3[] N = { new Vector3(1,0,0), new Vector3(-1,0,0), new Vector3(0,1,0),
                            new Vector3(0,-1,0), new Vector3(0,0,1), new Vector3(0,0,-1) };
            Vector3[] U = { new Vector3(0,1,0), new Vector3(0,0,1), new Vector3(0,0,1),
                            new Vector3(1,0,0), new Vector3(1,0,0), new Vector3(0,1,0) };
            Vector3[] V = { new Vector3(0,0,1), new Vector3(0,1,0), new Vector3(1,0,0),
                            new Vector3(0,0,1), new Vector3(0,1,0), new Vector3(1,0,0) };

            Vector3[] vertices = new Vector3[24];
            Vector3[] normals = new Vector3[24];
            int[] triangles = new int[36];

            for (int f = 0; f < 6; f++)
            {
                Vector3 c = N[f] * 0.5f;
                Vector3 u = U[f] * 0.5f;
                Vector3 v = V[f] * 0.5f;
                int b = f * 4;

                vertices[b + 0] = c - u - v;
                vertices[b + 1] = c - u + v;
                vertices[b + 2] = c + u - v;
                vertices[b + 3] = c + u + v;
                for (int i = 0; i < 4; i++) normals[b + i] = N[f];

                int t = f * 6;
                triangles[t + 0] = b; triangles[t + 1] = b + 2; triangles[t + 2] = b + 1;
                triangles[t + 3] = b + 2; triangles[t + 4] = b + 3; triangles[t + 5] = b + 1;
            }

            Mesh m = new Mesh();
            m.name = "CustomCubeMesh";
            m.vertices = vertices;
            m.normals = normals;
            m.triangles = triangles;
            m.RecalculateBounds();
            return m;
        }

        Material CreateMaterial()
        {
            Shader s = Shader.Find("Universal Render Pipeline/Lit");
            if (s == null) s = Shader.Find("Standard");
            if (s == null) s = Shader.Find("Sprites/Default");
            Material m = new Material(s);
            m.color = color;
            return m;
        }

        /// <summary>Applique la matrice 4x4 (homogène) à chaque sommet du cube unitaire.</summary>
        public void ApplyTransformation(Matrix4x4 m)
        {
            for (int i = 0; i < originalVertices.Length; i++)
            {
                Vector3 v = originalVertices[i];
                transformedVertices[i] = new Vector3(
                    m.m00 * v.x + m.m01 * v.y + m.m02 * v.z + m.m03,
                    m.m10 * v.x + m.m11 * v.y + m.m12 * v.z + m.m13,
                    m.m20 * v.x + m.m21 * v.y + m.m22 * v.z + m.m23);
            }
            mesh.vertices = transformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }
    }
}
