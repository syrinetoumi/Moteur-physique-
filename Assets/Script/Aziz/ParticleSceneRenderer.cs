using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

public class ParticleSceneRenderer
{
    readonly Mesh mesh;
    readonly List<Particle> particles;
    readonly float radius;

    Vector3[] baseVerts;
    int[] baseTris;
    int vertsPerSphere;
    int trisPerSphere;

    Vector3[] allVertices;
    Vector3[] allNormals;
    int[] allTriangles;

    public ParticleSceneRenderer(
        string name,
        List<Particle> particles,
        float radius,
        Color color)
    {
        this.particles = particles;
        this.radius = radius;

        BuildIcosphere();

        int count = particles.Count;

        allVertices =
            new Vector3[count * vertsPerSphere];

        allNormals =
            new Vector3[count * vertsPerSphere];

        // Normale d'une sphere = direction du sommet : elle ne depend pas de la position.
        // On la calcule UNE fois (RecalculateNormals a chaque frame coutait cher).
        for (int i = 0; i < count; i++)
            for (int j = 0; j < vertsPerSphere; j++)
                allNormals[i * vertsPerSphere + j] = baseVerts[j];

        allTriangles =
            new int[count * trisPerSphere];

        for (int i = 0; i < count; i++)
        {
            int triOffset = i * trisPerSphere;
            int vertOffset = i * vertsPerSphere;

            for (int j = 0; j < trisPerSphere; j++)
            {
                allTriangles[triOffset + j] =
                    vertOffset + baseTris[j];
            }
        }

        mesh = new Mesh();
        mesh.name = name;
        mesh.indexFormat =
            UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.MarkDynamic();

        mesh.vertices = allVertices;
        mesh.normals = allNormals;
        mesh.triangles = allTriangles;

        GameObject go = new GameObject(name);

        MeshFilter filter =
            go.AddComponent<MeshFilter>();

        MeshRenderer renderer =
            go.AddComponent<MeshRenderer>();

        filter.mesh = mesh;

        Shader shader =
            Shader.Find("Universal Render Pipeline/Lit");

        if (shader == null)
            shader = Shader.Find("Standard");

        Material mat = new Material(shader);
        mat.color = color;

        renderer.material = mat;

        Update();
    }

    void BuildIcosphere()
    {
        // Low-poly sphere (octahedron subdivided once).

        float t = 0.5f;

        baseVerts = new Vector3[]
        {
            Vec.Normalize(new Vector3(0, 1, 0)),
            Vec.Normalize(new Vector3(0, -1, 0)),

            Vec.Normalize(new Vector3(1, 0, 0)),
            Vec.Normalize(new Vector3(-1, 0, 0)),

            Vec.Normalize(new Vector3(0, 0, 1)),
            Vec.Normalize(new Vector3(0, 0, -1)),

            // Edge midpoints.
            Vec.Normalize(new Vector3(t, t, 0)),
            Vec.Normalize(new Vector3(-t, t, 0)),
            Vec.Normalize(new Vector3(t, -t, 0)),
            Vec.Normalize(new Vector3(-t, -t, 0)),

            Vec.Normalize(new Vector3(0, t, t)),
            Vec.Normalize(new Vector3(0, -t, t)),
            Vec.Normalize(new Vector3(0, t, -t)),
            Vec.Normalize(new Vector3(0, -t, -t)),

            Vec.Normalize(new Vector3(t, 0, t)),
            Vec.Normalize(new Vector3(-t, 0, t)),
            Vec.Normalize(new Vector3(t, 0, -t)),
            Vec.Normalize(new Vector3(-t, 0, -t)),
        };

        baseTris = new int[]
        {
            0, 6, 10,
            0, 10, 7,
            0, 7, 12,
            0, 12, 6,

            6, 2, 14,
            6, 14, 10,
            10, 14, 4,
            10, 4, 15,

            10, 15, 7,
            7, 15, 3,
            7, 3, 17,
            7, 17, 12,

            12, 17, 5,
            12, 5, 16,
            12, 16, 6,
            6, 16, 2,

            1, 8, 11,
            1, 11, 9,
            1, 9, 13,
            1, 13, 8,

            8, 2, 16,
            8, 16, 13,
            13, 16, 5,
            13, 5, 17,

            13, 17, 9,
            9, 17, 3,
            9, 3, 15,
            9, 15, 11,

            11, 15, 4,
            11, 4, 14,
            11, 14, 8,
            8, 14, 2,
        };

        vertsPerSphere = baseVerts.Length;
        trisPerSphere = baseTris.Length;
    }

    public void Update()
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Vector3 center = particles[i].position;
            int offset = i * vertsPerSphere;

            for (int j = 0; j < vertsPerSphere; j++)
            {
                allVertices[offset + j] =
                    center + baseVerts[j] * radius;
            }
        }

        mesh.vertices = allVertices;
        mesh.RecalculateBounds();
    }
}