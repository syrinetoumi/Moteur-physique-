// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

public class RigidCubeSimulation
{
    public readonly List<Particle> Particles =
        new List<Particle>();

    public float Radius { get; private set; }

    /// <summary>Centre de masse c(t) calcule au dernier shape matching.</summary>
    public Vector3 Center { get; private set; }

    readonly List<Vector3> restPositions =
        new List<Vector3>();

    readonly int phase;

    const float Epsilon = 0.000001f;

    // ============================================
    // MANUAL 3x3 MATRIX
    // ============================================

    struct Matrix3
    {
        public float a00, a01, a02;
        public float a10, a11, a12;
        public float a20, a21, a22;

        public static Matrix3 Identity()
        {
            Matrix3 m = new Matrix3();

            m.a00 = 1f;
            m.a11 = 1f;
            m.a22 = 1f;

            return m;
        }

        public static Matrix3 Transpose(Matrix3 m)
        {
            Matrix3 r = new Matrix3();

            r.a00 = m.a00;
            r.a01 = m.a10;
            r.a02 = m.a20;

            r.a10 = m.a01;
            r.a11 = m.a11;
            r.a12 = m.a21;

            r.a20 = m.a02;
            r.a21 = m.a12;
            r.a22 = m.a22;

            return r;
        }

        public static float Determinant(Matrix3 m)
        {
            return
                m.a00 * (m.a11 * m.a22 - m.a12 * m.a21) -
                m.a01 * (m.a10 * m.a22 - m.a12 * m.a20) +
                m.a02 * (m.a10 * m.a21 - m.a11 * m.a20);
        }

        public static bool Inverse(
            Matrix3 m,
            out Matrix3 result)
        {
            float det = Determinant(m);

            if (Mathf.Abs(det) < Epsilon)
            {
                result = Identity();
                return false;
            }

            float invDet = 1f / det;

            result = new Matrix3();

            result.a00 =
                (m.a11 * m.a22 - m.a12 * m.a21) * invDet;

            result.a01 =
                (m.a02 * m.a21 - m.a01 * m.a22) * invDet;

            result.a02 =
                (m.a01 * m.a12 - m.a02 * m.a11) * invDet;

            result.a10 =
                (m.a12 * m.a20 - m.a10 * m.a22) * invDet;

            result.a11 =
                (m.a00 * m.a22 - m.a02 * m.a20) * invDet;

            result.a12 =
                (m.a02 * m.a10 - m.a00 * m.a12) * invDet;

            result.a20 =
                (m.a10 * m.a21 - m.a11 * m.a20) * invDet;

            result.a21 =
                (m.a01 * m.a20 - m.a00 * m.a21) * invDet;

            result.a22 =
                (m.a00 * m.a11 - m.a01 * m.a10) * invDet;

            return true;
        }

        public static Matrix3 Add(
            Matrix3 a,
            Matrix3 b)
        {
            Matrix3 r = new Matrix3();

            r.a00 = a.a00 + b.a00;
            r.a01 = a.a01 + b.a01;
            r.a02 = a.a02 + b.a02;

            r.a10 = a.a10 + b.a10;
            r.a11 = a.a11 + b.a11;
            r.a12 = a.a12 + b.a12;

            r.a20 = a.a20 + b.a20;
            r.a21 = a.a21 + b.a21;
            r.a22 = a.a22 + b.a22;

            return r;
        }

        public static Matrix3 Scale(Matrix3 a, float s)
        {
            Matrix3 r = new Matrix3();

            r.a00 = a.a00 * s;
            r.a01 = a.a01 * s;
            r.a02 = a.a02 * s;

            r.a10 = a.a10 * s;
            r.a11 = a.a11 * s;
            r.a12 = a.a12 * s;

            r.a20 = a.a20 * s;
            r.a21 = a.a21 * s;
            r.a22 = a.a22 * s;

            return r;
        }

        public static Vector3 Multiply(
            Matrix3 m,
            Vector3 v)
        {
            return new Vector3(
                m.a00 * v.x + m.a01 * v.y + m.a02 * v.z,
                m.a10 * v.x + m.a11 * v.y + m.a12 * v.z,
                m.a20 * v.x + m.a21 * v.y + m.a22 * v.z
            );
        }

        public static float DifferenceSquared(
            Matrix3 a,
            Matrix3 b)
        {
            float d00 = a.a00 - b.a00;
            float d01 = a.a01 - b.a01;
            float d02 = a.a02 - b.a02;

            float d10 = a.a10 - b.a10;
            float d11 = a.a11 - b.a11;
            float d12 = a.a12 - b.a12;

            float d20 = a.a20 - b.a20;
            float d21 = a.a21 - b.a21;
            float d22 = a.a22 - b.a22;

            return
                d00 * d00 + d01 * d01 + d02 * d02 +
                d10 * d10 + d11 * d11 + d12 * d12 +
                d20 * d20 + d21 * d21 + d22 * d22;
        }
    }

    // ============================================
    // CONSTRUCTOR
    // ============================================

    public RigidCubeSimulation(
        Vector3 center,
        Vector3 size,
        float totalMass,
        int resolution,
        int particlePhase,
        float radiusFactor = 0.5f)
    {
        phase = particlePhase;

        resolution = Mathf.Max(2, resolution);

        float spacingX =
            size.x / (resolution - 1);

        float spacingY =
            size.y / (resolution - 1);

        float spacingZ =
            size.z / (resolution - 1);

        float spacing = Mathf.Min(
            spacingX,
            Mathf.Min(spacingY, spacingZ)
        );

        // Spheres qui se touchent (rayon = demi-espacement) : la surface du cube est pleine,
        // une particule d un autre cube ne peut pas se glisser entre les sphères (article 5.1 : locking).
        Radius = spacing * radiusFactor;

        int count =
            resolution * resolution * resolution;

        float particleMass =
            totalMass / count;

        for (int z = 0; z < resolution; z++)
        {
            for (int y = 0; y < resolution; y++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    Vector3 rest = new Vector3(
                        -size.x * 0.5f + x * spacingX,
                        -size.y * 0.5f + y * spacingY,
                        -size.z * 0.5f + z * spacingZ
                    );

                    // PhysicsLib : Particle(position, masse, phase)
                    Particle p = new Particle(center + rest, particleMass, phase);

                    Particles.Add(p);
                    restPositions.Add(rest);
                }
            }
        }
    }

    // ============================================
    // CENTER OF MASS
    // ============================================

    Vector3 CalculateCenterOfMass()
    {
        Vector3 sum = Vector3.zero;
        float totalMass = 0f;

        for (int i = 0; i < Particles.Count; i++)
        {
            float mass = Particles[i].Mass;

            sum += Particles[i].position * mass;
            totalMass += mass;
        }

        if (totalMass < Epsilon)
            return Vector3.zero;

        return sum / totalMass;
    }

    // ============================================
    // POLAR DECOMPOSITION
    // ============================================

    static Matrix3 ExtractRotation(Matrix3 covariance)
    {
        Matrix3 rotation = covariance;

        // Newton iteration:
        //
        // R(next) = 1/2 (R + inverse(R)^T)
        //
        // This extracts the orthogonal factor
        // of a nonsingular covariance matrix.

        for (int iteration = 0;
             iteration < 12;
             iteration++)
        {
            Matrix3 inverse;

            if (!Matrix3.Inverse(
                rotation,
                out inverse))
            {
                return Matrix3.Identity();
            }

            Matrix3 inverseTranspose =
                Matrix3.Transpose(inverse);

            Matrix3 next =
                Matrix3.Scale(
                    Matrix3.Add(
                        rotation,
                        inverseTranspose
                    ),
                    0.5f
                );

            float difference =
                Matrix3.DifferenceSquared(
                    next,
                    rotation
                );

            rotation = next;

            if (difference < 0.00000001f)
                break;
        }

        // Prevent a reflection from being used
        // as a rigid rotation.
        if (Matrix3.Determinant(rotation) < 0f)
            return Matrix3.Identity();

        return rotation;
    }

    // ============================================
    // SHAPE MATCHING
    // NVIDIA SECTION 5
    // EQUATIONS 15-16
    // ============================================

    public void SolveShapeMatching(float stiffness)
    {
        if (Particles.Count == 0)
            return;

        stiffness = Mathf.Clamp01(stiffness);

        Vector3 center =
            CalculateCenterOfMass();

        Center = center;

        Matrix3 covariance = new Matrix3();

        for (int i = 0; i < Particles.Count; i++)
        {
            Vector3 current =
                Particles[i].position - center;

            Vector3 rest =
                restPositions[i];

            float mass = Particles[i].Mass;

            covariance.a00 +=
                mass * current.x * rest.x;

            covariance.a01 +=
                mass * current.x * rest.y;

            covariance.a02 +=
                mass * current.x * rest.z;

            covariance.a10 +=
                mass * current.y * rest.x;

            covariance.a11 +=
                mass * current.y * rest.y;

            covariance.a12 +=
                mass * current.y * rest.z;

            covariance.a20 +=
                mass * current.z * rest.x;

            covariance.a21 +=
                mass * current.z * rest.y;

            covariance.a22 +=
                mass * current.z * rest.z;
        }

        Matrix3 rotation =
            ExtractRotation(covariance);

        for (int i = 0; i < Particles.Count; i++)
        {
            Vector3 goal =
                center +
                Matrix3.Multiply(
                    rotation,
                    restPositions[i]
                );

            Vector3 correction =
                goal - Particles[i].position;

            Particles[i].position +=
                correction * stiffness;
        }
    }

    // Kept for compatibility with older scripts.
    // Shape matching is the rigid constraint.
    public void SolveDistanceConstraints()
    {
    }
}