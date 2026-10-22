using UnityEngine;
using PhysicsLib;

/// <summary>
/// Secousse du sol (séisme), exprimée comme une force sur chaque brique.
/// Quand le sol accélère de a_sol, dans le repère du sol chaque corps subit la force d'inertie  F = -m.a_sol.
/// On simule donc un sol qui vibre sans bouger le plan de collision : F(t) = m . A . intensité . sin(2.pi.f.t).
/// (Elle implémente IForce : elle s'ajoute comme n'importe quelle autre force du moteur.)
/// </summary>
public class ShakeForce : IForce
{
    public float Intensity;                // 0 = sol immobile, 1 = secousse maximale (pilotée par la souris)
    public float MaxAcceleration = 35f;    // amplitude max de l'accélération du sol (m/s²)
    public float Frequency = 1f;           // fréquence de vibration (Hz)
    public float ZFactor = 0.8f;           // part de la secousse selon z (profondeur) par rapport à x

    readonly PhysicsWorld world;

    public ShakeForce(PhysicsWorld world) { this.world = world; }

    public Vector3 Compute(Vector3 position, Vector3 velocity, float mass)
    {
        if (Intensity <= 0f) return Vector3.zero;
        float w = 2f * Mathf.PI * Frequency;
        float t = world.Time;
        float a = MaxAcceleration * Intensity;
        // vibration surtout selon x, un peu selon z (pour que les briques ne glissent pas toutes dans le même plan)
        return new Vector3(a * Mathf.Sin(w * t), 0f, ZFactor * a * Mathf.Sin(1.37f * w * t + 1f)) * mass;
    }
}