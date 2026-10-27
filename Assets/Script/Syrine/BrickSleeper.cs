using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Mise en sommeil des briques immobiles (idée de l'article, équation 14 : on fige une particule dont la vitesse
/// est tombée sous un seuil). Elle supprime la petite dérive numérique d'un mur au repos.
///  - une brique quasi immobile pendant TimeToSleep secondes est figée (vitesses remises à 0, IsStatic = true) ;
///  - elle est réveillée dès que la secousse du sol est active, ou qu'une brique rapide s'approche d'elle.
/// </summary>
public class BrickSleeper
{
    public float SpeedThreshold = 0.1f;      // vitesse (m/s) sous laquelle une brique est "immobile"
    public float AngularThreshold = 0.2f;    // vitesse de rotation (rad/s) sous laquelle elle est "immobile"
    public float TimeToSleep = 0.6f;         // durée d'immobilité avant de s'endormir (s)
    public float WakeSpeed = 0.6f;           // vitesse d'une brique voisine qui réveille les briques endormies (m/s)
    public float WakeDistance = 1.0f;        // marge de proximité pour le réveil (m)
    public float ShakeWakeLevel = 0.05f;     // intensité de secousse au-dessus de laquelle tout se réveille

    public int SleepingCount { get; private set; }

    float[] still = new float[0];

    /// <summary>A appeler une fois par frame, après world.Update.</summary>
    public void Update(List<RigidBody> bricks, float dt, float shakeIntensity)
    {
        int n = bricks.Count;
        if (still.Length != n) still = new float[n];

        if (shakeIntensity > ShakeWakeLevel)
        {
            for (int i = 0; i < n; i++) { bricks[i].IsStatic = false; still[i] = 0f; }
            SleepingCount = 0;
            return;
        }

        // réveil par approche : une brique rapide près d'une brique endormie la réveille
        for (int i = 0; i < n; i++)
        {
            if (!bricks[i].IsStatic) continue;
            for (int j = 0; j < n; j++)
            {
                if (bricks[j].IsStatic) continue;
                if (bricks[j].Velocity.magnitude < WakeSpeed) continue;
                float reach = WakeDistance + 0.5f * (bricks[i].Shape.RenderScale.magnitude + bricks[j].Shape.RenderScale.magnitude);
                if ((bricks[i].Position - bricks[j].Position).sqrMagnitude < reach * reach)
                { bricks[i].IsStatic = false; still[i] = 0f; break; }
            }
        }

        // endormissement des briques immobiles
        int sleeping = 0;
        for (int i = 0; i < n; i++)
        {
            RigidBody b = bricks[i];
            if (b.IsStatic) { sleeping++; continue; }

            if (b.Velocity.magnitude < SpeedThreshold && b.AngularVelocity.magnitude < AngularThreshold)
                still[i] += dt;
            else
                still[i] = 0f;

            if (still[i] >= TimeToSleep)
            {
                b.State.P = Vector3.zero;
                b.State.L = Vector3.zero;
                b.IsStatic = true;
                sleeping++;
            }
        }
        SleepingCount = sleeping;
    }
}
