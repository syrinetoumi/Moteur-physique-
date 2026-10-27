using PhysicsLib;
using UnityEngine;

namespace Haithem.Chapitre2Partie1
{
    // Euler avec la vitesse et l'accélération du début du pas.
    public class EulerExplicite : IIntegrator
    {
        Vector3[] acceleration = new Vector3[0];

        public void Step(Vector3[] positions, Vector3[] vitesses, float dt, AccelerationFunc calculerAcceleration)
        {
            if (acceleration.Length != positions.Length)
                acceleration = new Vector3[positions.Length];

            calculerAcceleration(positions, vitesses, acceleration);
            for (int i = 0; i < positions.Length; i++)
            {
                positions[i] += vitesses[i] * dt;
                vitesses[i] += acceleration[i] * dt;
            }
        }
    }
}
