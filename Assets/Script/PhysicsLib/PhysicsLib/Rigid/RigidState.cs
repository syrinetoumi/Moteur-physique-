using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Vecteur d'état d'un corps rigide X(t) = (x(t), R(t), P(t), L(t))   (slide 37).
    /// Les opérateurs + et * scalaire permettent d'écrire Euler et RK4 de façon naturelle.
    /// </summary>
    public struct RigidState
    {
        public Vector3 x;   // position du centre de masse
        public Mat3 R;      // matrice d'orientation
        public Vector3 P;   // quantité de mouvement
        public Vector3 L;   // moment angulaire

        public static RigidState operator +(RigidState a, RigidState b)
        {
            RigidState s;
            s.x = a.x + b.x; s.R = a.R + b.R; s.P = a.P + b.P; s.L = a.L + b.L;
            return s;
        }

        public static RigidState operator *(RigidState a, float k)
        {
            RigidState s;
            s.x = a.x * k; s.R = a.R * k; s.P = a.P * k; s.L = a.L * k;
            return s;
        }
    }

    /// <summary>
    /// Force appliquée en un point du corps : F (repère global) en un point exprimé dans le repère de l'objet
    /// (r_i - x(t)). Elle dure "Duration" secondes (float.PositiveInfinity = permanente).
    /// </summary>
    public class ForceApplication
    {
        public Vector3 Force;
        public Vector3 LocalPoint;
        public float Duration;
        public float Elapsed;

        public ForceApplication(Vector3 force, Vector3 localPoint, float duration)
        { Force = force; LocalPoint = localPoint; Duration = duration; }

        // tolérance : l'addition répétée de dt en float peut laisser Elapsed un poil sous Duration
        public bool Active { get { return Elapsed < Duration - 1e-4f; } }
    }
}
