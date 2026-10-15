using UnityEngine;

namespace PhysicsLib
{
    /// <summary>
    /// Relie un RigidBody (simulation) à un CustomCube (affichage) :
    /// à chaque frame, la matrice du corps est appliquée au mesh. Le moteur ne dépend pas de ce script.
    /// </summary>
    [RequireComponent(typeof(CustomCube))]
    public class BodyView : MonoBehaviour
    {
        public RigidBody Body;
        CustomCube cube;

        void Awake() { cube = GetComponent<CustomCube>(); }

        void LateUpdate()
        {
            if (Body != null) cube.ApplyTransformation(Body.ModelMatrix());
        }
    }
}
