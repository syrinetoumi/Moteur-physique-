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

        bool applied;                                    // le mesh a déjà été mis à jour au moins une fois

        void LateUpdate()
        {
            if (Body == null) return;
            if (Body.IsStatic && applied) return;        // corps immobile / endormi : son mesh n'a pas changé, on économise le calcul
            cube.ApplyTransformation(Body.ModelMatrix());
            applied = true;
        }
    }
}