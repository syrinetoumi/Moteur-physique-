using System;
using PhysicsLib;
using UnityEditor;
using UnityEngine;

namespace Haithem.Chapitre1
{
    // Quelques tests à lancer depuis le menu Haithem, hors Play Mode.
    public static class Chapitre1Verification
    {
        [MenuItem("Haithem/Chapitre 1/Vérifier la géométrie")]
        public static void Verifier()
        {
            // Avec un quart de tour autour de Z, X devient Y.
            Quat quart = Quat.FromAxisAngle(Vector3.forward, Mathf.PI * 0.5f);
            Proche(quart.Rotate(Vector3.right), Vector3.up, "Rotation connue");
            Proche(quart.ToMat3() * Vector3.right, Vector3.up, "Matrice équivalente");
            Vector3 point = new Vector3(2f, -3f, 4f);
            Proche(quart.Conjugate().Rotate(quart.Rotate(point)), point, "Rotation inverse");
            Vector3 transforme = quart.ToMat3().ToMatrix4x4(new Vector3(3, 4, 5), new Vector3(2, 3, 4))
                .MultiplyPoint3x4(Vector3.right);
            Proche(transforme, new Vector3(3, 6, 5), "Ordre échelle, rotation, translation");

            GameObject objet = new GameObject("Verification chapitre 1");
            objet.hideFlags = HideFlags.HideAndDontSave;
            try
            {
                CubeRoulant cube = objet.AddComponent<CubeRoulant>();
                foreach (float taille in new[] { 1f, 2f })
                {
                    cube.taille = taille;
                    foreach (Vector3 direction in new[] { Vector3.right, Vector3.left, Vector3.forward, Vector3.back })
                    {
                        cube.Reinitialiser();
                        Exiger(cube.Rouler(direction), "Début du roulement");
                        Exiger(!cube.TournerSurPlace(), "Pas de commande concurrente");
                        cube.Avancer(cube.dureeTour * 0.5f);
                        // Le cube monte un peu pendant le roulement.
                        Exiger(Mathf.Abs(cube.Position.y - taille / Mathf.Sqrt(2f)) < 0.0001f, "Hauteur à mi-tour");
                        VerifierSol(cube);
                        cube.Avancer(cube.dureeTour);
                        Proche(cube.Position, direction * taille + Vector3.up * taille * 0.5f, "Déplacement d'une arête");
                        VerifierSol(cube);

                        // Après quatre tours, on retrouve la même orientation.
                        for (int i = 0; i < 3; i++)
                        {
                            cube.Rouler(direction);
                            cube.Avancer(cube.dureeTour);
                        }
                        Proche(cube.Orientation.Rotate(point), point, "Orientation après quatre tours");
                    }
                }

                cube.Reinitialiser();
                Vector3 centre = cube.Position;
                cube.TournerSurPlace();
                cube.Avancer(cube.dureeTour);
                Proche(cube.Position, centre, "Rotation sur place sans translation");
                Proche(cube.Orientation.Rotate(Vector3.forward), Vector3.right, "Quart de tour sur place");
                cube.Rouler(Vector3.forward);
                cube.Avancer(cube.dureeTour * 0.5f);
                VerifierSol(cube);
                cube.Reinitialiser();
                Exiger(!cube.EnMouvement, "Reset pendant un mouvement");
                Exiger(!cube.Rouler(Vector3.one), "Direction non cardinale refusée");

                cube.Rouler(Vector3.right);
                cube.Avancer(cube.dureeTour);
                Vector3 final = cube.Position;
                Quat orientation = cube.Orientation;
                cube.Reinitialiser();
                cube.Rouler(Vector3.right);
                for (int i = 0; i < 60; i++) cube.Avancer(cube.dureeTour / 59f);
                Proche(cube.Position, final, "Indépendance du nombre d'images");
                Proche(cube.Orientation.Rotate(point), orientation.Rotate(point), "Orientation indépendante du framerate");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(objet);
            }
            Debug.Log("Chapitre 1 : tests OK");
        }

        static void VerifierSol(CubeRoulant cube)
        {
            for (int i = 0; i < 8; i++)
            {
                Vector3 sommet = new Vector3((i & 1) == 0 ? -1 : 1,
                    (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1) * cube.taille * 0.5f;
                Vector3 monde = cube.Position + cube.Orientation.Rotate(sommet);
                Exiger(monde.y >= -0.0001f, "Sommet au-dessus du sol");
            }
        }

        static void Proche(Vector3 obtenu, Vector3 attendu, string message)
        {
            Exiger((obtenu - attendu).sqrMagnitude < 0.000001f, message + " : " + obtenu + " / " + attendu);
        }

        static void Exiger(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException("Chapitre 1 : " + message);
        }
    }
}
