using System;
using PhysicsLib;
using UnityEditor;
using UnityEngine;

namespace Haithem.Chapitre2Partie2
{
    public static class CorpsRigidesVerification
    {
        [MenuItem("Haithem/Chapitre 2 Partie 2/Vérifier les calculs")]
        public static void Verifier()
        {
            PhysicsWorld forces = ExperiencesRigides.Creer(0, true);
            for (int i = 0; i < 500; i++) forces.Step(0.002f);
            RigidBody centre = forces.Bodies[0];
            RigidBody decale = forces.Bodies[1];
            Proche(centre.Velocity.x, 1f, 0.001f, "Force appliquée pendant 0,5 seconde");
            Proche(centre.Position.x + 3f, 0.75f, 0.002f, "Translation attendue");
            Proche((centre.State.P - decale.State.P).magnitude, 0f, 0.0001f, "Même quantité de mouvement");
            Proche(centre.State.L.magnitude, 0f, 0.0001f, "Pas de couple au centre");
            VerifierCondition(decale.State.L.z < -0.1f, "Rotation avec une force décalée");
            VerifierCondition(centre.AppliedForces.Count == 0 && decale.AppliedForces.Count == 0, "Forces terminées");

            PhysicsWorld inertie = ExperiencesRigides.Creer(1, true);
            Proche(inertie.Bodies[0].InertiaWorld.c2.z, 1f / 6f, 0.00001f, "Inertie du cube");
            Proche(inertie.Bodies[1].InertiaWorld.c2.z, 4.25f / 12f, 0.00001f, "Inertie du pavé");
            for (int i = 0; i < 5; i++) inertie.Step(0.002f);
            VerifierCondition(Mathf.Abs(inertie.Bodies[0].AngularVelocity.z) > Mathf.Abs(inertie.Bodies[1].AngularVelocity.z), "Le cube tourne plus vite au départ");

            PhysicsWorld libre = ExperiencesRigides.Creer(2, true);
            Vector3[] moments = { libre.Bodies[0].State.L, libre.Bodies[1].State.L };
            float[] energies = { libre.Bodies[0].KineticEnergy, libre.Bodies[1].KineticEnergy };
            for (int i = 0; i < 1500; i++) libre.Step(0.002f);
            for (int i = 0; i < 2; i++)
            {
                RigidBody corps = libre.Bodies[i];
                Proche((corps.State.L - moments[i]).magnitude, 0f, 0.00001f, "Moment angulaire conservé");
                Proche(corps.KineticEnergy, energies[i], 0.001f, "Énergie en rotation libre RK4");
                Proche(corps.Position.magnitude, 3f, 0.0001f, "Centre immobile");
                VerifierOrientation(corps.Orientation);
            }

            PhysicsWorld euler = ExperiencesRigides.Creer(2, false);
            for (int i = 0; i < 1500; i++) euler.Step(0.002f);
            VerifierOrientation(euler.Bodies[1].Orientation);
            Debug.Log("Chapitre 2 partie 2 : tests OK");
        }

        static void VerifierOrientation(Mat3 r)
        {
            Proche(r.c0.magnitude, 1f, 0.0001f, "Axe X unitaire");
            Proche(r.c1.magnitude, 1f, 0.0001f, "Axe Y unitaire");
            Proche(r.c2.magnitude, 1f, 0.0001f, "Axe Z unitaire");
            Proche(Vector3.Dot(r.c0, r.c1), 0f, 0.0001f, "Axes perpendiculaires");
            Proche(r.Determinant(), 1f, 0.0001f, "Rotation sans déformation");
        }

        [MenuItem("Haithem/Chapitre 2 Partie 2/Vérifier la lecture (Play Mode)")]
        public static void VerifierLecture()
        {
            if (!EditorApplication.isPlaying) throw new InvalidOperationException("Lancer la scène en Play Mode.");
            DemoCorpsRigides demo = UnityEngine.Object.FindFirstObjectByType<DemoCorpsRigides>();
            if (demo == null) throw new InvalidOperationException("Ouvrir la scène des corps rigides.");
            if (demo.EnPause) demo.ChangerPause();
            demo.Reinitialiser();
            VerifierCondition(!demo.EnPause, "Reset ne doit pas mettre en pause");
            for (int mode = 0; mode < 3; mode++)
            {
                demo.ChoisirExemple(mode);
                for (int i = 0; i < 100; i++) demo.Avancer(0.1f);
                VerifierCondition(!demo.EnPause && demo.Monde.Time > 9.5f, "La simulation doit dépasser trois secondes");
                foreach (RigidBody corps in demo.Monde.Bodies)
                {
                    Vector3 point = Camera.main.WorldToViewportPoint(corps.Position);
                    VerifierCondition(point.z > 0f && point.x > 0f && point.x < 1f && point.y > 0f && point.y < 1f,
                        "La caméra doit garder les corps visibles");
                }
            }
            demo.ChangerPause();
            float temps = demo.Monde.Time;
            demo.Avancer(0.1f);
            Proche(demo.Monde.Time, temps, 0.00001f, "Pause demandée");
            demo.ChoisirExemple(0);
            VerifierCondition(demo.EnPause, "Le choix doit garder la pause demandée");
            Debug.Log("Lecture corps rigides : tests OK");
        }

        static void Proche(float obtenu, float attendu, float tolerance, string message)
        {
            VerifierCondition(Mathf.Abs(obtenu - attendu) < tolerance, message + " : " + obtenu + " / " + attendu);
        }

        static void VerifierCondition(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }
    }
}
