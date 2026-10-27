using System;
using PhysicsLib;
using UnityEditor;
using UnityEngine;

namespace Haithem.Chapitre2Partie1
{
    // Tests des calculs, sans lancer la scène.
    public static class Chapitre2Verification
    {
        [MenuItem("Haithem/Chapitre 2 Partie 1/Vérifier les calculs")]
        public static void Verifier()
        {
            ParticleGroup debut = ExperiencesParticules.CreerChute(new EulerExplicite(), 0f, 8f, 1f, 1f);
            debut.Step(0.1f);
            Proche(debut.Particles[0].position.y, 8f, 0.00001f, "Euler utilise l'ancienne vitesse");
            Proche(debut.Particles[0].velocity.y, -0.981f, 0.00001f, "Accélération au départ");

            float euler = ErreurChute(0.1f, false);
            float eulerFin = ErreurChute(0.05f, false);
            float rk4 = ErreurChute(0.2f, true);
            float rk4Fin = ErreurChute(0.1f, true);
            VerifierCondition(eulerFin < euler * 0.8f, "Euler devient plus précis avec un pas plus petit");
            VerifierCondition(rk4Fin < rk4 * 0.8f, "RK4 devient plus précis avec un pas plus petit");
            VerifierCondition(rk4Fin < eulerFin * 0.01f, "RK4 est plus précis sur cette chute");

            ParticleGroup sansFrottement = ExperiencesParticules.CreerChute(new RK4Integrator(), 0f, 8f, 2f, 0f);
            for (int i = 0; i < 100; i++) sansFrottement.Step(0.01f);
            Proche(sansFrottement.Particles[0].position.y, 8f - 9.81f * 0.5f, 0.0001f, "Chute sans frottement");
            Proche(ExperiencesParticules.HauteurExacte(1f, 8f, 2f, 0f), sansFrottement.Particles[0].position.y, 0.0001f, "Solution exacte sans frottement");

            ParticleGroup ressorts = ExperiencesParticules.CreerRessorts(0.5f, 20f, 1.5f);
            Vector3 attache = ressorts.Particles[0].position;
            float energieDepart = Energie(ressorts);
            for (int i = 0; i < 2000; i++) ressorts.Step(0.01f);
            Proche((ressorts.Particles[0].position - attache).sqrMagnitude, 0f, 0.000001f, "Point fixe");
            // Le premier ressort porte les deux masses, le second une seule.
            float y1 = 7f - 1.5f - 2f * 0.5f * 9.81f / 20f;
            float y2 = y1 - 1.5f - 0.5f * 9.81f / 20f;
            Proche(ressorts.Particles[1].position.y, y1, 0.01f, "Équilibre masse 1");
            Proche(ressorts.Particles[2].position.y, y2, 0.01f, "Équilibre masse 2");
            VerifierCondition(Energie(ressorts) < energieDepart - 0.1f, "L'amortissement retire de l'énergie");

            Debug.Log("Chapitre 2 partie 1 : tests OK / erreur Euler = " + eulerFin + " / erreur RK4 = " + rk4Fin);
        }

        static float ErreurChute(float dt, bool utiliserRK4)
        {
            IIntegrator methode = utiliserRK4 ? (IIntegrator)new RK4Integrator() : new EulerExplicite();
            ParticleGroup groupe = ExperiencesParticules.CreerChute(methode, 0f, 8f, 1f, 1f);
            int pas = Mathf.RoundToInt(2f / dt);
            for (int i = 0; i < pas; i++) groupe.Step(dt);
            float y = ExperiencesParticules.HauteurExacte(2f, 8f, 1f, 1f);
            float v = ExperiencesParticules.VitesseExacte(2f, 1f, 1f);
            return Mathf.Abs(groupe.Particles[0].position.y - y) + Mathf.Abs(groupe.Particles[0].velocity.y - v);
        }

        static float Energie(ParticleGroup groupe)
        {
            float energie = 0f;
            foreach (Particle particule in groupe.Particles)
                energie += 0.5f * particule.Mass * particule.velocity.sqrMagnitude + particule.Mass * 9.81f * particule.position.y;
            for (int i = 0; i < 2; i++)
            {
                float allongement = (groupe.Particles[i].position - groupe.Particles[i + 1].position).magnitude - 1.5f;
                energie += 0.5f * 20f * allongement * allongement;
            }
            return energie;
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
