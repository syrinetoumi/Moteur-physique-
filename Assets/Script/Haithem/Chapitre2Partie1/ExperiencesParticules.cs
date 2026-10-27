using PhysicsLib;
using UnityEngine;

namespace Haithem.Chapitre2Partie1
{
    public static class ExperiencesParticules
    {
        public static ParticleGroup CreerChute(IIntegrator methode, float x, float hauteur, float masse, float frottement)
        {
            ParticleGroup groupe = new ParticleGroup();
            groupe.Integrator = methode;
            groupe.Add(new Particle(new Vector3(x, hauteur, 0f), masse));
            groupe.Forces.Add(new GravityForce());
            groupe.Forces.Add(new ViscousDragForce(frottement));
            return groupe;
        }

        public static ParticleGroup CreerRessorts(float masse, float raideur, float amortissement)
        {
            ParticleGroup groupe = new ParticleGroup();
            groupe.Integrator = new RK4Integrator();
            groupe.Add(new Particle(new Vector3(0f, 7f, 0f), 0f));
            groupe.Add(new Particle(new Vector3(0f, 4f, 0f), masse));
            groupe.Add(new Particle(new Vector3(0f, 1f, 0f), masse));
            groupe.Forces.Add(new GravityForce());

            // Le point 0 est fixe, les deux autres sont reliés par des ressorts.
            groupe.PairForces.Add(new SpringForce(0, 1, raideur, 1.5f, amortissement));
            groupe.PairForces.Add(new SpringForce(1, 2, raideur, 1.5f, amortissement));
            return groupe;
        }

        // Solution exacte de la chute avec F = mg - kv, en partant du repos.
        public static float HauteurExacte(float temps, float hauteur, float masse, float frottement)
        {
            if (frottement == 0f) return hauteur - 0.5f * 9.81f * temps * temps;
            float taux = frottement / masse;
            return hauteur - 9.81f / taux * (temps - (1f - Mathf.Exp(-taux * temps)) / taux);
        }

        public static float VitesseExacte(float temps, float masse, float frottement)
        {
            if (frottement == 0f) return -9.81f * temps;
            float taux = frottement / masse;
            return -9.81f / taux * (1f - Mathf.Exp(-taux * temps));
        }
    }
}
