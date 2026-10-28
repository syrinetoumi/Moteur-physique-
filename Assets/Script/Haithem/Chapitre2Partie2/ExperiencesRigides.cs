using PhysicsLib;
using UnityEngine;

namespace Haithem.Chapitre2Partie2
{
    public static class ExperiencesRigides
    {
        public static PhysicsWorld Creer(int exemple, bool utiliserRK4)
        {
            PhysicsWorld monde = new PhysicsWorld();
            Vector3 tailleDroite = exemple == 0 ? Vector3.one : new Vector3(2f, 0.5f, 1f);
            RigidBody gauche = new RigidBody(new BoxShape(Vector3.one), 1f, new Vector3(-3f, 0f, 0f));
            RigidBody droite = new RigidBody(new BoxShape(tailleDroite), 1f, new Vector3(3f, 0f, 0f));
            gauche.Method = droite.Method = utiliserRK4 ? IntegrationMethod.RK4 : IntegrationMethod.Euler;
            monde.AddBody(gauche);
            monde.AddBody(droite);

            Vector3 force = new Vector3(2f, 0f, 0f);
            Vector3 point = new Vector3(0f, 0.25f, 0f);
            if (exemple == 0)
            {
                // Même force : au centre à gauche, décalée à droite.
                gauche.ApplyForce(force, 0.5f);
                droite.ApplyForceAtPoint(force, point, 0.5f);
            }
            else if (exemple == 1)
            {
                // Même masse et même force, mais des formes différentes.
                gauche.ApplyForceAtPoint(force, point, 0.5f);
                droite.ApplyForceAtPoint(force, point, 0.5f);
            }
            else
            {
                // Sans force extérieure, le moment angulaire doit rester constant.
                gauche.SetAngularVelocity(new Vector3(1f, 2f, 0.5f));
                droite.SetAngularVelocity(new Vector3(1f, 2f, 0.5f));
            }
            return monde;
        }
    }
}
