using System.Collections.Generic;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

using PhysicsLib;

/// <summary>
/// Scene de demonstration de PhysicsLib.
///
/// Boite A : chute libre + frottement visqueux.
/// Boite B : force excentree -> translation + rotation.
/// Boite C : rotation libre avec moment angulaire initial.
///
/// La physique est calculee uniquement par PhysicsLib.
///
/// Aucun Rigidbody.
/// Aucun Collider.
/// Aucun Transform pour les calculs physiques.
/// Aucune fonction mathematique vectorielle Unity.
///
/// Chapitre 0 : forces et integration.
/// Chapitre 2 Partie 2 : rotation, torque, inertie.
/// Chapitre 3 : detection des contacts.
///
/// Touche R : reinitialiser.
/// </summary>
public class PhysicsLibDemo : MonoBehaviour
{
    [Header("Integration")]
    public bool useRK4 = true;

    private PhysicsWorld world;

    private readonly List<GameObject> spawned =
        new List<GameObject>();

    private RigidBody a;
    private RigidBody b;
    private RigidBody c;

    void Start()
    {
        Build();
    }

    void Build()
    {
        // Supprimer les anciens objets visuels.
        for (int i = 0; i < spawned.Count; i++)
        {
            if (spawned[i] != null)
            {
                Destroy(spawned[i]);
            }
        }

        spawned.Clear();

        // Creer le monde physique.
        world = new PhysicsWorld();

        // Gravite.
        world.AddGlobalForce(
            new GravityForce(
                new Vector3(0f, -9.81f, 0f)
            )
        );

        // Plan du sol : y = 0.
        Plane3 ground = Plane3.FromPointNormal(
            Vector3.zero,
            new Vector3(0f, 1f, 0f)
        );

        // Detection des contacts.
        world.Detector = new PlaneContactDetector(
            ground
        );

        // Aucune resolution de collision pour
        // cette demonstration.
        world.Resolver = null;

        // Choix de l'integrateur.
        IntegrationMethod method =
            useRK4
                ? IntegrationMethod.RK4
                : IntegrationMethod.Euler;

        // -----------------------------------
        // Boite A : chute libre + frottement
        // -----------------------------------

        a = Spawn(
            new Vector3(1f, 1f, 1f),
            new Vector3(-3f, 8f, 0f),
            new Color(0.85f, 0.3f, 0.3f),
            method
        );

        a.Forces.Add(
            new ViscousDragForce(1f)
        );

        // -----------------------------------
        // Boite B : force excentree
        // -----------------------------------

        b = Spawn(
            new Vector3(1f, 1f, 1f),
            new Vector3(0f, 8f, 0f),
            new Color(0.3f, 0.5f, 0.9f),
            method
        );

        // Force appliquee hors du centre de masse.
        // Produit une translation et un moment.
        //
        // tau = r x F
        //
        // Chapitre 2 Partie 2, pages 20-23.

        b.ApplyForceAtPoint(
            new Vector3(30f, 0f, 0f),
            new Vector3(0f, 0.5f, 0f),
            0.3f
        );

        // -----------------------------------
        // Boite C : rotation libre
        // -----------------------------------

        c = Spawn(
            new Vector3(2f, 0.5f, 1f),
            new Vector3(3f, 8f, 0f),
            new Color(0.3f, 0.75f, 0.4f),
            method
        );

        c.SetAngularVelocity(
            new Vector3(0.5f, 6f, 0.3f)
        );

        // IMPORTANT :
        // La camera doit etre positionnee
        // manuellement dans l'Inspector Unity.
        //
        // Aucun Transform ni Quaternion Unity.
    }

    RigidBody Spawn(
        Vector3 size,
        Vector3 position,
        Color color,
        IntegrationMethod method)
    {
        // Creation de la geometrie physique.
        BoxShape shape = new BoxShape(size);

        // Creation du corps rigide.
        RigidBody body = new RigidBody(
            shape,
            1f,
            position
        );

        body.Method = method;

        // Ajouter au moteur physique.
        world.AddBody(body);

        // -----------------------------------
        // Affichage uniquement
        // -----------------------------------

        GameObject go = new GameObject(
            "PhysicsLib_Box"
        );

        // Mesh genere par PhysicsLib.
        CustomCube cube =
            go.AddComponent<CustomCube>();

        cube.SetColor(color);

        // Liaison physique -> affichage.
        BodyView view =
            go.AddComponent<BodyView>();

        view.Body = body;

        spawned.Add(go);

        return body;
    }

    void FixedUpdate()
    {
        if (world == null)
            return;

        // Pas fixe fourni par Unity.
        // Les calculs physiques sont effectues
        // exclusivement dans PhysicsLib.

        world.Update(Time.fixedDeltaTime);
    }

    void Update()
    {
        if (ResetPressed())
        {
            Build();
        }
    }

    static bool ResetPressed()
    {
#if ENABLE_INPUT_SYSTEM

        return Keyboard.current != null
            && Keyboard.current.rKey.wasPressedThisFrame;

#else

        return Input.GetKeyDown(KeyCode.R);

#endif
    }

    void OnGUI()
    {
        if (world == null ||
            a == null ||
            b == null ||
            c == null)
        {
            return;
        }

        // Norme du moment angulaire
        // calculee manuellement par Vec.Length.

        float angularMomentum =
            Vec.Length(c.State.L);

        string information =
            "PHYSICSLIB - DEMONSTRATION\n" +

            "R : Recommencer\n" +

            "Methode : " +
            (useRK4 ? "RK4" : "Euler") +
            "\n\n" +

            "Temps : " +
            world.Time.ToString("F2") +
            " s\n" +

            "Contacts detectes : " +
            world.Contacts.Count +
            "\n\n" +

            "Boite A (chute libre)\n" +
            "Y = " +
            a.Position.y.ToString("F2") +
            " | V = " +
            a.Velocity +
            "\n\n" +

            "Boite B (force excentree)\n" +
            "X = " +
            b.Position.x.ToString("F2") +
            " | Omega = " +
            b.AngularVelocity +
            "\n\n" +

            "Boite C (rotation libre)\n" +
            "|L| = " +
            angularMomentum.ToString("F3") +
            " | Ec = " +
            c.KineticEnergy.ToString("F3");

        GUI.Label(
            new Rect(10f, 10f, 700f, 350f),
            information
        );
    }
}