using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
using PhysicsLib;

/// <summary>
/// Scène de test de la bibliothèque : à glisser sur un GameObject vide.
///  - Boîte A (rouge)  : chute libre + frottements visqueux
///  - Boîte B (bleue)  : force excentrée pendant 0.3 s -> translation + rotation
///  - Boîte C (verte)  : rotation libre (moment angulaire initial)
/// Pas de collision résolue pour l'instant : les boîtes traversent le sol, mais les contacts sont comptés.
/// Touche R : recommencer.
/// </summary>
public class PhysicsLibDemo : MonoBehaviour
{
    public bool useRK4 = true;

    PhysicsWorld world;
    readonly List<GameObject> spawned = new List<GameObject>();
    RigidBody a, b, c;

    void Start() { Build(); }

    void Build()
    {
        foreach (GameObject go in spawned) Destroy(go);
        spawned.Clear();

        world = new PhysicsWorld();
        world.AddGlobalForce(new GravityForce());
        world.Detector = new PlaneContactDetector(Plane3.FromPointNormal(Vector3.zero, Vector3.up));

        IntegrationMethod m = useRK4 ? IntegrationMethod.RK4 : IntegrationMethod.Euler;

        a = Spawn(new Vector3(1, 1, 1), new Vector3(-3, 8, 0), new Color(0.85f, 0.3f, 0.3f), m);
        a.Forces.Add(new ViscousDragForce(1f));

        b = Spawn(new Vector3(1, 1, 1), new Vector3(0, 8, 0), new Color(0.3f, 0.5f, 0.9f), m);
        b.ApplyForceAtPoint(new Vector3(30f, 0, 0), new Vector3(0, 0.5f, 0), 0.3f);

        c = Spawn(new Vector3(2, 0.5f, 1), new Vector3(3, 8, 0), new Color(0.3f, 0.75f, 0.4f), m);
        c.SetAngularVelocity(new Vector3(0.5f, 6f, 0.3f));

        if (Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(0, 5, -16);
            Camera.main.transform.rotation = Quaternion.Euler(10, 0, 0);
        }
    }

    RigidBody Spawn(Vector3 size, Vector3 pos, Color color, IntegrationMethod method)
    {
        RigidBody body = new RigidBody(new BoxShape(size), 1f, pos);
        body.Method = method;
        world.AddBody(body);

        GameObject go = new GameObject("Box");
        CustomCube cube = go.AddComponent<CustomCube>();
        cube.SetColor(color);
        BodyView view = go.AddComponent<BodyView>();
        view.Body = body;
        spawned.Add(go);
        return body;
    }

    void Update()
    {
        if (ResetPressed()) Build();
        world.Update(Time.deltaTime);
    }

    static bool ResetPressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.R);
#endif
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 700, 160),
            "R = recommencer | Méthode : " + (useRK4 ? "RK4" : "Euler") + "\n" +
            "t = " + world.Time.ToString("F2") + " s | contacts (sommets sous le sol) = " + world.Contacts.Count + "\n" +
            "A : y = " + a.Position.y.ToString("F2") + "  v = " + a.Velocity + "\n" +
            "B : x = " + b.Position.x.ToString("F2") + "  w = " + b.AngularVelocity + "\n" +
            "C : |L| = " + c.State.L.magnitude.ToString("F3") + "  Ec = " + c.KineticEnergy.ToString("F3"));
    }
}
