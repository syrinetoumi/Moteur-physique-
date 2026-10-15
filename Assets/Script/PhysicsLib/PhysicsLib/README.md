# PhysicsLib — bibliothèque commune du projet Moteur Physique (5GamiX)

Bibliothèque partagée par les 3 membres du groupe. **Aucune physique Unity** : pas de `Rigidbody`,
pas de `Collider`, pas de `Physics.*`. Tout est calculé avec nos propres classes (cours Ch.1, 2 et 3).

## 1. Installation
1. Copier le dossier `PhysicsLib/` dans `Assets/` de votre projet Unity (ou le mettre dans le dépôt Git commun).
2. **Supprimer vos anciens fichiers du TP** `Mat3.cs`, `Quat.cs`, `CustomCube.cs`, `RigidBodyEuler.cs` :
   les versions de la bibliothèque les remplacent (sinon Unity garde silencieusement votre ancienne version).
3. En haut de vos scripts : `using PhysicsLib;`
4. Tester : glisser `Demo/PhysicsLibDemo.cs` sur un GameObject vide d'une scène vide, puis Play (touche R = recommencer).

## 2. Règles d'équipe
- Un fichier = une classe, même nom. Tout est dans le namespace `PhysicsLib`.
- Commentaire `///` au-dessus de chaque classe/méthode, avec le numéro de slide ou l'équation du cours
  (c'est ce qui nourrit la *Documentation technique* de la grille).
- On **ajoute** des classes (nouvelle force, nouvelle forme, nouveau détecteur) ; on ne modifie pas
  une classe existante sans prévenir les deux autres. Petits commits, un message clair.
- Si un besoin est commun (ex : une fonction géométrique), il va dans la bibliothèque, pas dans votre scène.

## 3. Contenu

| Dossier | Classes | Cours |
|---|---|---|
| `Math/` | `Mat3` (matrice 3x3, inverse, Gram-Schmidt, `Star`), `Quat` (quaternion, `ToMat3`) | Ch.1 |
| `Geometry/` | `Ray3`, `Plane3`, `Triangle3`, `Intersections` (rayon-plan, rayon-triangle, rayon-sphère, barycentriques, distances point-droite et droite-droite) | Ch.3 |
| `Dynamics/` | `IForce`, `IPairForce`, `GravityForce`, `ViscousDragForce`, `ConstantForce`, `SpringForce`, `EulerIntegrator`, `RK4Integrator`, `Ode`, `Particle`, `ParticleGroup` | Ch.2 partie 1 |
| `Rigid/` | `Shape` (`BoxShape`, `SphereShape`), `RigidState`, `ForceApplication`, `RigidBody` (Euler ou RK4) | Ch.2 partie 2 |
| `Collision/` | `Contact`, `ICollisionDetector`, `ICollisionResolver`, `PlaneContactDetector` (point-plan) | Ch.3 |
| `World/` | `PhysicsWorld` (pas fixe 0.002 s, forces globales, détection/résolution) | — |
| `Rendering/` | `CustomCube` (cube construit dans le code), `BodyView` (relie un corps à son cube) | TP |
| `Demo/` | `PhysicsLibDemo` | — |

## 4. Exemples

**Un corps rigide**
```csharp
var world = new PhysicsWorld();
world.AddGlobalForce(new GravityForce());                       // gravité pour tous les corps

var body = world.AddBody(new RigidBody(new BoxShape(new Vector3(1, 0.5f, 2)), 1f, new Vector3(0, 5, 0)));
body.Method = IntegrationMethod.RK4;                            // ou Euler
body.ApplyForceAtPoint(new Vector3(10, 0, 0), new Vector3(0, 0.25f, 0), 0.3f);   // force excentrée 0.3 s

// dans Update() :
world.Update(Time.deltaTime);
// affichage : un GameObject avec CustomCube + BodyView (BodyView.Body = body)
```

**Un tissu / des ressorts**
```csharp
var cloth = new ParticleGroup();
int a = cloth.Add(new Particle(new Vector3(0, 5, 0), 0f));      // masse 0 = particule épinglée
int b = cloth.Add(new Particle(new Vector3(1, 5, 0), 0.1f));
cloth.PairForces.Add(new SpringForce(a, b, 200f, 1f, 2f));      // raideur, longueur au repos, amortissement
world.AddParticleGroup(cloth);                                  // reçoit la gravité globale
```

**Un rayon**
```csharp
var ray = new Ray3(origine, direction);
float t; Vector3 p;
if (Intersections.RayTriangle(ray, triangle, out t, out p)) { /* touché en p */ }
```

## 5. Comment étendre
- **Nouvelle force** : classe qui implémente `IForce` (une particule/un corps) ou `IPairForce` (liens entre particules).
- **Nouvelle forme** : classe qui hérite de `Shape` (sommets, triangles, tenseur d'inertie).
- **Nouveau détecteur de collisions** : implémente `ICollisionDetector` ; **résolveur** : `ICollisionResolver`.
  Les brancher avec `world.Detector = ...` et `world.Resolver = ...`.

## 6. Correspondance avec la grille d'évaluation
Conception du moteur -> interfaces + `PhysicsWorld` · Géométrie -> `Shape`, `Mat3`, `Quat`, `Geometry/` ·
Forces -> `Dynamics/` · Collisions -> `Collision/` · Rotation et moments -> `RigidBody` (tau = r ^ F) ·
Inertie -> `Shape.InertiaBody` + `InertiaWorld` · Performances -> pas fixe, buffers réutilisés (à mesurer).

## 7. À faire (après le cours de la semaine prochaine)
- `ICollisionResolver` : réponse aux collisions (aujourd'hui seule la **détection point-plan** existe,
  les corps traversent donc le sol dans la démo).
- Détection corps-corps (sommets contre faces, arêtes), puis accélération (grille spatiale) pour beaucoup d'objets.
- Empilement stable (ex : *Dense stacking*), tissu, fumée : chacun dans son propre dossier de scène, en réutilisant la bibliothèque.
