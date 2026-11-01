// Auteur : Aziz (Med ElAziz Kerkeni) - projet Moteur Physique 5GamiX 2026-2027
// Fichier PERSONNEL : il APPELLE la phase large partagee de PhysicsLib (BroadPhase.cs) sans la modifier.
using System.Collections.Generic;
using UnityEngine;
using PhysicsLib;

/// <summary>
/// Adaptateur entre les PARTICULES (tissu, fumee) et la phase large SweepAndPruneBroadPhase de la
/// bibliotheque, qui ne sait traiter que des RigidBody (Chapitre 4, slides 57 a 61).
///
/// Principe : chaque particule est representee par un petit RigidBody "fantome" (une BoxShape de cote 2 r,
/// orientation identite) dont la position est copiee a chaque appel. Le SAP de la bibliotheque calcule les
/// AABB, trie les evenements sur X et renvoie les paires dont les boites se chevauchent. L'adaptateur
/// ne garde que les paires "groupe A - groupe B" (ici tissu - fumee) et les traduit en indices de particules.
///
/// Ce que le SAP ne fait PAS : decider si deux particules se touchent vraiment (phase etroite : |xi - xj| &lt; ri + rj)
/// ni repondre au contact. Cela reste dans ParticleCollisionSolver.
///
/// Points importants :
///  - le nombre de fantomes est constant, ce qui garde le tri par insertion du SAP presque gratuit d'une
///    frame a l'autre (coherence temporelle) ;
///  - une particule morte (phase &lt; 0) est rangee tres loin sur X, espacee des autres, immobile : aucune paire ;
///  - une particule epinglee est "statique" : deux statiques ne sont jamais testees ensemble.
/// </summary>
public sealed class ParticleSAPBridge
{
    /// <summary>Le SAP de la bibliotheque (BroadPhase.cs) : on lit ici Margin et TestCount.</summary>
    public readonly SweepAndPruneBroadPhase Broad = new SweepAndPruneBroadPhase();

    public readonly int PhaseA;
    public readonly int PhaseB;

    /// <summary>Tests de recouvrement faits par le SAP au dernier appel (a comparer a BruteForceTests).</summary>
    public int LastTestCount { get; private set; }

    /// <summary>Paires tissu - fumee (boites qui se chevauchent) renvoyees au dernier appel.</summary>
    public int LastCandidatePairs { get; private set; }

    /// <summary>Tests qu'aurait faits la force brute : (A vivantes non statiques x B vivantes) environ.</summary>
    public int BruteForceTests { get; private set; }

    readonly List<Particle> particles;
    readonly List<RigidBody> proxies = new List<RigidBody>();
    readonly int[] particleIndex;      // fantome k -> indice dans la liste de particules du solveur
    readonly bool[] inGroupB;          // fantome k appartient au groupe B ?
    readonly List<int> rawPairs = new List<int>();

    public ParticleSAPBridge(List<Particle> all, float[] radii, int[] groupA, int[] groupB, int phaseA, int phaseB)
    {
        particles = all;
        PhaseA = phaseA;
        PhaseB = phaseB;

        int n = groupA.Length + groupB.Length;
        particleIndex = new int[n];
        inGroupB = new bool[n];

        // Une BoxShape est partagee par toutes les particules de meme rayon.
        Dictionary<float, BoxShape> shapes = new Dictionary<float, BoxShape>();

        int k = 0;
        for (int g = 0; g < 2; g++)
        {
            int[] group = (g == 0) ? groupA : groupB;
            for (int m = 0; m < group.Length; m++)
            {
                int index = group[m];
                float r = radii[index];

                BoxShape shape;
                if (!shapes.TryGetValue(r, out shape))
                {
                    shape = new BoxShape(new Vector3(2f * r, 2f * r, 2f * r));
                    shapes.Add(r, shape);
                }

                RigidBody proxy = new RigidBody(shape, 1f, all[index].position);
                proxies.Add(proxy);
                particleIndex[k] = index;
                inGroupB[k] = (g == 1);
                k++;
            }
        }
    }

    /// <summary>Cette paire de phases est-elle geree par le SAP (et donc a ignorer par la grille) ?</summary>
    public bool Handles(int phase1, int phase2)
    {
        return (phase1 == PhaseA && phase2 == PhaseB) || (phase1 == PhaseB && phase2 == PhaseA);
    }

    /// <summary>
    /// Appel de la phase large : AJOUTE a outPairs les paires (i, j) d'indices de particules
    /// (une du groupe A, une du groupe B) dont les AABB se chevauchent.
    /// margin : demi-marge ajoutee a chaque boite (comme ContactMargin dans la grille).
    /// </summary>
    public void FindPairs(float margin, List<int> outPairs)
    {
        Broad.Margin = margin;

        int aliveA = 0, aliveB = 0;
        for (int k = 0; k < proxies.Count; k++)
        {
            Particle p = particles[particleIndex[k]];
            RigidBody body = proxies[k];

            if (p.phase < 0)
            {
                // Particule morte : parquee loin a gauche, immobile, distincte des autres.
                body.State.x = new Vector3(-1000f - 2f * k, 0f, 0f);
                body.IsStatic = true;
            }
            else
            {
                body.State.x = p.position;
                body.IsStatic = p.IsFixed;
                if (inGroupB[k]) aliveB++; else aliveA++;
            }
        }

        rawPairs.Clear();
        Broad.FindPairs(proxies, rawPairs);          // <-- le SAP de la bibliotheque
        LastTestCount = Broad.TestCount;
        BruteForceTests = aliveA * aliveB;

        int kept = 0;
        for (int m = 0; m < rawPairs.Count; m += 2)
        {
            int a = rawPairs[m], b = rawPairs[m + 1];
            if (inGroupB[a] == inGroupB[b]) continue;     // tissu-tissu ou fumee-fumee : pas gere ici
            outPairs.Add(particleIndex[a]);
            outPairs.Add(particleIndex[b]);
            kept++;
        }
        LastCandidatePairs = kept;
    }
}
