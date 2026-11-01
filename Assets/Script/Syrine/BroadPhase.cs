using System.Collections.Generic;
using UnityEngine;

namespace PhysicsLib
{
    /// <summary>Boîte englobante alignée sur les axes (AABB) : 6 valeurs, min et max sur chaque axe (Chapitre 4).</summary>
    public struct AABB
    {
        public Vector3 min, max;

        public bool Overlaps(AABB o)
        {
            return min.x <= o.max.x && max.x >= o.min.x &&
                   min.y <= o.max.y && max.y >= o.min.y &&
                   min.z <= o.max.z && max.z >= o.min.z;
        }

        /// <summary>AABB d'un corps. Pour une boîte tournée : demi-étendue_i = somme_k |R_ik| . h_k.</summary>
        public static AABB OfBody(RigidBody b, float margin)
        {
            Vector3 ext;
            BoxShape box = b.Shape as BoxShape;
            if (box != null)
            {
                Mat3 R = b.State.R;
                Vector3 h = box.Size * 0.5f;
                ext = new Vector3(
                    Mathf.Abs(R.c0.x) * h.x + Mathf.Abs(R.c1.x) * h.y + Mathf.Abs(R.c2.x) * h.z,
                    Mathf.Abs(R.c0.y) * h.x + Mathf.Abs(R.c1.y) * h.y + Mathf.Abs(R.c2.y) * h.z,
                    Mathf.Abs(R.c0.z) * h.x + Mathf.Abs(R.c1.z) * h.y + Mathf.Abs(R.c2.z) * h.z);
            }
            else
            {
                float r = b.Shape.RenderScale.magnitude * 0.5f;       // repli : sphère englobante
                ext = new Vector3(r, r, r);
            }
            ext += new Vector3(margin, margin, margin);
            AABB a;
            a.min = b.Position - ext;
            a.max = b.Position + ext;
            return a;
        }
    }

    /// <summary>
    /// Phase large : trouve rapidement les paires de corps dont les volumes englobants se chevauchent.
    /// Chaque algorithme est interchangeable (Chapitre 4, "synthèse : la boucle de collision du moteur").
    /// "pairs" reçoit les indices à plat : i0, j0, i1, j1... avec i &lt; j.
    /// </summary>
    public interface IBroadPhase
    {
        void FindPairs(List<RigidBody> bodies, List<int> pairs);
        int TestCount { get; }          // nombre de tests de recouvrement effectués (pour comparer les méthodes)
    }

    /// <summary>Calcul commun des AABB.</summary>
    public abstract class BroadPhaseBase : IBroadPhase
    {
        public float Margin = 0.02f;
        public int TestCount { get; protected set; }
        protected AABB[] boxes = new AABB[0];

        protected void ComputeBoxes(List<RigidBody> bodies)
        {
            int n = bodies.Count;
            if (boxes.Length != n) boxes = new AABB[n];
            for (int i = 0; i < n; i++) boxes[i] = AABB.OfBody(bodies[i], Margin);
        }

        protected static bool BothStatic(List<RigidBody> bodies, int i, int j) { return bodies[i].IsStatic && bodies[j].IsStatic; }

        public abstract void FindPairs(List<RigidBody> bodies, List<int> pairs);
    }

    /// <summary>Force brute : toutes les paires (n²-n)/2, sans auto-test ni paire répétée.</summary>
    public class BruteForceBroadPhase : BroadPhaseBase
    {
        public override void FindPairs(List<RigidBody> bodies, List<int> pairs)
        {
            ComputeBoxes(bodies);
            TestCount = 0;
            int n = bodies.Count;
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++)
                {
                    if (BothStatic(bodies, i, j)) continue;
                    TestCount++;
                    if (boxes[i].Overlaps(boxes[j])) { pairs.Add(i); pairs.Add(j); }
                }
        }
    }

    /// <summary>
    /// Sweep and Prune (slides 57 à 61 et script du prof) :
    ///  1) liste des événements : début b_i et fin e_i de l'intervalle de chaque AABB sur l'axe X ;
    ///  2) tri de la liste (à égalité, le début passe avant la fin : deux intervalles qui se touchent se chevauchent) ;
    ///  3) balayage croissant avec la liste des intervalles ACTIFS :
    ///       - début de i : i chevauche tous les intervalles actifs sur X ; on vérifie Y et Z (cas 3D), puis i devient actif ;
    ///       - fin de i   : i sort de la liste active.
    /// Le tri est conservé d'une frame à l'autre (tri par insertion : presque trié = presque gratuit).
    /// </summary>
    public class SweepAndPruneBroadPhase : BroadPhaseBase
    {
        int[] events = new int[0];                 // événement e : corps = e >> 1 ; (e & 1) = 0 début, 1 fin
        readonly List<int> active = new List<int>();

        float Value(int e)
        {
            AABB b = boxes[e >> 1];
            return (e & 1) == 0 ? b.min.x : b.max.x;
        }

        bool Before(int e1, int e2)
        {
            float v1 = Value(e1), v2 = Value(e2);
            if (v1 != v2) return v1 < v2;
            return (e1 & 1) == 0 && (e2 & 1) == 1;       // à égalité : début avant fin
        }

        public override void FindPairs(List<RigidBody> bodies, List<int> pairs)
        {
            ComputeBoxes(bodies);
            TestCount = 0;
            int n = bodies.Count;
            if (events.Length != 2 * n)
            {
                events = new int[2 * n];
                for (int i = 0; i < events.Length; i++) events[i] = i;
            }

            // tri par insertion des événements
            for (int a = 1; a < events.Length; a++)
            {
                int key = events[a];
                int b = a - 1;
                while (b >= 0 && Before(key, events[b])) { events[b + 1] = events[b]; b--; }
                events[b + 1] = key;
            }

            // balayage avec la liste active
            active.Clear();
            for (int k = 0; k < events.Length; k++)
            {
                int e = events[k];
                int i = e >> 1;
                if ((e & 1) == 0)                          // début de l'intervalle i
                {
                    for (int m = 0; m < active.Count; m++)
                    {
                        int j = active[m];
                        if (BothStatic(bodies, i, j)) continue;
                        TestCount++;
                        if (boxes[i].min.y <= boxes[j].max.y && boxes[i].max.y >= boxes[j].min.y &&
                            boxes[i].min.z <= boxes[j].max.z && boxes[i].max.z >= boxes[j].min.z)
                        {
                            if (i < j) { pairs.Add(i); pairs.Add(j); } else { pairs.Add(j); pairs.Add(i); }
                        }
                    }
                    active.Add(i);
                }
                else active.Remove(i);                     // fin : l'intervalle quitte la liste active
            }
        }
    }

    /// <summary>
    /// Grille uniforme (spatial hashing) : chaque corps est inscrit dans les cellules que touche son AABB ;
    /// seuls les corps d'une même cellule sont testés. Cellule = taille du plus gros objet.
    /// </summary>
    public class GridBroadPhase : BroadPhaseBase
    {
        public float CellSize = 0f;                              // 0 : automatique (plus grande dimension d'AABB)

        readonly Dictionary<long, List<int>> grid = new Dictionary<long, List<int>>();
        readonly HashSet<long> seen = new HashSet<long>();

        static long Key(int x, int y, int z)
        {
            return ((long)(x & 0x1FFFFF) << 42) | ((long)(y & 0x1FFFFF) << 21) | (long)(z & 0x1FFFFF);
        }

        public override void FindPairs(List<RigidBody> bodies, List<int> pairs)
        {
            ComputeBoxes(bodies);
            TestCount = 0;
            int n = bodies.Count;

            float s = CellSize;
            if (s <= 0f)
            {
                for (int i = 0; i < n; i++)
                {
                    Vector3 e = boxes[i].max - boxes[i].min;
                    s = Mathf.Max(s, Mathf.Max(e.x, Mathf.Max(e.y, e.z)));
                }
                s = Mathf.Max(s, 0.1f);
            }

            foreach (List<int> cell in grid.Values) cell.Clear();   // listes vidées mais conservées (pas d'allocation)
            seen.Clear();

            for (int i = 0; i < n; i++)
            {
                int x0 = Mathf.FloorToInt(boxes[i].min.x / s), x1 = Mathf.FloorToInt(boxes[i].max.x / s);
                int y0 = Mathf.FloorToInt(boxes[i].min.y / s), y1 = Mathf.FloorToInt(boxes[i].max.y / s);
                int z0 = Mathf.FloorToInt(boxes[i].min.z / s), z1 = Mathf.FloorToInt(boxes[i].max.z / s);
                for (int x = x0; x <= x1; x++)
                    for (int y = y0; y <= y1; y++)
                        for (int z = z0; z <= z1; z++)
                        {
                            long k = Key(x, y, z);
                            List<int> list;
                            if (!grid.TryGetValue(k, out list)) { list = new List<int>(); grid[k] = list; }
                            list.Add(i);
                        }
            }

            foreach (List<int> cell in grid.Values)
                for (int a = 0; a < cell.Count; a++)
                    for (int b = a + 1; b < cell.Count; b++)
                    {
                        int i = cell[a], j = cell[b];
                        if (i > j) { int t = i; i = j; j = t; }
                        long pk = (long)i * n + j;
                        if (!seen.Add(pk)) continue;                  // une seule fois par paire
                        if (BothStatic(bodies, i, j)) continue;
                        TestCount++;
                        if (boxes[i].Overlaps(boxes[j])) { pairs.Add(i); pairs.Add(j); }
                    }
        }
    }
}
