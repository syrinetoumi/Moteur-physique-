using System;
using UnityEngine;

namespace PhysicsLib
{
    public enum IntegrationMethod { Euler, RK4 }

    /// <summary>Calcule les accélérations a(x, v) pour toutes les particules.</summary>
    public delegate void AccelerationFunc(Vector3[] x, Vector3[] v, Vector3[] aOut);

    /// <summary>Intègre un système de N points matériels sur un pas dt (modifie x et v).</summary>
    public interface IIntegrator
    {
        void Step(Vector3[] x, Vector3[] v, float dt, AccelerationFunc accel);
    }

    /// <summary>
    /// Euler (comme le code du cours) : v(t+dt) = v + a.dt puis p(t+dt) = p + v(t+dt).dt
    /// </summary>
    public class EulerIntegrator : IIntegrator
    {
        Vector3[] a = new Vector3[0];

        public void Step(Vector3[] x, Vector3[] v, float dt, AccelerationFunc accel)
        {
            if (a.Length != x.Length) a = new Vector3[x.Length];
            accel(x, v, a);
            for (int i = 0; i < x.Length; i++)
            {
                v[i] += a[i] * dt;
                x[i] += v[i] * dt;
            }
        }
    }

    /// <summary>
    /// Runge-Kutta d'ordre 4 sur le système (x' = v, v' = a(x, v)).
    /// y(t+h) = y(t) + h/6 (k1 + 2 k2 + 2 k3 + k4)
    /// </summary>
    public class RK4Integrator : IIntegrator
    {
        Vector3[] xt = new Vector3[0], vt = new Vector3[0];
        Vector3[] kx1, kx2, kx3, kx4, kv1, kv2, kv3, kv4;

        void Resize(int n)
        {
            if (xt.Length == n) return;
            xt = new Vector3[n]; vt = new Vector3[n];
            kx1 = new Vector3[n]; kx2 = new Vector3[n]; kx3 = new Vector3[n]; kx4 = new Vector3[n];
            kv1 = new Vector3[n]; kv2 = new Vector3[n]; kv3 = new Vector3[n]; kv4 = new Vector3[n];
        }

        public void Step(Vector3[] x, Vector3[] v, float h, AccelerationFunc accel)
        {
            int n = x.Length;
            Resize(n);

            // k1
            accel(x, v, kv1);
            for (int i = 0; i < n; i++) kx1[i] = v[i];

            // k2
            for (int i = 0; i < n; i++) { xt[i] = x[i] + kx1[i] * (h * 0.5f); vt[i] = v[i] + kv1[i] * (h * 0.5f); }
            accel(xt, vt, kv2);
            for (int i = 0; i < n; i++) kx2[i] = vt[i];

            // k3
            for (int i = 0; i < n; i++) { xt[i] = x[i] + kx2[i] * (h * 0.5f); vt[i] = v[i] + kv2[i] * (h * 0.5f); }
            accel(xt, vt, kv3);
            for (int i = 0; i < n; i++) kx3[i] = vt[i];

            // k4
            for (int i = 0; i < n; i++) { xt[i] = x[i] + kx3[i] * h; vt[i] = v[i] + kv3[i] * h; }
            accel(xt, vt, kv4);
            for (int i = 0; i < n; i++) kx4[i] = vt[i];

            for (int i = 0; i < n; i++)
            {
                x[i] += (kx1[i] + 2f * kx2[i] + 2f * kx3[i] + kx4[i]) * (h / 6f);
                v[i] += (kv1[i] + 2f * kv2[i] + 2f * kv3[i] + kv4[i]) * (h / 6f);
            }
        }
    }

    /// <summary>Résolution d'une EDO du premier ordre y' = f(y, t) (pour les exercices du cours et les tests).</summary>
    public static class Ode
    {
        public static Vector3 EulerStep(Func<Vector3, float, Vector3> f, Vector3 y, float t, float h)
            => y + h * f(y, t);

        public static Vector3 RK4Step(Func<Vector3, float, Vector3> f, Vector3 y, float t, float h)
        {
            Vector3 k1 = f(y, t);
            Vector3 k2 = f(y + (h * 0.5f) * k1, t + h * 0.5f);
            Vector3 k3 = f(y + (h * 0.5f) * k2, t + h * 0.5f);
            Vector3 k4 = f(y + h * k3, t + h);
            return y + (h / 6f) * (k1 + 2f * k2 + 2f * k3 + k4);
        }
    }
}
