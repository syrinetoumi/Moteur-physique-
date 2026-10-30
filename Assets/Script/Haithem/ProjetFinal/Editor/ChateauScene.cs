using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Haithem.ProjetFinal
{
    public static class ChateauScene
    {
        public const string Path = "Assets/Scene/Haithem/ProjetFinal/ChateauParticules.unity";

        [MenuItem("Haithem/Projet final/Créer la scène")]
        public static void Creer()
        {
            if (EditorApplication.isPlaying || File.Exists(Path)) return;
            Directory.CreateDirectory(System.IO.Path.GetDirectoryName(Path));
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
            SceneManager.SetActiveScene(scene);

            var root = new GameObject("Château");
            var demo = root.AddComponent<ChateauParticules>();
            var serialized = new SerializedObject(demo);
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            serialized.FindProperty("sphere").objectReferenceValue = sphere.GetComponent<MeshFilter>().sharedMesh;
            serialized.FindProperty("materiau").objectReferenceValue = sphere.GetComponent<MeshRenderer>().sharedMaterial;
            Object.DestroyImmediate(sphere);
            serialized.ApplyModifiedPropertiesWithoutUndo();
            demo.Reinitialiser();

            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "Plane";
            floor.transform.localScale = new Vector3(10f, 1f, 10f);
            Object.DestroyImmediate(floor.GetComponent<Collider>());

            var cameraObject = new GameObject("Main Camera", typeof(Camera), typeof(AudioListener));
            cameraObject.tag = "MainCamera";
            var camera = cameraObject.GetComponent<Camera>();
            camera.fieldOfView = 52f;
            camera.farClipPlane = 150f;
            cameraObject.transform.position = new Vector3(13f, 11f, -19f);
            cameraObject.transform.LookAt(new Vector3(0f, 1.5f, -1.5f));

            var lightObject = new GameObject("Directional Light", typeof(Light));
            var light = lightObject.GetComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;
            light.shadows = LightShadows.Soft;
            lightObject.transform.rotation = Quaternion.Euler(45f, -35f, 0f);
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.65f, 0.65f, 0.65f);
            EditorSceneManager.SaveScene(scene, Path);
            Selection.activeGameObject = root;
        }
    }
}
