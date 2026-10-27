using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.AI;
using UnityEditor.SceneManagement;

public class PlacementTests
{
    private Type sampler;
    private NavMeshDataInstance navigation;
    [SetUp] public void SetUp()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        sampler = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("SEAN.Tasks.RandomTaskPlacement"))
            .First(t => t != null);
    }
    [TearDown] public void TearDown()
    {
        if (navigation.valid) navigation.Remove();
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
    }
    private void Bake(params Vector3[] centers)
    {
        var sources = centers.Select(center => new NavMeshBuildSource {
            shape = NavMeshBuildSourceShape.Box,
            size = new Vector3(centers.Length == 1 ? 20 : 4, .1f, centers.Length == 1 ? 20 : 4),
            transform = Matrix4x4.TRS(center - Vector3.up * .05f, Quaternion.identity, Vector3.one), area = 0
        }).ToList();
        NavMeshData data = NavMeshBuilder.BuildNavMeshData(NavMesh.GetSettingsByID(0), sources,
            new Bounds(Vector3.zero, new Vector3(60, 10, 60)), Vector3.zero, Quaternion.identity);
        Assert.That(data, Is.Not.Null);
        navigation = NavMesh.AddNavMeshData(data);
    }
    private bool Pair(out Vector3 start, out Vector3 goal)
    {
        object[] args = { null, null, null };
        bool found = (bool)sampler.GetMethod("TryPair").Invoke(null, args);
        start = (Vector3)args[1]; goal = (Vector3)args[2];
        return found;
    }
    [Test] public void WarehouseRejectsPreviouslyBlockedStartAndSamplesClearPairs()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/SEAN/Warehouse.unity");
        Physics.SyncTransforms();
        Assert.That((bool)sampler.GetMethod("IsClear").Invoke(null,
            new object[] { new Vector3(7.7116866f, .051449f, 1.6128832f), null }), Is.False);
        foreach (int seed in new[] { 0, 1, 7, 42, 12345 })
        {
            UnityEngine.Random.InitState(seed);
            for (int i = 0; i < 20; i++) Assert.That(Pair(out _, out _), Is.True);
        }
    }
    [Test] public void MapConstraintRejectsOccupiedAndOutOfBoundsFootprints()
    {
        Type mapType = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("SEAN.Tasks.PlacementMap"))
            .First(t => t != null);
        object map = Activator.CreateInstance(mapType);
        mapType.GetField("width").SetValue(map, 20);
        mapType.GetField("height").SetValue(map, 20);
        mapType.GetField("resolution").SetValue(map, 1f);
        byte[] cells = Enumerable.Repeat((byte)255, 50).ToArray();
        int occupied = 10*20+10;
        cells[occupied/8] &= (byte)~(1 << (occupied%8));
        mapType.GetField("freeCells").SetValue(map, Convert.ToBase64String(cells));
        MethodInfo free = mapType.GetMethod("IsFree");
        Assert.That((bool)free.Invoke(map, new object[] { 5f, 5f }), Is.True);
        Assert.That((bool)free.Invoke(map, new object[] { 10f, 10f }), Is.False);
        Assert.That((bool)free.Invoke(map, new object[] { 9f, 10f }), Is.False);
        Assert.That((bool)free.Invoke(map, new object[] { -.01f, 5f }), Is.False);
        Assert.That((bool)free.Invoke(map, new object[] { 19.9f, 5f }), Is.False);
    }
    [Test] public void MissingNavigationFailsWithoutAnOriginFallback()
    {
        Assert.That(Pair(out Vector3 start, out Vector3 goal), Is.False);
        Assert.That(start, Is.EqualTo(Vector3.zero));
        Assert.That(goal, Is.EqualTo(Vector3.zero));
    }
    [Test] public void ColliderAbsentFromBakeRejectsPlacement()
    {
        Bake(Vector3.zero);
        var obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.transform.position = Vector3.up;
        obstacle.transform.localScale = new Vector3(50, 2, 50);
        Assert.That(Pair(out _, out _), Is.False);
    }
    [Test] public void DisconnectedIslandsCannotProduceATask()
    {
        Bake(Vector3.left * 10, Vector3.right * 10);
        Assert.That(Pair(out _, out _), Is.False,
            "Each island is too small for a three metre task; cross-island pairs must fail");
    }
    [Test] public void ElevatedSurfaceIsPreservedAndSamplingRepeatsForTheSameSeed()
    {
        Bake(Vector3.up * 2);
        UnityEngine.Random.InitState(7);
        Assert.That(Pair(out Vector3 start, out Vector3 goal), Is.True);
        Assert.That(start.y, Is.EqualTo(2).Within(.2f));
        Assert.That(goal.y, Is.EqualTo(2).Within(.2f));
        Assert.That(Vector3.Distance(start, goal), Is.GreaterThanOrEqualTo(3));
        UnityEngine.Random.InitState(7);
        Assert.That(Pair(out Vector3 again, out Vector3 againGoal), Is.True);
        Assert.That(again, Is.EqualTo(start));
        Assert.That(againGoal, Is.EqualTo(goal));
    }
}
