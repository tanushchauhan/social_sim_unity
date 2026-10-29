using System;
using NUnit.Framework;
using UnityEngine;
using SEAN.Scenario.Controllers;

public class NavMeshControllerGeometryTests
{
    [Test] public void SharedTriangleEdgesAreRemovedAndRoomBoundaryIsClockwise()
    {
        var vertices = new[] { new Vector3(0,0,0),new Vector3(0,0,2),new Vector3(-2,0,2),new Vector3(-2,0,0) };
        var polygons = NavMeshControllerGeometry.FromTriangles(vertices,new[] { 0,1,2,0,2,3 });
        Assert.AreEqual(1,polygons.Count); Assert.AreEqual(4,polygons[0].Vertices.Count);
        float area=0; var v=polygons[0].Vertices;
        for(int i=0;i<v.Count;i++) area+=v[i].x*v[(i+1)%v.Count].y-v[(i+1)%v.Count].x*v[i].y;
        Assert.Less(area,0,"Walkable interior must be on the right of an RVO boundary");
    }
    [Test] public void DuplicateTileVerticesWeldAndStackedGeometryFailsExplicitly()
    {
        var vertices = new[] { new Vector3(0,0,0),new Vector3(0,0,2),new Vector3(-2,0,2),
            new Vector3(0,0,0),new Vector3(-2,0,2),new Vector3(-2,0,0) };
        Assert.AreEqual(4,NavMeshControllerGeometry.FromTriangles(vertices,new[] { 0,1,2,3,4,5 })[0].Vertices.Count);
        vertices[3].y=vertices[4].y=vertices[5].y=1;
        Assert.Throws<ArgumentException>(()=>NavMeshControllerGeometry.FromTriangles(vertices,new[] { 0,1,2,3,4,5 }));
    }
    [Test] public void RobotConnectedFloorIsExplicitlySelectedFromDisconnectedPlatforms()
    {
        var v=new[] {new Vector3(0,0,0),new Vector3(0,0,2),new Vector3(-2,0,2),new Vector3(-2,0,0),
            new Vector3(8,1,8),new Vector3(8,1,10),new Vector3(10,1,10)};
        var result=NavMeshControllerGeometry.FromConnectedFloor(v,new[] {0,1,2,0,2,3,4,5,6},new Vector3(-1,.05f,1));
        Assert.AreEqual(1,result.Count);Assert.AreEqual(4,result[0].Vertices.Count);
        Assert.Throws<ArgumentException>(()=>NavMeshControllerGeometry.FromConnectedFloor(v,new[] {0,1,2,0,2,3,4,5,6},new Vector3(50,0,50)));
    }
}
