using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

namespace SEAN.Scenario.Controllers
{
    // Explicit NavMeshBoundaryV1 geometry: navigable-domain boundaries, which
    // already include the bake's agent clearance. These are not physical wall
    // surfaces; adding actor radius is conservative and must be reported.
    public static class NavMeshControllerGeometry
    {
        public static IReadOnlyList<Obstacle> Capture(Vector3 anchor)
        {
            var mesh = NavMesh.CalculateTriangulation();
            return FromConnectedFloor(mesh.vertices, mesh.indices, anchor);
        }
        public static IReadOnlyList<Obstacle> FromConnectedFloor(Vector3[] vertices, int[] triangles, Vector3 anchor)
        {
            if (vertices == null || triangles == null || triangles.Length == 0 || triangles.Length%3 != 0 ||
                triangles.Any(i => i < 0 || i >= vertices.Length)) throw new ArgumentException("Invalid NavMesh triangles");
            var weld = new Dictionary<Vector3Int,int>(); var ids = new int[vertices.Length];
            for (int i=0;i<vertices.Length;i++)
            {
                var v=vertices[i];
                if (!Contract.Finite(v.x)||!Contract.Finite(v.y)||!Contract.Finite(v.z)) throw new ArgumentException("Nonfinite NavMesh vertex");
                var key=new Vector3Int(Mathf.RoundToInt(v.x*1000),Mathf.RoundToInt(v.y*1000),Mathf.RoundToInt(v.z*1000));
                if (!weld.ContainsKey(key)) weld.Add(key,weld.Count);
                ids[i]=weld[key];
            }
            var edges=new Dictionary<Tuple<int,int>,List<int>>();
            var neighbors=new List<int>[triangles.Length/3];
            int seed=-1; float heightError=float.PositiveInfinity;
            var point=Contract.FromUnity(anchor);
            for(int t=0;t<neighbors.Length;t++)
            {
                neighbors[t]=new List<int>();
                var a=Contract.FromUnity(vertices[triangles[3*t]]);
                var b=Contract.FromUnity(vertices[triangles[3*t+1]]);
                var c=Contract.FromUnity(vertices[triangles[3*t+2]]);
                Func<Vector2,Vector2,Vector2,float> side=(u,v,p)=>(v.x-u.x)*(p.y-u.y)-(v.y-u.y)*(p.x-u.x);
                float ab=side(a,b,point),bc=side(b,c,point),ca=side(c,a,point);
                bool contains=(ab>=-1e-6f&&bc>=-1e-6f&&ca>=-1e-6f)||(ab<=1e-6f&&bc<=1e-6f&&ca<=1e-6f);
                float y=(vertices[triangles[3*t]].y+vertices[triangles[3*t+1]].y+vertices[triangles[3*t+2]].y)/3;
                if(contains && Mathf.Abs(side(a,b,c))>1e-8f && Mathf.Abs(y-anchor.y)<heightError)
                { seed=t; heightError=Mathf.Abs(y-anchor.y); }
                for(int j=0;j<3;j++)
                {
                    int u=ids[triangles[3*t+j]],v=ids[triangles[3*t+(j+1)%3]];
                    var key=Tuple.Create(Math.Min(u,v),Math.Max(u,v));
                    if(!edges.ContainsKey(key)) edges.Add(key,new List<int>());
                    edges[key].Add(t);
                }
            }
            if(seed<0 || heightError>.5f) throw new ArgumentException("Robot anchor is not on a supported floor");
            foreach(var edge in edges.Values)
            {
                if(edge.Count>2) throw new ArgumentException("Nonmanifold floor connectivity");
                if(edge.Count==2) {neighbors[edge[0]].Add(edge[1]);neighbors[edge[1]].Add(edge[0]);}
            }
            var selected=new HashSet<int>(); var queue=new Queue<int>(); queue.Enqueue(seed);
            while(queue.Count>0)
            {
                int t=queue.Dequeue(); if(!selected.Add(t)) continue;
                foreach(int n in neighbors[t]) if(!selected.Contains(n)) queue.Enqueue(n);
            }
            var compact=new List<Vector3>(); var indices=new List<int>(); var map=new Dictionary<int,int>();
            foreach(int t in selected.OrderBy(t=>t)) for(int j=0;j<3;j++)
            {
                int raw=triangles[3*t+j];
                if(!map.ContainsKey(raw)) {map.Add(raw,compact.Count);compact.Add(vertices[raw]);}
                indices.Add(map[raw]);
            }
            return FromTriangles(compact.ToArray(),indices.ToArray());
        }
        public static IReadOnlyList<Obstacle> FromTriangles(Vector3[] vertices, int[] triangles)
        {
            if (vertices == null || vertices.Length < 3 || triangles == null || triangles.Length % 3 != 0 || triangles.Length == 0)
                throw new ArgumentException("A triangulated navigable floor is required");
            if (vertices.Any(v => !Contract.Finite(v.x) || !Contract.Finite(v.y) || !Contract.Finite(v.z)))
                throw new ArgumentException("Nonfinite floor geometry");
            var canonical = new Dictionary<Vector2Int, int>();
            var heights = new Dictionary<Vector2Int, float>();
            var points = new List<Vector2>(); var remap = new int[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                var p = Contract.FromUnity(vertices[i]);
                var key = new Vector2Int(Mathf.RoundToInt(p.x * 1000), Mathf.RoundToInt(p.y * 1000));
                int index;
                if (!canonical.TryGetValue(key, out index))
                {
                    index = points.Count; canonical.Add(key, index); points.Add(p);
                    heights.Add(key, vertices[i].y);
                }
                else if (Mathf.Abs(heights[key]-vertices[i].y) > .002f)
                    throw new ArgumentException("Overlapping floor levels cannot share a planar controller frame");
                remap[i] = index;
            }
            var counts = new Dictionary<Tuple<int,int>, int>();
            var directed = new Dictionary<Tuple<int,int>, Tuple<int,int>>();
            var planarTriangles = new List<int[]>();
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int[] ids = new int[3];
                for (int j = 0; j < 3; j++)
                {
                    int raw = triangles[i+j];
                    if (raw < 0 || raw >= vertices.Length) throw new ArgumentException("Invalid NavMesh triangle index");
                    ids[j] = remap[raw];
                }
                var ab = points[ids[1]]-points[ids[0]]; var ac = points[ids[2]]-points[ids[0]];
                float cross = ab.x*ac.y-ab.y*ac.x;
                if (Mathf.Abs(cross) < 1e-8f) continue;
                planarTriangles.Add(ids);
                for (int j = 0; j < 3; j++)
                {
                    int a = ids[j], b = ids[(j+1)%3];
                    var key = Tuple.Create(Math.Min(a,b), Math.Max(a,b));
                    counts[key] = counts.ContainsKey(key) ? counts[key]+1 : 1;
                    // RVO obstacles have solid space to their left. Orient each
                    // boundary so the triangle's walkable interior is right.
                    directed[key] = cross > 0 ? Tuple.Create(b,a) : Tuple.Create(a,b);
                }
            }
            // A ramp is a valid 2.5D floor. A folded/stacked surface whose
            // horizontal triangles overlap is not; reject it before solving.
            for (int i=0;i<planarTriangles.Count;i++) for (int j=i+1;j<planarTriangles.Count;j++)
            {
                var a=planarTriangles[i].Select(n=>points[n]).ToArray();
                var b=planarTriangles[j].Select(n=>points[n]).ToArray();
                if (TrianglesOverlap(a,b)) throw new ArgumentException("Floor triangles overlap in the planar controller frame");
            }
            if (counts.Values.Any(n => n > 2)) throw new ArgumentException("Nonmanifold NavMesh edge");
            var next = new Dictionary<int,int>();
            foreach (var pair in counts.Where(p => p.Value == 1))
            {
                var edge = directed[pair.Key];
                if (next.ContainsKey(edge.Item1)) throw new ArgumentException("Ambiguous NavMesh boundary junction");
                next.Add(edge.Item1, edge.Item2);
            }
            var output = new List<Obstacle>();
            while (next.Count > 0)
            {
                int start = next.Keys.Min(), cursor = start; var loop = new List<Vector2>();
                do
                {
                    int successor;
                    if (!next.TryGetValue(cursor,out successor)) throw new ArgumentException("Open NavMesh boundary");
                    loop.Add(points[cursor]); next.Remove(cursor); cursor = successor;
                } while (cursor != start);
                output.Add(new Obstacle("navmesh-boundary-" + output.Count, loop));
            }
            if (output.Count == 0) throw new ArgumentException("No NavMesh boundary polygons");
            return output.AsReadOnly();
        }
        static float Cross(Vector2 a, Vector2 b, Vector2 c)
        {
            return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
        }
        static bool Inside(Vector2 p, Vector2[] triangle)
        {
            float a=Cross(triangle[0],triangle[1],p),b=Cross(triangle[1],triangle[2],p),c=Cross(triangle[2],triangle[0],p);
            return (a>1e-6f&&b>1e-6f&&c>1e-6f)||(a<-1e-6f&&b<-1e-6f&&c<-1e-6f);
        }
        static bool TrianglesOverlap(Vector2[] a, Vector2[] b)
        {
            if(a.Any(p=>Inside(p,b))||b.Any(p=>Inside(p,a))) return true;
            for(int i=0;i<3;i++) for(int j=0;j<3;j++)
            {
                float ab0=Cross(a[i],a[(i+1)%3],b[j]),ab1=Cross(a[i],a[(i+1)%3],b[(j+1)%3]);
                float ba0=Cross(b[j],b[(j+1)%3],a[i]),ba1=Cross(b[j],b[(j+1)%3],a[(i+1)%3]);
                if(ab0*ab1<-1e-10f&&ba0*ba1<-1e-10f) return true;
            }
            return false;
        }
    }
}
