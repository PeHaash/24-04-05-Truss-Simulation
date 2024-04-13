using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Remoting.Services;
using System.Text;
using System.Threading.Tasks;

using Rhino;
using Rhino.Geometry;

/*using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;*/


/*
 * Here are the rules:
 * We have only one class: Truss, for now
 * Truss keeps both datas: uncompiled and compiled
 * because, if we want to do some optimizations, it is better to just edit the compiled data, not copy it every time.
 * so yes, we don't have a builder.
 * However, later it might be useful to Have multiple trusses: for example if we don't want to have un-uniform masses
 * but for now, only Liz.Truss
 * and yes, it get copied all over the data set in the grasshopper. pfff
 * and we add damper, free force, etc to our code first. Then we go for the GPU.
*/

namespace Liz
{
    internal class Default
    {
        internal const double Mass  = 10;
        internal const double Stiffness = 500;
        internal const double DeltaTime = 0.01;
        internal const int MaxStep = 100;
        internal const double DamperConstant  = 20;
        internal static readonly Vector3d ZeroVector = new Vector3d(0, 0, 0);
        internal static readonly Point3d ZeroPoint  = new Point3d(0, 0, 0);
    }
    public class DisjointSet
    {
        private readonly int[] Par;
        //private int NumberOfSets;
        //private int NumberOfElements;

        public DisjointSet(int n)
        {
            Par = new int[n];
            for (int i = 0; i < n; i++) Par[i] = -1;
            // NumberOfSets = n;
            // NumberOfElements = n;
        }
        public int FindParent(int p)
        {
            if (Par[p] == -1) return p;
            return Par[p] = FindParent(Par[p]);
        }
        public void Join(int a, int b)
        {
            if (FindParent(a) != FindParent(b))
            {
                Par[FindParent(a)] = FindParent(b);
                // NumberOfSets--;
            }
        }
        public bool IsRoot(int a)
        {
            return Par[a] == -1;
        }

    }
    public struct Triple
    {
        public double x, y, z;
        public Triple(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        internal Triple(Point3d p)
        {
            x = p.X; y = p.Y; z = p.Z;
        }
        internal Triple(Vector3d v)
        {
            x = v.X; y = v.Y; z = v.Z;
        }
        internal Triple(Triple start, Triple end, double len)
        {
            // make a vector, from start to end with a set len
            x = end.x - start.x; y = end.y - start.y; z = end.z - start.z;
            double t = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2)); // now_len, here!
            //if (t == 0) it will crash, but we shouldn't have zero len here!
            t = len / t; // to adjust the length
            x *= t; y *= t; z *= t;
        }
        internal double Len()
        {
            return Math.Sqrt(x*x + y*y + z*z);
        }

        internal static double Distance(Triple a, Triple b)
        {
            return Math.Sqrt(Math.Pow(a.x - b.x, 2) + Math.Pow(a.y - b.y, 2) + Math.Pow(a.z - b.z, 2));
        }
        public static Triple operator +(Triple a, Triple b)
        {
            return new Triple(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        public static Triple operator -(Triple a, Triple b)
        {
            return new Triple(a.x - b.x, a.y - b.y, a.z - b.z);
        }
        public static Triple operator *(Triple a, double b)
        {
            return new Triple(a.x * b, a.y * b, a.z * b);
        }
        
    } 
    public struct Node {
        public Triple Pos0 { internal set; get; }
        public Triple Position { internal set; get; }
        public Triple Velocity { internal set; get; }
        public double OneOverMass { internal set; get; }
        public Triple ConstantForce { internal set; get; } // only on some nodes!
        public Triple Force { internal set; get; }
        public int SupportType { internal set; get; }
        public Triple ReactionForce { internal set; get; } // only on some nodes!
        internal Node(Point3d p0, Vector3d force, double mass, int support_type)
        {
            Pos0 = new Triple(p0);
            Position = new Triple(p0);
            Velocity = new Triple();
            OneOverMass = 1/mass;
            ConstantForce = new Triple(force);
            Force = new Triple();
            SupportType = support_type;
            ReactionForce = new Triple();
        }
        internal void UpdateReactionForce()
        {
            Triple nReactionForce = new Triple(
                ((SupportType & 4) != 0) ? -Force.x: 0,
                ((SupportType & 2) != 0) ? -Force.y : 0,
                ((SupportType & 1) != 0) ? -Force.z : 0
                );
            ReactionForce = nReactionForce;
            //Force += ReactionForce; kinda side-effect, cleaned it
        }
    }
    public struct Beam {
        public int StartNode { internal set; get; }
        public int EndNode { internal set; get; }
        public double InitialLength { internal set; get; }
        public double SpringConstant { internal set; get; }
        public double InternalForce { internal set; get; } // +: compression beam: push nodes, -: tension beam: pull nodes
        internal Beam(int startnode, int endnode, double initlen, double sprconst)
        {
            StartNode = startnode;
            EndNode = endnode;
            InitialLength = initlen;
            SpringConstant = sprconst;
            InternalForce = 0;
        }
    }
    public struct ProtoNode
    {
        public Point3d Pos0 { internal set; get; } // shayad beshe inam public set kard. 
        public Vector3d Force { internal set; get; }
        public int SupportType { internal set; get; }
        public double Mass { set; get; }
        public ProtoNode(Point3d P0)
        {
            Pos0 = P0;
            Force = new Vector3d(0, 0, 0);
            SupportType = 0;
            Mass = Default.Mass;
        }
        internal Node ToNode()
        {
            return new Node(Pos0, Force, Mass, SupportType);
        }
    }
    public struct ProtoBeam
    {
        public Tuple<int, int> Link { internal set; get; }
        public double Stiffness { set; get; } // set is also OK man!!
        public double Length { internal set; get; }
        public ProtoBeam(int a, int b, double initial_lenght, double stiff = Default.Stiffness)
        {
            Link = new Tuple<int, int>(a, b);
            Stiffness = stiff;
            Length = initial_lenght;
        }
        internal Beam ToBeam()
        {
            return new Beam(Link.Item1, Link.Item2, Length, Stiffness);
        }
    }
    public class Truss
    {
        // general data:
        public double DeltaTime { set; get; } = Default.DeltaTime;
        private int Iteration = 0;
        public int MaxStep { set; get; } = Default.MaxStep;
        private int NodeCount;
        private int BeamCount;
        public double DamperConstant { set; get; } = Default.DamperConstant;

        // compiled data: this things are on the GPU!!
        public Node[] Nodes { private set; get; }
        public Beam[] Beams { private set; get; }
        private int[] ForcedNodesIndexes;
        private int[] SupportedNodesIndexes;

        // uncompiled data: This things are on the CPU! 
        public List<ProtoNode> ProtoNodes { private set; get; }
        public List<ProtoBeam> ProtoBeams { private set; get; }

        // constructors:
        public Truss(List<Point3d> Points, double MaxBeamLen)
        {
            // another constructor
            BeamCount = NodeCount = 0;

        }
        public Truss(List<Line> Beam_Lines, double Tolerance)
        {
            // classic constructor
            ProtoNodes = new List<ProtoNode>();
            ProtoBeams = new List<ProtoBeam>();
            BeamCount = NodeCount = 0;

            List<Point3d> points = new List<Point3d>();
            DisjointSet dst = new DisjointSet(Beam_Lines.Count * 2);

            foreach (Line t in Beam_Lines)
            {
                points.Add(t.From);
                points.Add(t.To);
            }
            for (int i = 0; i < points.Count; i++)
            {
                for (int j = i + 1; j < points.Count; j++)
                {
                    if (dst.FindParent(i) != dst.FindParent(j) && points[i].DistanceTo(points[j]) < Tolerance)
                    {
                        dst.Join(i, j);
                    }
                }
            }
            int[] co_node = new int[points.Count]; // corresponding node for the root points
            for (int i = 0; i < points.Count; i++)
            {
                if (dst.IsRoot(i))
                {
                    ProtoNodes.Add(new ProtoNode(points[i]));
                    co_node[i] = ProtoNodes.Count - 1;
                }
            }

            for (int i = 0; i < points.Count; i += 2)
            {
                ProtoBeams.Add(new ProtoBeam(
                    co_node[dst.FindParent(i)],
                    co_node[dst.FindParent(i + 1)],
                    ProtoNodes[co_node[dst.FindParent(i)]].Pos0.DistanceTo(ProtoNodes[co_node[dst.FindParent(i + 1)]].Pos0)));
            }
        }
        public Truss(Truss other)
        {
            // copy constructor
            DeltaTime = other.DeltaTime;
            Iteration = other.Iteration;
            NodeCount = other.NodeCount;
            BeamCount = other.BeamCount;
            DamperConstant = other.DamperConstant;
            if (other.Nodes != null) Nodes = (Node[])other.Nodes.Clone();
            if (other.Beams != null) Beams = (Beam[])other.Beams.Clone();
            if (other.ForcedNodesIndexes != null) ForcedNodesIndexes = (int[])other.ForcedNodesIndexes.Clone();
            if (other.SupportedNodesIndexes != null) SupportedNodesIndexes = (int[])other.SupportedNodesIndexes.Clone();
            if (other.ProtoNodes != null) ProtoNodes = other.ProtoNodes.ToList();
            if (other.ProtoBeams != null) ProtoBeams = other.ProtoBeams.ToList();
        }

        // single features should be updated from Properties
        public void AddForce(Point3d p, Vector3d v, double strenght)
        {
            // add a Force to the nearest Node
            v.Unitize();
            // new code:
            int index = NearestNode(p);
            var node = ProtoNodes[index];
            node.Force += v * strenght;
            ProtoNodes[index] = node;
            // old code: // cannot be done bc ProtoNode is a value type, not a reference type
            // ProtoNodes[NearestNode(p)].Force += v * strenght; 
        }

        public void AddSupport(Point3d p, int type)
        {
            // add a Support to the nearest Node
            int index = NearestNode(p);
            var node = ProtoNodes[index];
            node.SupportType |= type;
            ProtoNodes[index] = node;
            // same problem as in AddForce: ProtoNodes[NearestNode(p)].SupportType|= type; is not possible   
        }


        public void Compile()
        {
            // make code ready for gpu!
            Iteration = 0;
            NodeCount = ProtoNodes.Count;
            BeamCount = ProtoBeams.Count;
            Nodes = new Node[NodeCount];
            Beams = new Beam[BeamCount];
            ForcedNodesIndexes = new int[ProtoNodes.Count(item => !item.Force.IsZero)]; // count number of non, zero forces
            SupportedNodesIndexes = new int[ProtoNodes.Count(item => item.SupportType != 0)]; // like-wise, for supports
            // this is also possible with LINQ, but I prefer to keep it simple
            int fn_po = 0, sn_po = 0;
            for(int i = 0; i < NodeCount; i++)
            {
                Nodes[i] = ProtoNodes[i].ToNode();
                if (!ProtoNodes[i].Force.IsZero) ForcedNodesIndexes[fn_po++] = i;
                if (ProtoNodes[i].SupportType != 0) SupportedNodesIndexes[sn_po++] = i;
            }
            for (int i = 0; i < BeamCount; i++) Beams[i] = ProtoBeams[i].ToBeam();
            // int[] intArray = objects.Select(obj => obj.ToInt()).ToArray(); sample in LINQ
        }

        public double Update()
        {
            double free_forces = 0;
            if (Iteration == MaxStep) return 0;
            // update beams, position is input, force is output
            for (int i = 0; i < BeamCount; i++)
            {
                double delta_len = Triple.Distance(Nodes[Beams[i].StartNode].Position, Nodes[Beams[i].EndNode].Position) 
                    - Beams[i].InitialLength;
                // deltaLen <0 : newLen is shorter --> compression --> InternalForce should be >0, vice versa
                Beams[i].InternalForce = -Beams[i].SpringConstant * delta_len;
                // vector from start to end
                Triple vector_force = new Triple(
                    Nodes[Beams[i].StartNode].Position,
                    Nodes[Beams[i].EndNode].Position,
                    Beams[i].InternalForce
                    );
                // comp -> int mosbat -> vector force mosbat be samte end;
                Nodes[Beams[i].StartNode].Force -= vector_force; /// -=, and stuff like that is saving my code:implicit copying in handled
                Nodes[Beams[i].EndNode].Force += vector_force;
            }

            
            // add the constant force to some nodes with force
            for(int i = 0; i < ForcedNodesIndexes.Length; i++)
                Nodes[ForcedNodesIndexes[i]].Force += Nodes[ForcedNodesIndexes[i]].ConstantForce;

            // enforce damper constant to all nodes
            for (int i = 0; i < NodeCount; i++)
                Nodes[i].Force -= Nodes[i].Velocity * DamperConstant;

            // make nodes with support 0 in force, put them in the ReactionForce (one of the main outputs!)
            for (int i = 0; i < SupportedNodesIndexes.Length; i++)
            {
                Nodes[SupportedNodesIndexes[i]].UpdateReactionForce();
                Nodes[SupportedNodesIndexes[i]].Force += Nodes[SupportedNodesIndexes[i]].ReactionForce;
            }

            for(int i = 0; i < NodeCount; i++)
            {
                free_forces += Nodes[i].Force.Len();
                //Acceleration[i] = Force[i] / Mass[i]; // f = m.a
                //Velocity[i] += Acceleration[i] * Delta; 
                Nodes[i].Velocity += Nodes[i].Force * Nodes[i].OneOverMass * DeltaTime;
                Nodes[i].Position += Nodes[i].Velocity * DeltaTime;
                Nodes[i].Force = new Triple(); // we are always working with the copies of structs here :/
            }
            
            Iteration++;
            return free_forces;


        }
            

        // important NOTE!!!!
        // for changing elements in an array of structs, you should do this:
        /*
            struct MyStruct
                {
                    public int Value;
                    public void Increment() => Value++;
                }

            MyStruct[] myArray = new MyStruct[1];
            myArray[0] = new MyStruct();

            // Directly modifying the struct in the array
            ref MyStruct item = ref myArray[0];  /// !!!!!
            item.Increment();
        */

        // private functions:
        private int NearestNode(Point3d p)
        {
            int min_ind = 0;
            double min_dis = p.DistanceTo(ProtoNodes[0].Pos0);
            for (int i = 1; i < ProtoNodes.Count; i++)
            {
                if (p.DistanceTo(ProtoNodes[i].Pos0) < min_dis)
                {
                    min_dis = p.DistanceTo(ProtoNodes[i].Pos0);
                    min_ind = i;
                }
            }
            return min_ind;
        }


    }
}
