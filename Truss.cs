using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Remoting.Services;
using System.Text;
using System.Threading.Tasks;

using Rhino;
using Rhino.Geometry;

using ILGPU;
using ILGPU.Algorithms;
using ILGPU.Runtime;
using ILGPU.IR.Types;
using ILGPU.Runtime.OpenCL;
using ILGPU.Runtime.Cuda;
using ILGPU.Runtime.CPU;
using System.Security.Cryptography;
using Old;
using System.Numerics;

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

namespace Old
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
    public struct fTriple
    {
        public float x, y, z;
        public fTriple(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        internal fTriple(Point3d p)
        {
            x = (float)p.X; y = (float)p.Y; z = (float)p.Z;
        }
        internal fTriple(Vector3d v)
        {
            x = (float)v.X; y = (float)v.Y; z = (float)v.Z;
        }
        internal fTriple(fTriple start, fTriple end, float len)
        {
            // make a vector, from start to end with a set len
            x = end.x - start.x; y = end.y - start.y; z = end.z - start.z;
            float t = (float)XMath.Sqrt(x*x + y*y + z*z); // now_len, here!
            //if (t == 0) it will crash, but we shouldn't have zero len here!
            t = len / t; // to adjust the length
            x *= t; y *= t; z *= t;
            
        }
        internal float Len()
        {
            //return XMath.Sqrt(x * x + y * y + z * z);
            return XMath.Sqrt(x * x + y * y + z * z);
        }

        internal static float Distance(fTriple a, fTriple b)
        {
            //return XMath.Sqrt(XMath.Pow(a.x - b.x, 2) + XMath.Pow(a.y - b.y, 2) + XMath.Pow(a.z - b.z, 2));
            return XMath.Sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
        }
        public static fTriple operator +(fTriple a, fTriple b)
        {
            return new fTriple(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        public static fTriple operator -(fTriple a, fTriple b)
        {
            return new fTriple(a.x - b.x, a.y - b.y, a.z - b.z);
        }
        public static fTriple operator *(fTriple a, float b)
        {
            return new fTriple(a.x * b, a.y * b, a.z * b);
        }
        public Triple ToTriple()
        {
            return new Triple((double)x, (double)y, (double)z);
        }

    }

    public struct fNode
    {
        public fTriple Pos0 { internal set; get; }
        public fTriple Position { internal set; get; }
        public fTriple Velocity { internal set; get; }
        public float OneOverMass { internal set; get; }
        public fTriple ConstantForce { internal set; get; } // only on some nodes!
        public fTriple Force { internal set; get; }
        public int SupportType { internal set; get; }
        public fTriple ReactionForce { internal set; get; } // only on some nodes!
        internal fNode(Point3d p0, Vector3d force, float mass, int support_type)
        {
            Pos0 = new fTriple(p0);
            Position = new fTriple(p0);
            Velocity = new fTriple();
            OneOverMass = 1 / mass;
            ConstantForce = new fTriple(force);
            Force = new fTriple();
            SupportType = support_type;
            ReactionForce = new fTriple();
        }
        internal void UpdateReactionForce()
        {
            fTriple nReactionForce = new fTriple(
                ((SupportType & 4) != 0) ? -Force.x : 0,
                ((SupportType & 2) != 0) ? -Force.y : 0,
                ((SupportType & 1) != 0) ? -Force.z : 0
                );
            ReactionForce = nReactionForce;
            //Force += ReactionForce; kinda side-effect, cleaned it
        }
        internal Node ToNode()
        {
            Node ret = new Node();
            ret.Position = Position.ToTriple();
            ret.Pos0 = Pos0.ToTriple();
            ret.Velocity = Velocity.ToTriple();
            ret.OneOverMass = OneOverMass;
            ret.ConstantForce = ConstantForce.ToTriple();
            ret.Force = Force.ToTriple();
            ret.SupportType = SupportType;
            ret.ReactionForce = ReactionForce.ToTriple();
            return ret;
        }
    }

    public struct fBeam
    {
        public int StartNode { internal set; get; }
        public int EndNode { internal set; get; }
        public float InitialLength { internal set; get; }
        public float SpringConstant { internal set; get; }
        public float InternalForce { internal set; get; } // +: compression beam: push nodes, -: tension beam: pull nodes
        internal fBeam(int startnode, int endnode, float initlen, float sprconst)
        {
            StartNode = startnode;
            EndNode = endnode;
            InitialLength = initlen;
            SpringConstant = sprconst;
            InternalForce = 0;
        }
        internal Beam ToBeam()
        {
            Beam ret = new Beam();
            ret.StartNode = StartNode;
            ret.EndNode = EndNode;
            ret.InitialLength = InitialLength;
            ret.SpringConstant = SpringConstant;
            ret.InternalForce = InternalForce;
            return ret;
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
            double t = XMath.Sqrt(XMath.Pow(x, 2) + XMath.Pow(y, 2) + XMath.Pow(z, 2)); // now_len, here!
            //if (t == 0) it will crash, but we shouldn't have zero len here!
            t = len / t; // to adjust the length
            x *= t; y *= t; z *= t;
        }
        internal double Len()
        {
            return XMath.Sqrt(x*x + y*y + z*z);
        }

        internal static double Distance(Triple a, Triple b)
        {
            return XMath.Sqrt(XMath.Pow(a.x - b.x, 2) + XMath.Pow(a.y - b.y, 2) + XMath.Pow(a.z - b.z, 2));
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
        internal fNode TofNode()
        {
            return new fNode(Pos0, Force, (float)Mass, SupportType); ;
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
        internal fBeam TofBeam()
        {
            return new fBeam(Link.Item1, Link.Item2, (float)Length, (float)Stiffness);
        }
    }
    public class Truss
    {
        // general data:
        public double DeltaTime { set; get; } = Default.DeltaTime;
        //private int Iteration = 0;
        //public int MaxStep { set; get; } = Default.MaxStep;
        private int NodeCount;
        private int BeamCount;
        public double DamperConstant { set; get; } = Default.DamperConstant;
        private bool ContextAllocated = false;
        // compiled data: this things are changed during update!!
        public Node[] Nodes { private set; get; }
        public Beam[] Beams { private set; get; }
        public fNode[] fNodes { private set; get; }
        public fBeam[] fBeams { private set; get; }
        private int[] ForcedNodesIndexes;
        private int[] SupportedNodesIndexes;

        // compiled data: this things are on the GPU!!, and then they will be copied to the compiled part
        Context context;
        Accelerator device;
        private MemoryBuffer1D<fNode,  Stride1D.Dense> gpuNodes;
        private MemoryBuffer1D<fBeam,  Stride1D.Dense> gpuBeams;
        private MemoryBuffer1D<int,   Stride1D.Dense> gpuForcedNodesIndexes;
        private MemoryBuffer1D<int,   Stride1D.Dense> gpuSupportedNodesIndexes;
        private MemoryBuffer1D<float,Stride1D.Dense> gpuDamperConstant;
        private MemoryBuffer1D<float, Stride1D.Dense> gpuDeltaTime;
        // kernel functions:
        Action<Index1D, ArrayView<fBeam>, ArrayView<fNode>, int> Lk_beam;
        Action<Index1D, ArrayView<fNode>, ArrayView<int>> Lk_constant_force;
        Action<Index1D, ArrayView<fNode>, ArrayView<float>> Lk_damper;
        Action<Index1D, ArrayView<fNode>, ArrayView<int>> Lk_support_nodes;
        Action<Index1D, ArrayView<fNode>, ArrayView<float>> Lk_nodes;

        // uncompiled data: This things are on the CPU, and ready to be changed 
        public List<ProtoNode> ProtoNodes { private set; get; }
        public List<ProtoBeam> ProtoBeams { private set; get; }

        // functions to use
        //public functions and stuff related to GPU

        public int DeviceType { private set; get; } // 0: CPU / 1: ThreadCPU / 2: ILGPU-CPU / 3: ILGPU-OpenCL / 4: ILGPU-CUDA 
        public Func<int, double> Update;
        public Func<int, int> Compile;

        // constructors:
        public Truss(List<Point3d> Points, double MaxBeamLen, string deviceType)
        {
            // another constructor
            BeamCount = NodeCount = 0;

        }
        public Truss(List<Line> Beam_Lines, double Tolerance, string deviceType)
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
            SetDeviceType(deviceType);
            SetDeviceFunctions();
        }
        public Truss(Truss other)
        {
            // copy constructor

            DeltaTime = other.DeltaTime;
            //MaxStep = other.MaxStep;
            //Iteration = other.Iteration;
            NodeCount = other.NodeCount;
            BeamCount = other.BeamCount;
            DamperConstant = other.DamperConstant;
            if (other.Nodes != null) Nodes = (Node[])other.Nodes.Clone();
            if (other.Beams != null) Beams = (Beam[])other.Beams.Clone();
            if (other.ForcedNodesIndexes != null) ForcedNodesIndexes = (int[])other.ForcedNodesIndexes.Clone();
            if (other.SupportedNodesIndexes != null) SupportedNodesIndexes = (int[])other.SupportedNodesIndexes.Clone();
            if (other.ProtoNodes != null) ProtoNodes = other.ProtoNodes.ToList();
            if (other.ProtoBeams != null) ProtoBeams = other.ProtoBeams.ToList();
            DeviceType = other.DeviceType;
            SetDeviceFunctions();
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


        private int CPU_Compile(int _=0)
        {
            // make code ready for gpu!
            //Iteration = 0;
            NodeCount = ProtoNodes.Count;
            BeamCount = ProtoBeams.Count;
            Nodes = new Node[NodeCount];
            Beams = new Beam[BeamCount];
            fNodes = new fNode[NodeCount];
            fBeams = new fBeam[BeamCount];
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
            return 0;
        }

        private double CPU_Update(int Step_count)
        {

            double free_forces = 0;
            // update beams, position is input, force is output
            for (int iterator = 0; iterator < Step_count; iterator++)
            {
                free_forces = 0;
                // KernelBeam
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


                // add the constant force to some nodes with force (KernelConstantForce)
                for (int i = 0; i < ForcedNodesIndexes.Length; i++)
                    Nodes[ForcedNodesIndexes[i]].Force += Nodes[ForcedNodesIndexes[i]].ConstantForce;

                // enforce damper constant to all nodes (KernelDamper)
                for (int i = 0; i < NodeCount; i++)
                    Nodes[i].Force -= Nodes[i].Velocity * DamperConstant;

                // make nodes with support 0 in force, put them in the ReactionForce (one of the main outputs!)(KernelSupprotNodes)
                for (int i = 0; i < SupportedNodesIndexes.Length; i++)
                {
                    Nodes[SupportedNodesIndexes[i]].UpdateReactionForce();
                    Nodes[SupportedNodesIndexes[i]].Force += Nodes[SupportedNodesIndexes[i]].ReactionForce;
                }

                // update nodes: with the force they receive <freeforcecalculation + (KernelNodes)>
                for (int i = 0; i < NodeCount; i++)
                {
                    free_forces += Nodes[i].Force.Len();
                    //Acceleration[i] = Force[i] / Mass[i]; // f = m.a
                    //Velocity[i] += Acceleration[i] * Delta; 
                    Nodes[i].Velocity += Nodes[i].Force * Nodes[i].OneOverMass * DeltaTime;
                    Nodes[i].Position += Nodes[i].Velocity * DeltaTime;
                    Nodes[i].Force = new Triple(); // we are always working with the copies of structs here :/
                }
            }
                //Iteration++;
            return free_forces;

        }
        private double Parallel_Update(int Step_count)
        {
            double free_forces = 0;
            // update beams, position is input, force is output
            for (int iterator = 0; iterator < Step_count; iterator++)
            {
                free_forces = 0;
                Parallel.For(0, BeamCount, i =>
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
                });


                // add the constant force to some nodes with force
                Parallel.For(0, ForcedNodesIndexes.Length, i =>
                {
                    Nodes[ForcedNodesIndexes[i]].Force += Nodes[ForcedNodesIndexes[i]].ConstantForce;
                });

                // enforce damper constant to all nodes
                Parallel.For(0, NodeCount, i =>
                {
                    Nodes[i].Force -= Nodes[i].Velocity * DamperConstant;
                });


                // make nodes with support 0 in force, put them in the ReactionForce (one of the main outputs!)
                Parallel.For(0, SupportedNodesIndexes.Length, i =>
                {
                    Nodes[SupportedNodesIndexes[i]].UpdateReactionForce();
                    Nodes[SupportedNodesIndexes[i]].Force += Nodes[SupportedNodesIndexes[i]].ReactionForce;
                });

                // sum the free force, only for the last iteration!, it cannot be parallel because of race condition
                if (iterator == Step_count -1)
                    for (int i = 0; i < NodeCount; i++)
                        free_forces += Nodes[i].Force.Len();

                // update nodes: with the force they receive
                Parallel.For(0, NodeCount, i =>
                {
                    
                    Nodes[i].Velocity += Nodes[i].Force * Nodes[i].OneOverMass * DeltaTime;
                    Nodes[i].Position += Nodes[i].Velocity * DeltaTime;
                    Nodes[i].Force = new Triple(); // we are always working with the copies of structs here :/
                });
                
            }
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

        private int ILGPU_Compile(int _ = 0)
        {
            /**/
            NodeCount = ProtoNodes.Count;
            BeamCount = ProtoBeams.Count;
            fNodes = new fNode[NodeCount];
            fBeams = new fBeam[BeamCount];

            Nodes = new Node[NodeCount];
            Beams = new Beam[BeamCount];
            ForcedNodesIndexes = new int[ProtoNodes.Count(item => !item.Force.IsZero)]; // count number of non, zero forces
            SupportedNodesIndexes = new int[ProtoNodes.Count(item => item.SupportType != 0)]; // like-wise, for supports
            // this is also possible with LINQ, but I prefer to keep it simple
            int fn_po = 0, sn_po = 0;
            for (int i = 0; i < NodeCount; i++)
            {
                fNodes[i] = ProtoNodes[i].TofNode();
                if (!ProtoNodes[i].Force.IsZero) ForcedNodesIndexes[fn_po++] = i;
                if (ProtoNodes[i].SupportType != 0) SupportedNodesIndexes[sn_po++] = i;
            }
            for (int i = 0; i < BeamCount; i++) fBeams[i] = ProtoBeams[i].TofBeam();
            /**/
            if (!ContextAllocated)
            {
                
                context = Context.Create(builder => builder.Default().EnableAlgorithms());
                if(DeviceType == 2)
                    device = context.GetCPUDevice(0).CreateAccelerator(context);
                if(DeviceType == 3)
                    device = context.GetCLDevice(0).CreateAccelerator(context);
                if (DeviceType == 4)
                    device = context.GetCudaDevice(0).CreateAccelerator(context);
                gpuDamperConstant = device.Allocate1D<float>(1);
                gpuDeltaTime = device.Allocate1D<float>(1);

                // Load all kernels
                //Action<Index1D, ArrayView<TripleTest>> test = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<TripleTest>>(TestKernel);
                Lk_beam = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<fBeam>, ArrayView<fNode>,int> (KernelBeam2);
                /*debug1*/
                Lk_constant_force = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<fNode>, ArrayView<int>>(KernelConstantForce);
                Lk_damper = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<fNode>, ArrayView<float>>(KernelDamper);
                Lk_support_nodes = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<fNode>, ArrayView<int>>(KernelSupprotNodes);
                Lk_nodes = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<fNode>, ArrayView<float>> (KernelNodes);

                ContextAllocated = true;
                /**/
            }
            else
            {
                // we have came here before, so we should despose the previous things on GPU to avoid memmory leak ?!
                gpuNodes.Dispose();
                gpuBeams.Dispose();
                gpuForcedNodesIndexes.Dispose();
                gpuSupportedNodesIndexes.Dispose();
                // gpuDamperConstant.Dispose();
                // gpuDeltaTime.Dispose(); no need for this, their size is always fixed. so just copy the array again here
            }

            // now send stuff into the GPU!
            // we keep these here, because it may 
            /* debug1*/
            gpuNodes = device.Allocate1D(fNodes);
            gpuBeams = device.Allocate1D(fBeams);
            gpuForcedNodesIndexes = device.Allocate1D(ForcedNodesIndexes);
            gpuSupportedNodesIndexes = device.Allocate1D(SupportedNodesIndexes);
            gpuDamperConstant.CopyFromCPU(new float[]{(float)DamperConstant});
            gpuDeltaTime.CopyFromCPU(new float[] { (float)DeltaTime });
            /**/
            // end sending them
            return 0;
        }

        private double ILGPU_Update(int Step_count)
        {
            for(int i = 0; i < Step_count; i++)
            {

                Lk_beam(1, gpuBeams.View, gpuNodes.View,BeamCount);
                device.Synchronize();
                Lk_constant_force(NodeCount, gpuNodes.View, gpuForcedNodesIndexes.View);
                device.Synchronize();
                Lk_damper(NodeCount,gpuNodes.View,gpuDamperConstant.View);
                device.Synchronize();
                Lk_support_nodes(NodeCount, gpuNodes.View, gpuSupportedNodesIndexes.View);
                // next phases: sum the free force and output it
                Lk_nodes(NodeCount, gpuNodes.View, gpuDeltaTime.View);
                device.Synchronize();
            }
            // send it back to the CPU memory
            gpuBeams.CopyToCPU(fBeams);
            gpuNodes.CopyToCPU(fNodes);
            for(int i = 0; i < BeamCount; i++)
            {
                Beams[i] = fBeams[i].ToBeam();
            }
            for(int i = 0;i < NodeCount; i++)
            {
                Nodes[i] = fNodes[i].ToNode();
            }
            return -1;
        }

        // Kernels:

        static void KernelBeam2(Index1D iterator, ArrayView<fBeam> beams, ArrayView<fNode> nodes, int beam_count)
        {
            for (int i = 0; i < beam_count; i++)
            {
                /*debug1*/
                float delta_len = fTriple.Distance(nodes[beams[i].StartNode].Position, nodes[beams[i].EndNode].Position)
                    - beams[i].InitialLength;
                beams[i].InternalForce = -beams[i].SpringConstant * delta_len;
                fTriple vector_force = new fTriple(
                    nodes[beams[i].StartNode].Position,
                    nodes[beams[i].EndNode].Position,
                    beams[i].InternalForce
                    );
                nodes[beams[i].StartNode].Force -= vector_force;
                nodes[beams[i].EndNode].Force += vector_force;/**/
            }
        }

        static void KernelBeam(Index1D i, ArrayView<fBeam> beams, ArrayView<fNode> nodes)
        {

            /*debug1*/
            float delta_len = fTriple.Distance(nodes[beams[i].StartNode].Position, nodes[beams[i].EndNode].Position)
                - beams[i].InitialLength;
            beams[i].InternalForce = -beams[i].SpringConstant * delta_len;
            fTriple vector_force = new fTriple(
                nodes[beams[i].StartNode].Position,
                nodes[beams[i].EndNode].Position,
                beams[i].InternalForce
                );
            nodes[beams[i].StartNode].Force -= vector_force; 
            nodes[beams[i].EndNode].Force += vector_force;/**/
        }

        static void KernelConstantForce(Index1D i, ArrayView<fNode> nodes, ArrayView<int> forced_nodes_indexes)
        {
            //nodes[forced_nodes_indexes[i]].Force += nodes[forced_nodes_indexes[i]].ConstantForce;
            nodes[i].Force += nodes[i].ConstantForce;
        }

        static void KernelDamper(Index1D i, ArrayView<fNode> nodes, ArrayView<float> damper_constant)
        {
            nodes[i].Force -= nodes[i].Velocity * damper_constant[0];
        }

        static void KernelSupprotNodes(Index1D i, ArrayView<fNode> nodes, ArrayView<int> supported_nodes_indexes)
        {
            //nodes[supported_nodes_indexes[i]].UpdateReactionForce();
            //nodes[supported_nodes_indexes[i]].Force += nodes[supported_nodes_indexes[i]].ReactionForce;
            nodes[i].UpdateReactionForce();
            nodes[i].Force += nodes[i].ReactionForce;
        }
        static void KernelNodes(Index1D i, ArrayView<fNode> nodes, ArrayView<float> delta_time)
        {
            nodes[i].Velocity += nodes[i].Force * nodes[i].OneOverMass * delta_time[0];
            nodes[i].Position += nodes[i].Velocity * delta_time[0];
            nodes[i].Force = new fTriple(); // we are always working with the copies of structs here :/
        }

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
        private void SetDeviceType(string deviceType)
        {
            // 0: CPU / 1: ThreadCPU / 2: ILGPU-CPU / 3: ILGPU-OpenCL / 4: ILGPU-CUDA
            switch (deviceType)
            {
                case "ILGPU-CUDA":
                    DeviceType = 4; break;
                case "ILGPU-OpenCL":
                    DeviceType = 3; break;
                case "ILGPU-CPU":
                    DeviceType = 2; break;
                case "ThreadCPU":
                    DeviceType = 1; break;
                default:
                    DeviceType = 0; break;
            }
        }

        private void SetDeviceFunctions()
        {
            if(DeviceType >= 2)
            {
                Compile = ILGPU_Compile;
                Update = ILGPU_Update;
            }
            else if(DeviceType == 1)
            {
                Compile = CPU_Compile;
                Update = Parallel_Update;
            }
            else
            {
                Compile = CPU_Compile;
                Update = CPU_Update;
            }
        }

        ~Truss()
        {
            if(ContextAllocated)
            {
                gpuNodes.Dispose();
                gpuBeams.Dispose();
                gpuForcedNodesIndexes.Dispose();
                gpuSupportedNodesIndexes.Dispose();
                gpuDamperConstant.Dispose();
                gpuDeltaTime.Dispose();
                device.Dispose();
                context.Dispose();
            }
        }

    }
}


namespace Liz
{
    internal class Default
    {
        internal const double Mass = 10;
        internal const double Stiffness = 500;
        internal const double DeltaTime = 0.01;
        internal const int MaxStep = 100;
        internal const double DamperConstant = 20;
        internal static readonly Vector3d ZeroVector = new Vector3d(0, 0, 0);
        internal static readonly Point3d ZeroPoint = new Point3d(0, 0, 0);
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

    public interface ISimulator
    {
        void Send(List<ProtoNode> ProtoNodes, List<ProtoBeam> ProtoBeams);
        double Update(int Step_count);
        void Receive(ref Point3d[] points_positions, ref Vector3d[] reaction_forces, ref Tuple<int, int, double>[] beam_forces);

    }

    public class Truss
    {
        // general data:
        public double DeltaTime { set; get; } = Default.DeltaTime;
        public double DamperConstant { set; get; } = Default.DamperConstant;
        // these are the inputs of our truss, the protos:
        public List<ProtoNode> ProtoNodes { private set; get; }
        public List<ProtoBeam> ProtoBeams { private set; get; }

        // these data are the outputs of our truss
        public Point3d[] oNodes;
        public Vector3d[] oReactionForces;  
        public Tuple<int, int, double>[] oBeamForces;
        // details of the simulator we are using
        internal int SimulatorType { private set; get; } // 0: CPU / 1: ThreadCPU / 2: ILGPU-CPU / 3: ILGPU-OpenCL / 4: ILGPU-CUDA 
        ISimulator Simulator;

        // constructors:
        public Truss(List<Point3d> Points, double MaxBeamLen, string deviceType)
        {
            // another constructor

        }
        public Truss(List<Line> Beam_Lines, double Tolerance, string deviceType)
        {
            // classic constructor

            ProtoNodes = new List<ProtoNode>();
            ProtoBeams = new List<ProtoBeam>();

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
            SetSimulatorType(deviceType);
        }
        public Truss(Truss other)
        {
            // copy constructor
            // single data
            DeltaTime = other.DeltaTime;
            DamperConstant = other.DamperConstant;
            // input data
            if (other.ProtoNodes != null) ProtoNodes = other.ProtoNodes.ToList();
            if (other.ProtoBeams != null) ProtoBeams = other.ProtoBeams.ToList();
            // output data
            if (other.oNodes != null) oNodes = (Point3d[])other.oNodes.Clone();
            if (other.oReactionForces != null) oReactionForces = (Vector3d[])other.oReactionForces.Clone();
            if (other.oBeamForces != null) oBeamForces = (Tuple<int, int, double>[])other.oBeamForces.Clone();
            // simulator
            SimulatorType = other.SimulatorType;
            SetSimulatorType("", SimulatorType);
        }

        public void Compile()
        {
            Simulator.Send(ProtoNodes, ProtoBeams);
        }
        public double Update(int Step_Count)
        {
            return Simulator.Update(Step_Count);
        }
        public void Receive()
        {
            Simulator.Receive(ref oNodes, ref oReactionForces, ref oBeamForces);
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
        private void SetSimulatorType(string deviceType, int deviceCode = -1)
        {
            // 0: CPU / 1: ThreadCPU / 2: ILGPU-CPU / 3: ILGPU-OpenCL / 4: ILGPU-CUDA
            if (deviceCode == -1) {
                switch (deviceType)
                {
                    case "ILGPU-CUDA":
                        SimulatorType = 4;
                        break;
                    case "ILGPU-OpenCL":
                        SimulatorType = 3; break;
                    case "ILGPU-CPU":
                        SimulatorType = 2; break;
                    case "ThreadCPU":
                        SimulatorType = 1; break;
                    default:
                        SimulatorType = 0; break;
                }
            }
            //SetSimulatorType 

        }

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

    public class CPU_Simulator: ISimulator
    {
        private double DeltaTime, DamperConstant;
        private int NodeCount, BeamCount;
        // compiled data: this things are kept to work in the update with them!!
        private Node[] Nodes;
        private Beam[] Beams;
        private int[] ForcedNodesIndexes;
        private int[] SupportedNodesIndexes;
        // the update function
        private Func<int, double> Updatetype;

        internal CPU_Simulator(bool parallel)
        {
            if (parallel)
                Updatetype = UpdateParallelFor;
            else
                Updatetype = UpdateConventional;
            
        }
        public void Send(List<ProtoNode> ProtoNodes, List<ProtoBeam> ProtoBeams) {
            NodeCount = ProtoNodes.Count;
            BeamCount = ProtoBeams.Count;
            Nodes = new Node[NodeCount];
            Beams = new Beam[BeamCount];
            ForcedNodesIndexes = new int[ProtoNodes.Count(item => !item.Force.IsZero)]; // count number of non, zero forces
            SupportedNodesIndexes = new int[ProtoNodes.Count(item => item.SupportType != 0)]; // like-wise, for supports
            // this is also possible with LINQ, but I prefer to keep it simple
            int fn_po = 0, sn_po = 0;
            for (int i = 0; i < NodeCount; i++)
            {
                Nodes[i] = new Node(ProtoNodes[i]);
                if (!ProtoNodes[i].Force.IsZero) ForcedNodesIndexes[fn_po++] = i;
                if (ProtoNodes[i].SupportType != 0) SupportedNodesIndexes[sn_po++] = i;
            }
            for (int i = 0; i < BeamCount; i++) Beams[i] = new Beam(ProtoBeams[i]);
        }
        public double Update(int Step_count) { return Updatetype(Step_count); }
        public void Receive(ref Point3d[] points_positions, ref Vector3d[] reaction_forces, ref Tuple<int, int, double>[] beam_forces)
        {
            if(points_positions==null || points_positions.Length!= NodeCount)
                points_positions = new Point3d[NodeCount];
            if(reaction_forces==null || reaction_forces.Length!= NodeCount)
                reaction_forces = new Vector3d[NodeCount];
            if (beam_forces == null || beam_forces.Length != BeamCount)
                beam_forces = new Tuple<int, int, double>[BeamCount];

            for(int i = 0; i < NodeCount; i++)
            {
                points_positions[i] = new Point3d(Nodes[i].Pos0.x, Nodes[i].Pos0.y, Nodes[i].Pos0.z);
                reaction_forces[i] = new Vector3d(Nodes[i].ReactionForce.x, Nodes[i].ReactionForce.y, Nodes[i].ReactionForce.z);
            }
            for(int i = 0; i < BeamCount; i++)
                beam_forces[i] = new Tuple<int, int, double>(Beams[i].StartNode, Beams[i].EndNode, Beams[i].InternalForce);

        }

        private double UpdateConventional(int Step_count) {
            double free_forces = 0;
            // update beams, position is input, force is output
            for (int iterator = 0; iterator < Step_count; iterator++)
            {
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
                for (int i = 0; i < ForcedNodesIndexes.Length; i++)
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
                // sum the free force, only for the last iteration!
                if (iterator == Step_count - 1)
                    for (int i = 0; i < NodeCount; i++)
                        free_forces += Nodes[i].Force.Len();

                // update nodes  with the force they receive
                for (int i = 0; i < NodeCount; i++)
                {
                    //Acceleration[i] = Force[i] / Mass[i]; // f = m.a
                    //Velocity[i] += Acceleration[i] * Delta; 
                    Nodes[i].Velocity += Nodes[i].Force * Nodes[i].OneOverMass * DeltaTime;
                    Nodes[i].Position += Nodes[i].Velocity * DeltaTime;
                    Nodes[i].Force = new Triple(); // we are always working with the copies of structs here :/
                }
            }
            return free_forces;
        }
        private double UpdateParallelFor(int Step_count) {
            double free_forces = 0;
            // update beams, position is input, force is output
            for (int iterator = 0; iterator < Step_count; iterator++)
            {
                free_forces = 0;
                Parallel.For(0, BeamCount, i =>
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
                });

                // add the constant force to some nodes with force
                Parallel.For(0, ForcedNodesIndexes.Length, i =>
                {
                    Nodes[ForcedNodesIndexes[i]].Force += Nodes[ForcedNodesIndexes[i]].ConstantForce;
                });

                // enforce damper constant to all nodes
                Parallel.For(0, NodeCount, i =>
                {
                    Nodes[i].Force -= Nodes[i].Velocity * DamperConstant;
                });

                // make nodes with support 0 in force, put them in the ReactionForce (one of the main outputs!)
                Parallel.For(0, SupportedNodesIndexes.Length, i =>
                {
                    Nodes[SupportedNodesIndexes[i]].UpdateReactionForce();
                    Nodes[SupportedNodesIndexes[i]].Force += Nodes[SupportedNodesIndexes[i]].ReactionForce;
                });

                // sum the free force, only for the last iteration!, it cannot be parallel because of race condition
                if (iterator == Step_count - 1)
                    for (int i = 0; i < NodeCount; i++)
                        free_forces += Nodes[i].Force.Len();

                // update nodes: with the force they receive
                Parallel.For(0, NodeCount, i =>
                {
                    Nodes[i].Velocity += Nodes[i].Force * Nodes[i].OneOverMass * DeltaTime;
                    Nodes[i].Position += Nodes[i].Velocity * DeltaTime;
                    Nodes[i].Force = new Triple(); // we are always working with the copies of structs here :/
                });
            }
            return free_forces;
        }


        private struct Triple
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
                return Math.Sqrt(x * x + y * y + z * z);
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
        private struct Node
        {
            public Triple Pos0;// { internal set; get; }
            public Triple Position;// { internal set; get; }
            public Triple Velocity;// { internal set; get; }
            public double OneOverMass;// { internal set; get; }
            public Triple ConstantForce;// { internal set; get; } // only on some nodes!
            public Triple Force;// { internal set; get; }
            public int SupportType;// { internal set; get; }
            public Triple ReactionForce;// { internal set; get; } // only on some nodes!
            internal Node(Point3d p0, Vector3d force, double mass, int support_type)
            {
                Pos0 = new Triple(p0);
                Position = new Triple(p0);
                Velocity = new Triple();
                OneOverMass = 1 / mass;
                ConstantForce = new Triple(force);
                Force = new Triple();
                SupportType = support_type;
                ReactionForce = new Triple();
            }
            internal Node(ProtoNode p0)
            {
                Pos0 = new Triple(p0.Pos0);
                Position = new Triple(p0.Pos0);
                Velocity = new Triple();
                OneOverMass = 1 / p0.Mass;
                ConstantForce = new Triple(p0.Force);
                Force = new Triple();
                SupportType = p0.SupportType;
                ReactionForce = new Triple();
            }
            internal void UpdateReactionForce()
            {
                Triple nReactionForce = new Triple(
                    ((SupportType & 4) != 0) ? -Force.x : 0,
                    ((SupportType & 2) != 0) ? -Force.y : 0,
                    ((SupportType & 1) != 0) ? -Force.z : 0
                    );
                ReactionForce = nReactionForce;
                //Force += ReactionForce; kinda side-effect, cleaned it
            }
        }
        private struct Beam
        {
            public int StartNode;// { internal set; get; }
            public int EndNode;// { internal set; get; }
            public double InitialLength;// { internal set; get; }
            public double SpringConstant;// { internal set; get; }
            public double InternalForce;// { internal set; get; } // +: compression beam: push nodes, -: tension beam: pull nodes
            internal Beam(int startnode, int endnode, double initlen, double sprconst)
            {
                StartNode = startnode;
                EndNode = endnode;
                InitialLength = initlen;
                SpringConstant = sprconst;
                InternalForce = 0;
            }
            internal Beam(ProtoBeam beam)
            {
                StartNode = beam.Link.Item1;
                EndNode = beam.Link.Item2;
                InitialLength = beam.Length;
                SpringConstant = beam.Stiffness;
                InternalForce = 0;
            }
        }

    }

    public class ILGPU_Simulator: ISimulator
    {
        public void Send(List<ProtoNode> ProtoNodes, List<ProtoBeam> ProtoBeams) { }
        public double Update(int Step_count) { return 0; }
        public void Receive(ref Point3d[] points_positions, ref Vector3d[] reaction_forces, ref Tuple<int, int, double>[] beam_forces) { }
    }

    public struct ProtoNode
    {
        public Point3d Pos0 { internal set; get; } // start position 
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

    }
}