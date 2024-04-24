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
        int Type { set; get; }
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
            switch (SimulatorType)
            {
                case 0:
                    Simulator = new CPU_Simulator();
                    break;
                case 1:
                    Simulator = new CPU_Simulator();
                    break;
                case 2:
                    Simulator = new ILGPU_Simulator();
                    break;
                case 3:
                    Simulator = new ILGPU_Simulator();
                    break;
                case 4:
                    Simulator = new ILGPU_Simulator();
                break;
            }    
            Simulator.Type = SimulatorType;
            
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

    public class CPU_Simulator : ISimulator
    {
        private readonly double DeltaTime, DamperConstant;
        private int NodeCount, BeamCount;
        // compiled data: this things are kept to work in the update with them!!
        private Node[] Nodes;
        private Beam[] Beams;
        private int[] ForcedNodesIndexes;
        private int[] SupportedNodesIndexes;
        // the update function
        private Func<int, double> Updatetype;
        public int Type{
            get { return Type; }
            set {
                    if (value == 1)
                        Updatetype = UpdateParallelFor;
                    if (value == 0)
                        Updatetype = UpdateConventional;
                    Type = value; 
                }
            }


        internal CPU_Simulator()
        {
            Type = 0;
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
                    Nodes[Beams[i].StartNode].Force -= vector_force;
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

    public class ILGPU_Simulator : ISimulator
    {
        // NO DOUBLE HERE, ONLY FLOATS
        // general data:
        private readonly float DeltaTime, DamperConstant;
        private int NodeCount, BeamCount;
        private bool ContextAllocated = false;

        // compiled data: this things are on the GPU!!, and then they will be copied to the compiled part
        Context context;
        Accelerator device;
        private MemoryBuffer1D<Node, Stride1D.Dense> Nodes;
        private MemoryBuffer1D<Beam, Stride1D.Dense> Beams;
        private MemoryBuffer1D<Triple, Stride1D.Dense> ForceOutputsFromBeams; //fofb
        //private MemoryBuffer1D<int, Stride1D.Dense> ForcedNodesIndexes;
        private MemoryBuffer1D<int, Stride1D.Dense> SupportedNodesIndexes;
        //private MemoryBuffer1D<float, Stride1D.Dense> gpuDamperConstant;
        //private MemoryBuffer1D<float, Stride1D.Dense> gpuDeltaTime;
        // kernel functions:
        Action<Index1D, ArrayView<Beam>, ArrayView<Node>, ArrayView<Triple>> Lk_UpdateBeam;
        Action<Index1D, ArrayView<Node>, ArrayView<Triple>, float> Lk_UpdateNodeForcesAndDamper;
        //Action<Index1D, ArrayView<fNode>, ArrayView<float>> Lk_damper;
        Action<Index1D, ArrayView<Node>, ArrayView<int>> Lk_UpdateSupportNodes;
        Action<Index1D, ArrayView<Node>, float> Lk_UpdateNodes;

        internal ILGPU_Simulator()
        {
            Type = 2;
        }

        // functions to use
        //public functions and stuff related to GPU
        public int Type { set; get; }
        public void Send(List<ProtoNode> ProtoNodes, List<ProtoBeam> ProtoBeams) {
            NodeCount = ProtoNodes.Count;
            BeamCount = ProtoBeams.Count;
            Node[] tempNodes = new Node[NodeCount];
            Beam[] tempBeams = new Beam[BeamCount];
            Triple[] tempFofb = new Triple[BeamCount * 2]; // later we just copy it, we want it to be all zero
            int[] tempSupportedNodesIndexes = new int[ProtoNodes.Count(item => item.SupportType != 0)];
            // make support node indexes
            int sn_po = 0;
            for (int i = 0; i < NodeCount; i++)
            {
                tempNodes[i] = new Node(ProtoNodes[i]);
                if (ProtoNodes[i].SupportType != 0) tempSupportedNodesIndexes[sn_po++] = i;
            }
            for (int i = 0; i < BeamCount; i++) tempBeams[i] = new Beam(ProtoBeams[i]);

            // make the FoFb ok & other things
            List<List<Tuple<int, int>>> graph = new List<List<Tuple<int,int>>>(NodeCount);
            for(int  i = 0; i < NodeCount; i++)
            {
                graph[i] = new List<Tuple<int, int>>();
            }
            for(int i = 0; i < BeamCount; i++)
            {
                graph[ProtoBeams[i].Link.Item1].Add(new Tuple<int,int>(i, -1)); // start node: -1
                graph[ProtoBeams[i].Link.Item2].Add(new Tuple<int, int>(i, 1)); // end node : 1
            }
            int checked_elements = 0;
            for(int i = 0; i < NodeCount; i++)
            {
                // update the node, add start and end of the range of the neighbors
                Node tempn = tempNodes[i];
                tempn.StartRange = checked_elements;

                // update each beam's detail that is connected to this particular node
                for(int j = 0; j < graph[i].Count; j++)
                {
                    Beam tempb = tempBeams[graph[i][j].Item1];
                    if (graph[i][j].Item2 == -1)
                    {
                        tempb.StartNodePointer = checked_elements;
                    }
                    else 
                    {
                        tempb.EndNodePointer = checked_elements;
                    }
                    checked_elements++;
                    tempBeams[graph[i][j].Item1] = tempb;
                }
                tempn.EndRange = checked_elements;
                tempNodes[i] = tempn;
            }


            // make the context
            if (!ContextAllocated)
            {
                context = Context.Create(builder => builder.Default().EnableAlgorithms().Math(MathMode.Fast32BitOnly));
                if (Type == 2)
                    device = context.GetCPUDevice(0).CreateAccelerator(context);
                if (Type == 3)
                    device = context.GetCLDevice(0).CreateAccelerator(context);
                if (Type == 4)
                    device = context.GetCudaDevice(0).CreateAccelerator(context);
                // Load all kernels
                Lk_UpdateBeam = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<Beam>, ArrayView<Node>, ArrayView<Triple>>
                    (UpdateBeam);
                Lk_UpdateNodeForcesAndDamper = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<Node>, ArrayView<Triple>, float>
                    (UpdateNodesForcesAndDamper);
                Lk_UpdateSupportNodes = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<Node>, ArrayView<int>>
                    (UpdateSupportNodes);
                Lk_UpdateNodes = device.LoadAutoGroupedStreamKernel<Index1D, ArrayView<Node>, float> (UpdateNodes);
                ContextAllocated = true;
                /**/
            }
            else
            {
                // we have came here before, so we should despose the previous things on GPU to avoid memmory leak!
                Nodes.Dispose();
                Beams.Dispose();
                ForceOutputsFromBeams.Dispose();
                SupportedNodesIndexes.Dispose();
            }
            // now send stuff into the GPU!
            Nodes = device.Allocate1D(tempNodes);
            Beams = device.Allocate1D(tempBeams);
            ForceOutputsFromBeams = device.Allocate1D(tempFofb);
            SupportedNodesIndexes = device.Allocate1D(tempSupportedNodesIndexes);
            // end sending them, then will receive them in the Receive function!
        }
        public double Update(int Step_count) {
            for (int i = 0; i < Step_count; i++)
            {
                Lk_UpdateBeam(BeamCount, Beams.View, Nodes.View, ForceOutputsFromBeams.View);
                device.Synchronize();
                Lk_UpdateNodeForcesAndDamper(NodeCount, Nodes.View, ForceOutputsFromBeams.View, DamperConstant);
                device.Synchronize();
                Lk_UpdateSupportNodes(NodeCount, Nodes.View, SupportedNodesIndexes.View);
                device.Synchronize();
                // next phases: sum the free force and output it
                Lk_UpdateNodes(NodeCount, Nodes.View, DeltaTime);
                device.Synchronize();
            }
            return -1;
        }
        public void Receive(ref Point3d[] points_positions, ref Vector3d[] reaction_forces, ref Tuple<int, int, double>[] beam_forces)
        {
            // init
            if (points_positions == null || points_positions.Length != NodeCount)
                points_positions = new Point3d[NodeCount];
            if (reaction_forces == null || reaction_forces.Length != NodeCount)
                reaction_forces = new Vector3d[NodeCount];
            if (beam_forces == null || beam_forces.Length != BeamCount)
                beam_forces = new Tuple<int, int, double>[BeamCount];

            Node[] tempNodes = new Node[NodeCount];
            Beam[] tempBeams = new Beam[NodeCount];

            // send it back to the CPU memory
            Beams.CopyToCPU(tempBeams);
            Nodes.CopyToCPU(tempNodes);

            for (int i = 0; i < NodeCount; i++)
            {
                points_positions[i] = new Point3d(tempNodes[i].Pos0.x, tempNodes[i].Pos0.y, tempNodes[i].Pos0.z);
                reaction_forces[i] = new Vector3d(tempNodes[i].ReactionForce.x, tempNodes[i].ReactionForce.y, tempNodes[i].ReactionForce.z);
            }
            for (int i = 0; i < BeamCount; i++)
                beam_forces[i] = new Tuple<int, int, double>(tempBeams[i].StartNode, tempBeams[i].EndNode, tempBeams[i].InternalForce);

        }

        // Kernels are here!
        static void UpdateBeam(Index1D i, ArrayView<Beam> beams, ArrayView<Node> nodes, ArrayView<Triple> fofb)
        {
            float delta_len = Triple.Distance(nodes[beams[i].StartNode].Position, nodes[beams[i].EndNode].Position)
                - beams[i].InitialLength;
            beams[i].InternalForce = -beams[i].SpringConstant * delta_len;
            Triple vector_force = new Triple(
                nodes[beams[i].StartNode].Position,
                nodes[beams[i].EndNode].Position,
                beams[i].InternalForce
                );
            fofb[beams[i].StartNodePointer] = -vector_force;
            fofb[beams[i].EndNodePointer] = vector_force;
        }

        static void UpdateNodesForcesAndDamper(Index1D i, ArrayView<Node> nodes, ArrayView<Triple> fofb, float damper)
        {
            nodes[i].Force = nodes[i].ConstantForce;
            for (int j = nodes[i].StartRange; j < nodes[i].EndRange; j++)
                nodes[i].Force += fofb[j];
            nodes[i].Force -= nodes[i].Velocity * damper;
        }

        static void UpdateSupportNodes(Index1D i, ArrayView<Node> nodes, ArrayView<int> supported_nodes_indexes)
        {
            nodes[supported_nodes_indexes[i]].UpdateReactionForce();
            nodes[supported_nodes_indexes[i]].Force += nodes[supported_nodes_indexes[i]].ReactionForce;
        }
        static void UpdateNodes(Index1D i, ArrayView<Node> nodes, float delta_time)
        {
            nodes[i].Velocity += nodes[i].Force * nodes[i].OneOverMass * delta_time;
            nodes[i].Position += nodes[i].Velocity * delta_time;
        }

        ~ILGPU_Simulator()
        {
            if (ContextAllocated)
            {
                Nodes.Dispose();
                Beams.Dispose();
                ForceOutputsFromBeams.Dispose(); 
                SupportedNodesIndexes.Dispose();
                device.Dispose();
                context.Dispose();
            }
        }

        private struct Triple
        {
            public float x, y, z;
            public Triple(float x, float y, float z)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }
            internal Triple(Point3d p)
            {
                x = (float)p.X; y = (float)p.Y; z = (float)p.Z;
            }
            internal Triple(Vector3d v)
            {
                x = (float)v.X; y = (float)v.Y; z = (float)v.Z;
            }
            internal Triple(Triple start, Triple end, float len)
            {
                // make a vector, from start to end with a set len
                x = end.x - start.x; y = end.y - start.y; z = end.z - start.z;
                float t = XMath.Sqrt(x * x + y * y + z * z); // now_len, here!
                t = len / t; // to adjust the length
                x *= t; y *= t; z *= t;
            }
            internal float Len()
            {
                return XMath.Sqrt(x * x + y * y + z * z);
            }

            internal static float Distance(Triple a, Triple b)
            {
                //return Math.Sqrt(Math.Pow(a.x - b.x, 2) + Math.Pow(a.y - b.y, 2) + Math.Pow(a.z - b.z, 2));
                return XMath.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
            }
            public static Triple operator +(Triple a, Triple b)
            {
                return new Triple(a.x + b.x, a.y + b.y, a.z + b.z);
            }
            public static Triple operator -(Triple a, Triple b)
            {
                return new Triple(a.x - b.x, a.y - b.y, a.z - b.z);
            }
            public static Triple operator -(Triple a)
            {
                return new Triple(-a.x, -a.y, -a.z);
            }
            public static Triple operator *(Triple a, float b)
            {
                return new Triple(a.x * b, a.y * b, a.z * b);
            }
        }
        private struct Node
        {
            public Triple Pos0;
            public Triple Position;
            public Triple Velocity;
            public float OneOverMass;
            public Triple ConstantForce;
            public Triple Force;
            public int SupportType;
            public Triple ReactionForce;
            public int StartRange, EndRange;
                
            internal Node(ProtoNode p0)
            {
                Pos0 = new Triple(p0.Pos0);
                Position = new Triple(p0.Pos0);
                Velocity = new Triple();
                OneOverMass = 1 / (float)p0.Mass;
                ConstantForce = new Triple(p0.Force);
                Force = new Triple();
                SupportType = p0.SupportType;
                ReactionForce = new Triple();
                StartRange = EndRange = -1; // should be updated later
            }
            internal void UpdateReactionForce()
            {
                Triple nReactionForce = new Triple(
                    ((SupportType & 4) != 0) ? -Force.x : 0,
                    ((SupportType & 2) != 0) ? -Force.y : 0,
                    ((SupportType & 1) != 0) ? -Force.z : 0
                    );
                ReactionForce = nReactionForce;
            }
        }
        private struct Beam
        {
            public int StartNode;
            public int EndNode;
            public float InitialLength;
            public float SpringConstant;
            public float InternalForce;// +: compression beam: push nodes, -: tension beam: pull nodes
            public int StartNodePointer, EndNodePointer;
                
            internal Beam(ProtoBeam beam)
            {
                StartNode = beam.Link.Item1;
                EndNode = beam.Link.Item2;
                InitialLength = (float)beam.Length;
                SpringConstant = (float)beam.Stiffness;
                InternalForce = 0;
                StartNodePointer = EndNodePointer = -1;
            }
        }

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