using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Remoting.Services;
using System.Text;
using System.Threading.Tasks;

using Rhino;
using Rhino.Geometry;
using static Truss.Truss;

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
        private int[] Par;
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
            if (t == 0) return; // an unfortunate event, we just put it under the rug :/
            t = len / t; // to adjust the length
            x *= t; y *= t; z *= t;
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
    } 
    public struct Node {
        public Triple Pos0 { internal set; get; }
        public Triple Position { internal set; get; }
        public Triple Velocity { internal set; get; }
        public double Mass { internal set; get; }
        public Triple ConstantForce { internal set; get; } // only on some nodes!
        public Triple Force { internal set; get; }
        public int SupportType { internal set; get; }
        public Triple ReactionForce { internal set; get; } // only on some nodes!
        internal Node(Point3d p0, Vector3d force, double mass, int support_type)
        {
            Pos0 = new Triple(p0);// { x = p0.X, y = p0.Y, z = p0.Z };
            Position = new Triple(p0);// { x = p0.X, y = p0.Y, z = p0.Z };
            Velocity = new Triple();
            Mass = mass;
            ConstantForce = new Triple(force);
            Force = new Triple();
            SupportType = support_type;
            ReactionForce = new Triple();
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
            Force = new Vector3d(0, 0, 0);// Default.ZeroVector;
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
            
        }
        public Truss(List<Line> Beam_Lines, double Tolerance)
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
        }
        public Truss(Truss other)
        {
            // copy constructor
            DeltaTime = other.DeltaTime;
            Iteration = other.Iteration;
            NodeCount = other.NodeCount;
            BeamCount = other.BeamCount;
            DamperConstant = other.DamperConstant;
            Nodes = (Node[])other.Nodes.Clone();
            Beams = (Beam[])other.Beams.Clone();
            ForcedNodesIndexes = (int[])other.ForcedNodesIndexes.Clone();
            SupportedNodesIndexes = (int[])other.SupportedNodesIndexes.Clone();
            ProtoNodes = other.ProtoNodes.ToList();
            ProtoBeams = other.ProtoBeams.ToList();
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

        public void Update()
        {
            if (Iteration == MaxStep) return;
            //for(int i = 0; i < BeamCount; i++) Beam[i].Update();
            //for (int i = 0; i < ForcedNodesIndexes.Length; i++) Node[i].UpdateForces();
            for(int i = 0; i < BeamCount; i++)
            {
                double delta_len = Triple.Distance(Nodes[Beams[i].StartNode].Position, Nodes[Beams[i].EndNode].Position) 
                    - Beams[i].InitialLength;
                // deltaLen <0 : newLen is shorter --> compression --> InternalForce should be >0, vice versa
                Beams[i].InternalForce = -Beams[i].SpringConstant * delta_len;
                // vector from start to end
                Triple vector_force = new Triple(Nodes[Beams[i].StartNode].Position, Nodes[Beams[i].EndNode].Position, Beams[i].InternalForce);
                // comp -> int mosbat -> vector force mosbat be samte end;
                Nodes[Beams[i].StartNode].Force -= vector_force;
                Nodes[Beams[i].EndNode].Force += vector_force;

            }
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


namespace Truss
{
    /*public class DisjointSet
    {
        private int[] Par;
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

    }*/

    public class CompiledTruss
    {
        double Delta;
        int StepCount;
        int MaxStep;
        int NodeCount, BeamCount;
        // nodes
        Point3d[] Pos0;
        Point3d[] Position;
        Vector3d[] Velocity;
        Vector3d[] Acceleration;  /// maybe redundant
        double[] Mass;
        Vector3d[] ConstantForce;
        Vector3d[] Force;
        int[] SupportType;
        Vector3d[] ReactionForce;
        // beams
        int[] StartPoint;
        int[] EndPoint;
        double[] InitialLength;
        double[] SpringConstant;
        double[] InternalForce; // +: compression beam: push nodes, -: tension beam: pull nodes
        internal CompiledTruss(List<Truss.Node> nodes, List<Truss.Beam> beams, double delta_time, int max_step) {
            Delta = delta_time;
            StepCount = 0;
            MaxStep = max_step;
            NodeCount = nodes.Count;
            BeamCount = beams.Count;
            // nodes
            Pos0 = new Point3d[nodes.Count];
            Position = new Point3d[nodes.Count];
            Velocity = new Vector3d[nodes.Count];
            Acceleration = new Vector3d[nodes.Count];
            Mass = new double[nodes.Count];
            ConstantForce = new Vector3d[nodes.Count];
            Force = new Vector3d[nodes.Count];
            SupportType = new int[nodes.Count];
            ReactionForce = new Vector3d[nodes.Count];

            for(int i = 0; i < nodes.Count; i++)
            {
                Pos0[i] = nodes[i].pos;
                Position[i] = nodes[i].pos;
                SupportType[i] = nodes[i].support_type;
                Mass[i] = nodes[i].mass;
                ConstantForce[i] = nodes[i].force;
            }

            // beams
            StartPoint = new int[beams.Count];
            EndPoint = new int[beams.Count];
            InitialLength = new double[beams.Count];
            SpringConstant = new double[beams.Count];
            InternalForce = new double[beams.Count]; // most important output!!
            for (int i = 0; i < beams.Count; i++)
            {
                StartPoint[i] = beams[i].link.Item1;
                EndPoint[i] = beams[i].link.Item2;
                InitialLength[i] = beams[i].length;
                SpringConstant[i] = beams[i].stifnees;
            }

        }

        public CompiledTruss(CompiledTruss other)
        {
            // Copy Constructor
            Delta = other.Delta;
            StepCount = other.StepCount;
            MaxStep = other.MaxStep;
            NodeCount = other.NodeCount;
            BeamCount = other.BeamCount;
            Pos0 = (Point3d[]) other.Pos0.Clone();
            Position = (Point3d[]) other.Position.Clone();
            Velocity = (Vector3d[]) other.Velocity.Clone();
            Acceleration = (Vector3d[]) other.Acceleration.Clone();
            Mass = (double[]) other.Mass.Clone();
            ConstantForce = (Vector3d[]) other.ConstantForce.Clone();
            Force = (Vector3d[]) other.Force.Clone();
            SupportType = (int[]) other.SupportType.Clone();
            ReactionForce = (Vector3d[]) other.ReactionForce.Clone();
            StartPoint = (int[]) other.StartPoint.Clone();
            EndPoint = (int[]) other.EndPoint.Clone();
            InitialLength = (double[]) other.InitialLength.Clone();
            SpringConstant = (double[]) other.SpringConstant.Clone();
            InternalForce = (double[]) other.InternalForce.Clone();

        }

        public void Update(double damper) {
            // delete all forces. 
            /*for (int i = 0; i < NodeCount; i++)
            {
                Force[i] = new Vector3d(0, 0, 0);
            }*/

            // update beams, point come, force goes
            for (int i = 0; i < BeamCount; i++)
            {
                double newLen = Position[StartPoint[i]].DistanceTo(Position[EndPoint[i]]);
                double deltaLen = newLen - InitialLength[i];
                
                // deltaLen <0 : newLen is shorter --> compression --> InternalForce should be >0, vice versa
                InternalForce[i] = -SpringConstant[i] * deltaLen;
                Vector3d VectorForce = Position[EndPoint[i]] - Position[StartPoint[i]]; // vector from start to end
                VectorForce.Unitize();
                VectorForce*= InternalForce[i]; // comp -> int mosbat -> vector force mosbat be samte end;
                Force[StartPoint[i]] -= VectorForce;
                Force[EndPoint[i]] += VectorForce;
            }

            // update nodes, force come, point goes
            for(int i = 0; i < NodeCount; i++)
            {
                Force[i] += ConstantForce[i] - Velocity[i] * damper;
                ReactionForce[i]= new Vector3d(0, 0, 0);
                /// Update ReactionForces, eliminate Force
                if ((SupportType[i] & 1)  != 0 && Force[i].Z > 0) ReactionForce[i].Z = -Force[i].Z;
                if ((SupportType[i] & 2)  != 0 && Force[i].Y > 0) ReactionForce[i].Y = -Force[i].Y;
                if ((SupportType[i] & 4)  != 0 && Force[i].X > 0) ReactionForce[i].X = -Force[i].X;
                if ((SupportType[i] & 8)  != 0 && Force[i].Z < 0) ReactionForce[i].Z = -Force[i].Z;
                if ((SupportType[i] & 16) != 0 && Force[i].Y < 0) ReactionForce[i].Y = -Force[i].Y;
                if ((SupportType[i] & 32) != 0 && Force[i].X < 0) ReactionForce[i].X = -Force[i].X;
                Force[i] += ReactionForce[i];
                ///
                Acceleration[i] = Force[i] / Mass[i]; // f = m.a
                Velocity[i] += Acceleration[i] * Delta;
                Position[i] += Velocity[i] * Delta;
                //ReactionForce[i] !!!!
                Force[i] = new Vector3d(0, 0, 0); // make it zero, for the next loop!
            }
            // node outputs: REACTION FORCE, POSITION
            // beam outputs: INTERNAL FORCE
        }
        public void Reset() { }
        public SolvedTruss Output()
        {
            return new SolvedTruss(Position, ReactionForce, StartPoint, EndPoint, InternalForce);
        }

    }

    public class SolvedTruss {
        public List<Point3d> Nodes;
        public List<Vector3d> NodeReactForce;
        public List<Tuple<int, int>> Beams;
        public List<double> BeamInternalForce;
        internal SolvedTruss(Point3d[] pos, Vector3d[] react_force, int[] start_point, int[] end_point, double[] internal_force)
        {
            Nodes = new List<Point3d>();
            NodeReactForce = new List<Vector3d>();
            Beams = new List<Tuple<int, int>>();
            BeamInternalForce = new List<double>();

            for(int i = 0; i < pos.Count(); i++)
            {
                Nodes.Add(pos[i]);
                NodeReactForce.Add(react_force[i]);
            }
            for(int i = 0;i < start_point.Count();i++)
            {
                Beams.Add(new Tuple<int, int>(start_point[i], end_point[i]));
                BeamInternalForce.Add(internal_force[i]);
            }
            
        }
    }

    public class Truss
    {
        // Constants
        const double DefaultStiffness = 500;
        const double DefaultMass = 0.5;
        const double DefaultDeltaTime = 0.5;
        const int DefaultStepCount = 100;

        // Classes
        internal class Node
        {
            public Point3d pos;
            public Vector3d force;
            public bool[] support;
            public int support_type;
            public double mass;
            public Node(Point3d p, double mass_ = DefaultMass)
            {
                pos = p;
                force = new Vector3d(0, 0, 0);
                support = new bool[6];
                support_type = 0;
                mass = mass_;
            }
            public Node(Node node)
            {
                // copy constructor
                pos = node.pos;
                force = node.force;
                support = (bool[])(node.support.Clone());
                support_type = node.support_type;
                mass = node.mass;
            }
        }
        internal class Beam
        {
            public Tuple<int, int> link;
            public double stifnees;
            public double length { get; private set; }
            public Beam(int a, int b, double initial_lenght, double stiff = DefaultStiffness)
            {
                link = new Tuple<int, int>(a, b);
                stifnees = stiff;
                length = initial_lenght;
            }
            public Beam(Beam beam)
            {
                // copy constructor
                link = beam.link;
                stifnees = beam.stifnees;
                length = beam.length;
            }
        }
        

        // Members
        private List<Node> Nodes;
        private List<Beam> Beams;
        private double DeltaTime = DefaultDeltaTime;
        private double Damping = 5;

        // Constructors
        public Truss(List<Line> beam_lines, double tolerance)
        {
            Nodes = new List<Node>();
            Beams = new List<Beam>();

            List<Point3d> points = new List<Point3d>();
            Liz.DisjointSet dst = new Liz.DisjointSet(beam_lines.Count * 2);

            foreach (Line t in beam_lines)
            {
                points.Add(t.From);
                points.Add(t.To);
            }
            for (int i = 0; i < points.Count; i++)
            {
                for (int j = i + 1; j < points.Count; j++)
                {
                    if (dst.FindParent(i) != dst.FindParent(j) && points[i].DistanceTo(points[j]) < tolerance)
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
                    Nodes.Add(new Node(points[i]));
                    co_node[i] = Nodes.Count - 1;
                }
            }

            for (int i = 0; i < points.Count; i += 2)
            {
                Beams.Add(new Beam(
                  co_node[dst.FindParent(i)],
                  co_node[dst.FindParent(i + 1)],
                  Nodes[co_node[dst.FindParent(i)]].pos.DistanceTo(Nodes[co_node[dst.FindParent(i + 1)]].pos)));
            }
        }

        public Truss(Truss truss)
        {
            // Copy Constructor
            DeltaTime = truss.DeltaTime;
            Nodes = new List<Node>();
            Beams = new List<Beam>();
            foreach(Node node in truss.Nodes)
            {
                Nodes.Add(new Node(node));
            }
            foreach(Beam beam in truss.Beams)
            {
                Beams.Add(new Beam(beam));
            }
        }




        // Methods


        public void SetDeltaTime(double tt)
        {
            DeltaTime = tt;
        }

        public void AddForce(Point3d p, Vector3d v, double strenght)
        {
            v.Unitize();
            Nodes[NearestNode(p)].force += v * strenght;

        }

        public void SetMassOnNode(double mass)
        {
            for (int i = 0; i < Nodes.Count; i++) Nodes[i].mass = mass;
        }

        public void SetStiffnessOnBeam(double stiff)
        {
            for (int i = 0; i < Beams.Count; i++) Beams[i].stifnees = stiff;
        }

        public void AddSupport(Point3d p, int type)
        {
            int index = NearestNode(p);
            Nodes[index].support_type |= type;
            for (int i = 5; i >= 0; i--)
            {
                Nodes[index].support[i] |= (type % 2 == 1);
                type /= 2;
            }
        }

        public CompiledTruss Compile()
        {
            //CompiledTruss(List < Truss.Node > nodes, List < Truss.Beam > beams, double delta_time, int max_step)
            return new CompiledTruss(Nodes, Beams, DeltaTime, DefaultStepCount);
        }

        // Output data
        public List<Point3d> NodePoints()
        {
            var ret = new List<Point3d>();
            foreach (Node n in Nodes) ret.Add(n.pos);
            return ret;
        }
        public List<Tuple<int, int>> Connections()
        {
            var ret = new List<Tuple<int, int>>();
            foreach (Beam b in Beams) ret.Add(b.link);
            return ret;
        }
        public List<Vector3d> Forces()
        {
            var ret = new List<Vector3d>();
            foreach (Node n in Nodes) ret.Add(n.force);
            return ret;
        }
        public List<bool[]> Supports()
        {
            var ret = new List<bool[]>();
            foreach (Node n in Nodes) ret.Add(n.support);
            return ret;
        }


        // Private Methods
        private int NearestNode(Point3d p)
        {
            int min_ind = 0;
            double min_dis = p.DistanceTo(Nodes[0].pos);
            for (int i = 1; i < Nodes.Count; i++)
            {
                if (p.DistanceTo(Nodes[i].pos) < min_dis)
                {
                    min_dis = p.DistanceTo(Nodes[i].pos);
                    min_ind = i;
                }
            }
            return min_ind;
        }



    }

}
