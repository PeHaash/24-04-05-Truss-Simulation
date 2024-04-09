using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino;
using Rhino.Geometry;
using static Truss.Truss;

/*using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;*/

namespace Liz
{
    public struct Point
    {
        double x, y, z; 
    } 
    public struct pointInSpace { }
    public struct Node {
        Point3d Pos0;
        Point3d Position;
        Vector3d Velocity;
        double Mass;
        Vector3d ConstantForce; // only on some nodes!
        Vector3d Force;
        int SupportType;
        Vector3d ReactionForce; // only on some nodes!
    }
    public struct Beam {
        int StartNode;
        int EndNode;
        public double InitialLength; // { get; private set; }
        double SpringConstant;
        double InternalForce; // +: compression beam: push nodes, -: tension beam: pull nodes
    }
    public class Truss
    {
        // compiled parts: data that can be manipulated!
        double Delta;
        int StepCount, MaxStep, NodeCount, BeamCount;
        Node[] Nodes;
        Beam[] Beams;



    }
}


namespace Truss
{
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
            DisjointSet dst = new DisjointSet(beam_lines.Count * 2);

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
