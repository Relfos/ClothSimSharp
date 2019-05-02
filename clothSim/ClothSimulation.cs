using OpenTK;
using System;
using System.Collections.Generic;

namespace LunarLabs.Cloth
{
    public struct Triangle
    {
        public int A;
        public int B;
        public int C;
        public Vector3 normal;
    }

    public struct Vertex
    {
        public Vector3 position;
        public Vector3 normal;
        public Vector2 uv;
    }

    public struct ClothSpring
    {
        //Indices of the Particles at either end of the spring
        public int P1;
        public int P2;

        public float _NaturalLength;
        public float _InverseLength;

        public float _Stiffness;

        public ClothSpring(int PID1, int PID2, float Len, float Stiffness)
        {
            this.P1 = PID1;
            this.P2 = PID2;
            this._NaturalLength = Len;
            this._InverseLength = 1.0f / Len;
            this._Stiffness = Stiffness;
        }
    }

    public struct ClothParticle
    {
        public Vector3 currentPosition;
        public Vector3 currentVelocity;

        public Vector3 nextPosition;
        public Vector3 nextVelocity;

        public Vector3 tension;
        public float inverseMass;
        
        public bool pinned; //true if particle is pinned / fixed in position
    }

    public struct ClothCollider
    {
        public Vector3 Position;
        public float Radius;

        public ClothCollider(Vector3 position, float radius)
        {
            this.Position = position;
            this.Radius = radius;
        }
    }

    public class ClothSimulation
    {
        private const int SimScale = 1;

        private const float minimumPhysicsDelta = 0.01f; // in seconds

        // size of the cloth mesh
        private const float clothScale = 20.0f; //10        

        //Values given to each spring
        private float StretchStiffness = 2.5f * clothScale;
        private float BendStiffness = 1.0f * clothScale;

        //Values given to each ball
        private float mass = 0.01f * SimScale;

        //Damping factor. Velocities are multiplied by this
        private float dampFactor = 0.9f;

        //Grid complexity. This is the number of Particles across and down in the model
        public const int gridSize = 13 * SimScale; //13;

        private ClothSpring[] _springs;
        private ClothParticle[] _particles;

        private float _timeSinceLastUpdate;

        private Vector3 _gravity;

        private List<ClothCollider> _colliders = new List<ClothCollider>();

        private Vertex[] _vertices;
        public Vertex[] vertices { get { return _vertices; } }

        private Triangle[] _triangles;
        public Triangle[] triangles { get { return _triangles; } }

        public ClothSimulation()
        {
            _gravity = new Vector3(0, -0.98f * SimScale, 0);

            //Calculate number of Particles
            int particleCount = gridSize * gridSize;

            //Calculate number of springs
            //There is a spring pointing right for each ball which is not on the right edge,
            //and one pointing down for each ball not on the bottom edge
            int springCount = (gridSize - 1) * gridSize * 2;

            //There is a spring pointing down & right for each ball not on bottom or right,
            //and one pointing down & left for each ball not on bottom or left
            springCount += (gridSize - 1) * (gridSize - 1) * 2;

            //There is a spring pointing right (to the next but one ball)
            //for each ball which is not on or next to the right edge,
            //and one pointing down for each ball not on or next to the bottom edge
            springCount += (gridSize - 2) * gridSize * 2;

            //Create space for Particles & springs
            _particles = new ClothParticle[particleCount];
            _springs = new ClothSpring[springCount];

            this.InitMesh();
            this.Reset();
        }

        private void InitMesh()
        {
            int triangleCount = (gridSize * gridSize) * 2;
            _triangles = new Triangle[triangleCount];

            int vertexCount = (gridSize * gridSize);
            _vertices = new Vertex[vertexCount];

            int k = 0;
            for (int j = 0; j < gridSize - 1; j++)
            {
                for (int i = 0; i < gridSize - 1; i++)
                {
                    var i0 = j * gridSize + i;
                    var i1 = j * gridSize + i + 1;
                    var i2 = (j + 1) * gridSize + i;
                    var i3 = (j + 1) * gridSize + i + 1;

                    _triangles[k].A = i2;
                    _triangles[k].B = i1;
                    _triangles[k].C = i0;
                    k++;

                    _triangles[k].A = i2;
                    _triangles[k].B = i3;
                    _triangles[k].C = i1;
                    k++;
                }
            }

            for (int j = 0; j < gridSize; j++)
            {
                for (int i = 0; i < gridSize; i++)
                {
                    int ballID = j * gridSize + i;
                    _vertices[ballID].uv = new Vector2(i / (float)gridSize, j / (float)gridSize);
                }
            }
        }

        public void Reset()
        {
            //Initialise the Particles in an evenly spaced grid in the x-z plane
            for (int j = 0; j < gridSize; j++)
            {
                for (int i = 0; i < gridSize; i++)
                {
                    float U = (i / (float)(gridSize - 1)) - 0.5f;
                    float V = (j / (float)(gridSize - 1)) - 0.5f;

                    int BallID = j * gridSize + i;
                    _particles[BallID].currentPosition = new Vector3((float)clothScale * U, 8.5f, (float)clothScale * V);
                    _particles[BallID].currentVelocity = Vector3.Zero;

                    _particles[BallID].inverseMass = 1.0f / mass;
                    _particles[BallID].pinned = false;

                    _particles[BallID].tension = Vector3.Zero;
                }
            }

            float naturalLength = (_particles[0].currentPosition - _particles[1].currentPosition).LengthFast;

            //Fix the top left & top right Particles in place
            _particles[0].pinned = true;
            _particles[gridSize - 1].pinned = true;

            //Fix the bottom left & bottom right Particles
            _particles[gridSize * (gridSize - 1)].pinned = true;
            _particles[gridSize * gridSize - 1].pinned = true;

            //Initialise the springs
            int currentSpring = 0;

            //The first (gridSize-1)*gridSize springs go from one ball to the next, excluding those on the right hand edge
            for (int j = 0; j < gridSize; j++)
                for (int i = 0; i < gridSize - 1; i++)
                {
                    _springs[currentSpring] = new ClothSpring(j * gridSize + i, j * gridSize + i + 1, naturalLength, StretchStiffness);
                    currentSpring++;
                }

            //The next (gridSize-1)*gridSize springs go from one ball to the one below, excluding those on the bottom edge
            for (int j = 0; j < gridSize - 1; j++)
                for (int i = 0; i < gridSize; i++)
                {
                    _springs[currentSpring] = new ClothSpring(j * gridSize + i, (j + 1) * gridSize + i, naturalLength, StretchStiffness);
                    currentSpring++;
                }

            //The next (gridSize-1)*(gridSize-1) go from a ball to the one below and right, excluding those on the bottom or right
            for (int j = 0; j < gridSize - 1; j++)
                for (int i = 0; i < gridSize - 1; i++)
                {
                    _springs[currentSpring] = new ClothSpring(j * gridSize + i, (j + 1) * gridSize + i + 1, naturalLength * (float)Math.Sqrt(2.0f), BendStiffness);
                    currentSpring++;
                }

            //The next (gridSize-1)*(gridSize-1) go from a ball to the one below and left, excluding those on the bottom or right
            for (int j = 0; j < gridSize - 1; j++)
                for (int i = 1; i < gridSize; i++)
                {
                    _springs[currentSpring] = new ClothSpring(j * gridSize + i, (j + 1) * gridSize + i - 1, naturalLength * (float)Math.Sqrt(2.0f), BendStiffness);
                    currentSpring++;
                }

            //The first (gridSize-2)*gridSize springs go from one ball to the next but one, excluding those on or next to the right hand edge
            for (int j = 0; j < gridSize; j++)
                for (int i = 0; i < gridSize - 2; i++)
                {
                    _springs[currentSpring] = new ClothSpring(j * gridSize + i, j * gridSize + i + 2, naturalLength * 2, BendStiffness);
                    currentSpring++;
                }


            //The next (gridSize-2)*gridSize springs go from one ball to the next but one below, excluding those on or next to the bottom edge
            for (int j = 0; j < gridSize - 2; j++)
                for (int i = 0; i < gridSize; i++)
                {
                    _springs[currentSpring] = new ClothSpring(j * gridSize + i, (j + 2) * gridSize + i, naturalLength * 2, BendStiffness);
                    currentSpring++;
                }

            UpdateMesh();
        }

        private static Vector3 CalculateNormal(Vector3 VA, Vector3 VB, Vector3 VC)
        {
            Vector3 a, b;

            a = VB - VA;
            b = VC - VA;

            var normal = Vector3.Cross(a, b);
            normal.Normalize();
            return normal;
        }

        private void UpdateMesh()
        {
            // calculate triangle normals 
            for (int i=0; i<_triangles.Length; i++)
            {
                var t = _triangles[i];
                var A = vertices[t.A].position;
                var B = vertices[t.B].position;
                var C = vertices[t.C].position;
                _triangles[i].normal = CalculateNormal(A, B, C);
            }

            //Calculate the normals on the current Particles
            for (int j = 0; j < gridSize; j++)
                for (int i = 0; i < gridSize; i++)
                {
                    int BallID = j * gridSize + i;
                    Vector3 normal = Vector3.Zero;
                    int count = 0;
                    for (int Y = 0; Y <= 1; Y++)
                        for (int X = 0; X <= 1; X++)
                        {
                            if (X + i < gridSize && Y + j < gridSize)
                            {
                                int Index = (j + Y) * gridSize + (i + X) * 2;
                                normal += _triangles[Index].normal;

                                Index++;
                                normal += _triangles[Index].normal;

                                count += 2;
                            }

                        }

                    //normal.Normalize();
                    normal /= (float)count;
                    _vertices[BallID].normal = normal;
                }

            for (int j = 0; j < gridSize; j++)
                for (int i = 0; i < gridSize; i++)
                {
                    int BallID = j * gridSize + i;
                    _vertices[BallID].position = _particles[BallID].currentPosition;
                }


            // TODO .UpdateBoundingBox();
        }



        public void Simulate(float deltaTime)
        {
            //if no time passed, there's nothing to update
            if (deltaTime <= 0)
            {
                return;
            }

            //Update the physics in intervals of 10ms to prevent problems with different frame rates causing different damping
            _timeSinceLastUpdate += deltaTime;

            bool updateMade = false;    //did we update the positions etc this time?

            float timePassedInSeconds = minimumPhysicsDelta;

            while (_timeSinceLastUpdate > minimumPhysicsDelta)
            {

                _timeSinceLastUpdate -= minimumPhysicsDelta;
                updateMade = true;

                //Calculate the tensions in the springs
                for (int i = 0; i < _springs.Length; i++)
                {
                    Vector3 tensionDirection = (_particles[_springs[i].P1].currentPosition - _particles[_springs[i].P2].currentPosition);

                    float springLength = tensionDirection.LengthFast;
                    float extension = springLength - _springs[i]._NaturalLength;

                    float tension = _springs[i]._Stiffness * (extension * _springs[i]._InverseLength);

                    tensionDirection *= (float)(tension / springLength);

                    _particles[_springs[i].P2].tension += tensionDirection;
                    _particles[_springs[i].P1].tension -= tensionDirection;
                }


                //Calculate the nextParticles from the currentParticles
                for (int i = 0; i < _particles.Length; i++)
                {
                    //If the ball is fixed, transfer the position and zero the velocity, otherwise calculate the new values
                    if (_particles[i].pinned)
                    {
                        _particles[i].nextPosition = _particles[i].currentPosition;
                        _particles[i].nextVelocity = Vector3.Zero;
                        // If MoveCloth Then _Particles[i].NextPosition.Add(VectorCreate(0, 2 * timePassedInSeconds, 5 * timePassedInSeconds));
                        continue;
                    }

                    //Calculate the force on this ball
                    Vector3 force = _gravity + _particles[i].tension;

                    //Calculate the acceleration
                    Vector3 acceleration = force * (float)_particles[i].inverseMass;

                    //Update velocity
                    _particles[i].nextVelocity = _particles[i].currentVelocity + (acceleration * timePassedInSeconds);

                    //Damp the velocity
                    _particles[i].nextVelocity *= dampFactor;

                    //Calculate new position
                    //Forcse := VectorAdd(VectorScale(Particles[i].NextVelocity, 0.5), VectorScale(Particles[i].CurrentVelocity, 0.5));
                    force = _particles[i].nextVelocity * timePassedInSeconds;

                    _particles[i].nextPosition = _particles[i].currentPosition + force;

                    //Check against floor
                    /* if(_Particles[i].NextPosition.y <= -sphereRadius* 0.92f) 
                    {

                          _Particles[i].NextPosition.y := -sphereRadius* 0.92f;
                            _Particles[i].NextVelocity.y := 0;
                            continue;
                    }*/

                    //Check against colliders
                    for (int j = 0; j < _colliders.Count; j++)
                    {
                        Vector3 P = _particles[i].nextPosition - _colliders[j].Position;

                        float cR = _colliders[j].Radius * 1.08f;

                        if (P.LengthSquared < cR * cR)
                        {
                            P.Normalize();
                            P *= cR;
                            _particles[i].nextPosition = P + _colliders[j].Position;
                            _particles[i].nextVelocity = Vector3.Zero;
                            break;
                        }
                    }
                }

                //Swap the currentParticles and newParticles pointers
                for (int i = 0; i < _particles.Length; i++)
                {
                    _particles[i].currentPosition = _particles[i].nextPosition;
                    _particles[i].currentVelocity = _particles[i].nextVelocity;
                    _particles[i].tension = Vector3.Zero;
                }
            }

            //Update the mesh if we have updated the positions
            if (updateMade)
            {
                UpdateMesh();
            }
        }


        public void AddCollider(Vector3 position, float radius)
        {
            var col = new ClothCollider(position, radius);
            _colliders.Add(col);
        }

        public void UnpinParticle(int index)
        {
            _particles[index].pinned = false;
        }

    }

}
