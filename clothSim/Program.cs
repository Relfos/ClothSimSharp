using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;

namespace LunarLabs.Cloth
{
    class Program
    {
        private static int mouseX;
        private static int mouseY;
        private static bool[] keys = new bool[256];

        private static Stopwatch timer = new Stopwatch();

        static int LoadImage(Bitmap image)
        {
            int texID = GL.GenTexture();

            GL.BindTexture(TextureTarget.Texture2D, texID);
            BitmapData data = image.LockBits(new System.Drawing.Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, data.Width, data.Height, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, data.Scan0);

            image.UnlockBits(data);

            GL.GenerateMipmap(GenerateMipmapTarget.Texture2D);


            return texID;
        }

        private static float GetTime()
        {
            return timer.ElapsedMilliseconds / 1000.0f;
        }

        [STAThread]
        public static void Main(string[] args)
        {
            int textureID = -1;

            var cloth = new ClothSimulation();
            cloth.AddCollider(Vector3.Zero, 5);

            float lastTime = GetTime();

            float curAngle = 0;
            bool mouseDown = false;

            timer.Start();

            using (var game = new GameWindow())
            {
                game.Load += (sender, e) =>
                {
                    game.Title = "Cloth Simulation";

                    game.VSync = VSyncMode.On;

                    textureID = LoadImage(new Bitmap(Image.FromFile("cloth.png")));
                };

                game.KeyDown += (sender, e) =>
                {
                    keys[(int)e.Key] = true;
                };

                game.KeyUp += (sender, e) =>
                {
                    keys[(int)e.Key] = false;
                };

                game.MouseMove += (sender, e) =>
                {
                    mouseX = e.X;
                    mouseY = e.Y;
                };

                game.MouseDown += (sender, e) =>
                {
                    if (e.Button == MouseButton.Left)
                    {
                        mouseDown = true;
                    }
                };

                game.MouseUp += (sender, e) =>
                {
                    if (e.Button == MouseButton.Left)
                    {
                        mouseDown = false;
                    }
                };

                game.Resize += (sender, e) =>
                {
                    GL.Viewport(0, 0, game.Width, game.Height);
                };

                game.UpdateFrame += (sender, e) =>
                {
                    float curTime = GetTime();
                    float delta = curTime - lastTime;
                    lastTime = curTime;
                    cloth.Simulate(delta);

                    if (mouseDown)
                    {
                        curAngle = (mouseX / (float)game.Width) * MathHelper.DegreesToRadians(360);
                    }


                    // demo controls
                    if (keys[(int)Key.Escape])
                    {
                        game.Exit();
                    }

                    if (keys[(int)Key.Space])
                    {
                        cloth.Reset();
                    }

                    if (keys[(int)Key.Number1])
                    {
                        cloth.UnpinParticle(0);
                    }

                    if (keys[(int)Key.Number2])
                    {
                        cloth.UnpinParticle(ClothSimulation.gridSize - 1);
                    }

                };

                game.RenderFrame += (sender, e) =>
                {
                    #region FRAME SETUP
                    GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                    GL.Enable(EnableCap.AlphaTest);
                    GL.AlphaFunc(AlphaFunction.Greater, 0.5f);

                    GL.Enable(EnableCap.DepthTest);
                    GL.DepthFunc(DepthFunction.Lequal);
                    #endregion

                    #region LIGHTING
                    float[] mat_specular = { 1.0f, 1.0f, 1.0f, 1.0f };
                    float[] mat_shininess = { 50.0f };
                    float[] light_position = { 1000.0f, 500.0f, 1000.0f, 100.0f };
                    float[] light_ambient = { 0.5f, 0.5f, 0.5f, 1.0f };

                    GL.Light(LightName.Light0, LightParameter.Position, light_position);
                    GL.Enable(EnableCap.Lighting);
                    GL.Enable(EnableCap.Light0);
                    GL.Enable(EnableCap.ColorMaterial);
                    #endregion

                    #region TEXTURING
                    GL.Enable(EnableCap.Texture2D);
                    GL.BindTexture(TextureTarget.Texture2D, textureID);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);
                    #endregion

                    #region PROJECTION MATRIX
                    float fov = MathHelper.DegreesToRadians(60);
                    float aspect = game.Width / (float)game.Height;
                    float zNear = 0.1f;
                    float zFar = 500;
                    var projMat = Matrix4.CreatePerspectiveFieldOfView(fov, aspect, zNear, zFar);
                    GL.MatrixMode(MatrixMode.Projection);
                    GL.LoadMatrix(ref projMat);
                    #endregion

                    #region CAMERA MATRIX
                    GL.MatrixMode(MatrixMode.Modelview);
                    var camPos = new Vector3(0, 15, 25);
                    var lookMat = Matrix4.LookAt(camPos, Vector3.Zero, Vector3.UnitY);
                    var modelMat = Matrix4.CreateRotationY(curAngle);
                    lookMat = modelMat * lookMat;
                    GL.LoadMatrix(ref lookMat);
                    #endregion

                    #region RENDERING
                    GL.Begin(PrimitiveType.Triangles);
                    foreach (var t in cloth.triangles)
                    {
                        var A = cloth.vertices[t.A];
                        var B = cloth.vertices[t.B];
                        var C = cloth.vertices[t.C];

                        GL.Color4(Color.White);
                        GL.Normal3(A.normal.X, A.normal.Y, A.normal.Z);
                        GL.TexCoord2(A.uv.X, A.uv.Y);
                        GL.Vertex3(A.position.X, A.position.Y, A.position.Z);

                        GL.Normal3(B.normal.X, B.normal.Y, B.normal.Z);
                        GL.TexCoord2(B.uv.X, B.uv.Y);
                        GL.Vertex3(B.position.X, B.position.Y, B.position.Z);

                        GL.Normal3(C.normal.X, C.normal.Y, C.normal.Z);
                        GL.TexCoord2(C.uv.X, C.uv.Y);
                        GL.Vertex3(C.position.X, C.position.Y, C.position.Z);
                    }
                    GL.End();
                    #endregion

                    game.SwapBuffers();
                };

                // Run the game at 60 updates per second
                game.Run(60.0);
            }
        }

    }
}
