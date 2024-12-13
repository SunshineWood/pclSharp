using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.IO;
using System.Threading;
using Util;

namespace UnitTest
{
    [TestClass]
    public class UnitTest1
    {
        [TestMethod]
        public void FitCircleTest()
        {
            string executionPath = AppDomain.CurrentDomain.BaseDirectory;
            var files = Directory.GetFiles(executionPath, "*-*.txt");
            foreach (var file in files)
            {
                var ps3D = FileHelper.LoadFileData(file);
                float[] cloudInput = new float[ps3D.Count * 2];
                for (int i = 0; i < ps3D.Count; i++)
                {
                    cloudInput[i * 2 + 0] = (float)ps3D[i].X;
                    cloudInput[i * 2 + 1] = (float)ps3D[i].Y;
                }
                PclSharp.FitCircle(cloudInput, out float diameter);
                Console.WriteLine($"File: {Path.GetFileName(file)}, Diameter: {diameter}");
            }
        }

        [TestMethod]
        public void SmoothFilterTest()
        {
            var ps3D = FileHelper.LoadFileData("Test.txt");
            float[] cloudInput = new float[ps3D.Count * 3];
            for (int i = 0; i < ps3D.Count; i++)
            {
                cloudInput[i * 3 + 0] = (float)ps3D[i].X;
                cloudInput[i * 3 + 1] = (float)ps3D[i].Y;
                cloudInput[i * 3 + 2] = (float)ps3D[i].Z;
            }
            var xx= PclSharp.SmoothFilter(cloudInput, 2,0.001f,out var size);
        }

        [TestMethod]
        public void StatisticalOutlierFilterTest()
        {
            var ps3D = FileHelper.LoadFileData("Test.txt");
            float[] cloudInput = new float[ps3D.Count * 3];
            for (int i = 0; i < ps3D.Count; i++)
            {
                cloudInput[i * 3 + 0] = (float)ps3D[i].X;
                cloudInput[i * 3 + 1] = (float)ps3D[i].Y;
                cloudInput[i * 3 + 2] = (float)ps3D[i].Z;
            }
            var x=PclSharp.StatisticalOutlierFilter(cloudInput, 80,0.6f,out var size);
        }


    }
}
