using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
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
            string folderPath = AppDomain.CurrentDomain.BaseDirectory + "\\TestData\\";
            string[] plyFiles = Directory.GetFiles(folderPath, "*.ply");
            List<Point3D> allPoints = new List<Point3D>();
            foreach (string file in plyFiles)
            {
                var ps3D = FileHelper.LoadFileData(file);
                allPoints.AddRange(ps3D);
            }
            float[] cloudInput = new float[allPoints.Count * 3];
            for (int i = 0; i < allPoints.Count; i++)
            {
                cloudInput[i * 3 + 0] = (float)allPoints[i].X;
                cloudInput[i * 3 + 1] = (float)allPoints[i].Y;
                cloudInput[i * 3 + 2] = (float)allPoints[i].Z;
            }
            var x=PclSharp.StatisticalOutlierFilter(cloudInput, 80,0.6f,out var size);
        }


    }
}
