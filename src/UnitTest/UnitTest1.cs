using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.IO;
using Util;

namespace UnitTest
{
    [TestClass]
    public class UnitTest1
    {
        [TestMethod]
        public void FitCircle()
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
    }
}
