using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace Util
{
    public class FileHelper
    {
        public static List<Point3D> LoadFileData(string fileName)
        {
            IEnumerable<string> lines = File.ReadLines(fileName);
            List<Point3D> currentStepProfile = new List<Point3D>();
            foreach (string line in lines)
            {
                var pArray = line.Split(' ').ToList();
                currentStepProfile.Add(new Point3D(float.Parse(pArray[0]), float.Parse(pArray[1]),
                    float.Parse(pArray[2])));
            }
            return currentStepProfile;
        }
    }

    public struct Point3D
    {
        public Point3D(float x, float y, float z)
        {
            X=x; Y=y; Z=z;
        }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }
}