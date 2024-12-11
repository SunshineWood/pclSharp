using System;
using System.Runtime.InteropServices;

namespace Util
{
    public class PclSharp
    {
        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr RadiusDownSamplingRun([In] float[] cloud, int rows, ref int size);
        
        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr StatisticalOutlierFilter([In] float[] cloud, int rows,int meanK, float stddevMulThresh, ref int size);

        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern void FitCircle([In] float[] cloud, int rows, ref float diameter);

        [DllImport(".\\PclSharp.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void FreeArray(IntPtr arr);
        
        public static float[] RadiusDownSamplingRun(float[] cloud, out int size)
        {
            size = 0;
            var ptr = RadiusDownSamplingRun(cloud, cloud.Length / 3, ref size);
            float[] managedArray = new float[size*3];
            Marshal.Copy(ptr, managedArray, 0, size*3);
            FreeArray(ptr);
            return managedArray;
        }
        
        public static float[] StatisticalOutlierFilter(float[] cloud,int meanK, float stddevMulThresh, out int size)
        {
            size = 0;
            var ptr = StatisticalOutlierFilter(cloud, cloud.Length / 3,meanK, stddevMulThresh, ref size);
            float[] managedArray = new float[size*3];
            Marshal.Copy(ptr, managedArray, 0, size*3);
            FreeArray(ptr);
            return managedArray;
        }

        public static void FitCircle(float[] cloud, out float diameter)
        {
            diameter = 0.0f;
            FitCircle(cloud, cloud.Length / 2, ref diameter);
        }
    }
}