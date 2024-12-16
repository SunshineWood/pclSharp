using System;
using System.Runtime.InteropServices;

namespace Util
{
    public class PclSharp
    {
        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr radius_down_sampling_run([In] float[] cloud, int rows, ref uint size);
        
        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr statistical_outlier_filter([In] float[] cloud, int rows,int meanK, float stddevMulThresh, ref uint size);

        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern void fit_circle([In] float[] cloud, int rows, ref float diameter);

        [DllImport(".\\PclSharp.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr smooth_filter([In] float[] cloud, int rows,int polynomialOrder,float searchRadius, ref uint size);

        [DllImport(".\\PclSharp.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void FreeArray(IntPtr arr);
        
        public static float[] RadiusDownSamplingRun(float[] cloud, out uint size)
        {
            size = 0;
            var ptr = radius_down_sampling_run(cloud, cloud.Length / 3, ref size);
            float[] managedArray = new float[size*3];
            Marshal.Copy(ptr, managedArray, 0, (int)size*3);
            FreeArray(ptr);
            return managedArray;
        }
        
        public static float[] StatisticalOutlierFilter(float[] cloud,int meanK, float stddevMulThresh, out uint size)
        {
            size = 0;
            var ptr = statistical_outlier_filter(cloud, cloud.Length / 3,meanK, stddevMulThresh, ref size);
            float[] managedArray = new float[size*3];
            Marshal.Copy(ptr, managedArray, 0, (int)(size*3));
            FreeArray(ptr);
            return managedArray;
        }

        public static void FitCircle(float[] cloud, out float diameter)
        {
            diameter = 0.0f;
            fit_circle(cloud, cloud.Length / 2, ref diameter);
        }

        public static float[] SmoothFilter(float[] cloud,int polynomialOrder, float searchRadius, out uint size)
        {
            size = 0;
            var ptr = smooth_filter(cloud, cloud.Length / 3,polynomialOrder, searchRadius, ref size);
            float[] managedArray = new float[size*3];
            Marshal.Copy(ptr, managedArray, 0, (int)(size*3));
            FreeArray(ptr);
            return managedArray;
        }
    }
}