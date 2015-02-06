using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    public class KalmanState<T> where T : struct, IEquatable<T>, IFormattable
    {
        public Vector<T> State { get; set; }
        public Matrix<T> Covariance { get; set; }
    }
}
