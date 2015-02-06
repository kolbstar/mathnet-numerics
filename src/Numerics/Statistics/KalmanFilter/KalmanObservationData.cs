using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    public abstract class KalmanObservationData<T> where T : struct, IEquatable<T>, IFormattable
    {
        public Vector<T> Observation { get; set; }
        public Matrix<T> ObservationError { get; set; }
        abstract public Vector<T> StateTransform(KalmanState<T> state);
        abstract public Matrix<T> StateTransformGradient(KalmanState<T> state);
    }
}
