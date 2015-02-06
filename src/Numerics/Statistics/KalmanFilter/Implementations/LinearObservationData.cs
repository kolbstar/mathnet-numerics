using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    public class LinearObservatinoData<T> : KalmanObservationData<T> where T : struct, IEquatable<T>, IFormattable 
    {
        public Matrix<T> ObservationMatrix { get; set; }

        public override Vector<T> StateTransform(KalmanState<T> state)
        {
            return ObservationMatrix.Multiply(state.State);
        }

        public override Matrix<T> StateTransformGradient(KalmanState<T> state)
        {
            return this.ObservationMatrix;
        }
    }
}
