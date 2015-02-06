using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    public class ExtendedKalmanData<T> : KalmanObservationData<T> where T : struct, IEquatable<T>, IFormattable 
    {
        Func<Vector<T>, Vector<T>> h;
        Func<Vector<T>, Matrix<T>> gradient_h;

        public ExtendedKalmanData(Func<Vector<T>, Vector<T>> h, Func<Vector<T>, Matrix<T>> gradient_h)
        {
            this.h = h;
            this.gradient_h = gradient_h;
        }

        public override Vector<T> StateTransform(KalmanState<T> state)
        {
            return h(state.State);
        }

        public override Matrix<T> StateTransformGradient(KalmanState<T> state)
        {
            return gradient_h(state.State);
        }
    }
}
