using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    public class ExtendedKalmanModel<T> : IKalmanModel<T> where T : struct, IEquatable<T>, IFormattable 
    {
        Func<Vector<T>, Matrix<T>> f;
        Func<KalmanState<T>, Matrix<T>> sc;

        public ExtendedKalmanModel(Func<Vector<T>, Matrix<T>> transition_model, Func<KalmanState<T>, Matrix<T>> system_covariance)
        {
            this.f = transition_model;
            this.sc = system_covariance;
        }

        public KalmanState<T> PredictState(KalmanState<T> prior_state)
        {
            var approx = this.f(prior_state.State);
            var approx_sc = this.sc(prior_state);

            var km = new KalmanModel<T>(approx, approx_sc);
            return km.PredictState(prior_state);
        }
    }
}
