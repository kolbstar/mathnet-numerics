using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    public class KalmanModel<T> : IKalmanModel<T> where T : struct, IEquatable<T>, IFormattable 
    {
        Matrix<T> f;
        Matrix<T> sc;

        public KalmanModel(Matrix<T> transition_model, Matrix<T> system_covariance)
        {
            this.f = transition_model;
            this.sc = system_covariance;
        }

        public KalmanState<T> PredictState(KalmanState<T> prior_state)
        {
            var ks = new KalmanState<T>();
            ks.State = this.f * prior_state.State;
            ks.Covariance = this.f * prior_state.Covariance * this.f.Transpose() + this.sc;
            return ks;
        }
    }

}
