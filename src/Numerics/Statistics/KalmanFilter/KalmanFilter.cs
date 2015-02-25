using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    /// <summary>
    /// 
    /// The Kalman Filter (http://en.wikipedia.org/wiki/Kalman_filter)
    /// 
    /// This implementation does a single update starting from the prior state and observation.
    /// The class is instantiated by taking the transition model in as a constructor argument.
    /// The transition model is an implementation of IKalmanModel, and its only member is a 
    /// method calculates the a priori state using the prior state as an argument.
    /// 
    /// The observation data used to compure the Kalman update is embedded in the
    /// KalmanObservationData object. This contains both the data, and the transformation
    /// necessary to map observations and their observation error to KalmanState objects.
    /// 
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class KalmanFilter<T> where T : struct, IEquatable<T>, IFormattable
    {
        IKalmanModel<T> f;

        public KalmanFilter(IKalmanModel<T> transition_model)
        {
            this.f = transition_model;
        }

        public KalmanState<T> Update(KalmanState<T> prior_state, KalmanObservationData<T> observation)
        {
            // I made every effort to keep the naming convention consistent with http://en.wikipedia.org/wiki/Kalman_filter#Details
            var k_state = new KalmanState<T>(); // to be the updated value of x

            var x_apriori = this.f.PredictState(prior_state);
            var y = observation.Observation - observation.StateTransform(x_apriori);
            var h_gradient = observation.StateTransformGradient(x_apriori);
            var s = h_gradient * x_apriori.Covariance * h_gradient.Transpose() + observation.ObservationError;

            var k_gain = s.Cholesky().Solve(h_gradient * x_apriori.Covariance.Transpose()).Transpose();
            k_state.State = x_apriori.State + k_gain * y;
            k_state.Covariance = (Matrix<T>.Build.DenseIdentity(k_state.State.Count) - k_gain * h_gradient) * x_apriori.Covariance;

            return k_state;
        }
    }
}
