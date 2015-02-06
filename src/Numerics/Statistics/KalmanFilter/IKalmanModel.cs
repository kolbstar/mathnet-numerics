using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Statistics.KalmanFilter
{
    /// <summary>
    /// 
    /// IKalmanModel defines the transition model of the Kalman Filter. Its only member is a 
    /// method calculates the `a priori` state using the prior state as an argument. For a standard,
    /// linear Kalman Filter, this predict state merely muliples the prior state by a transition
    /// matrix (see class KalmanModel). For the Extended Kalman Filter, non-linear characteriscs can
    /// be handled individually (see class ExtendedKalmanModel).
    /// 
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public interface IKalmanModel<T> where T : struct, IEquatable<T>, IFormattable
    {
        KalmanState<T> PredictState(KalmanState<T> prior_state);
    }
}
