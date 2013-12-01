﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MathNet.Numerics.Optimization
{
    public class GoldenSectionMinimizer
    {
        public double XTolerance { get; set; }
        public int MaximumIterations { get; set; }

        public GoldenSectionMinimizer(double x_tolerance=1e-5, int max_iterations=1000)
        {
            this.XTolerance = x_tolerance;
            this.MaximumIterations = max_iterations;
        }

        public MinimizationOutput1D FindMinimum(IObjectiveFunction1D objective, double lower_bound, double upper_bound)
        {
            if (!(objective is ObjectiveChecker1D))
                objective = new ObjectiveChecker1D(objective, this.ValueChecker, null, null);

            double middle_point_x = lower_bound + (upper_bound - lower_bound) / (1 + _golden_ratio);
            IEvaluation1D lower = objective.Evaluate(lower_bound);
            IEvaluation1D middle = objective.Evaluate(middle_point_x);
            IEvaluation1D upper = objective.Evaluate(upper_bound);
            
            if (upper_bound <= lower_bound)
                throw new OptimizationException("Lower bound must be lower than upper bound.");

            if (upper.Value < middle.Value || lower.Value < middle.Value)
                throw new OptimizationException("Lower and upper bounds do not necessarily bound a minimum.");

            int iterations = 0;
            while (Math.Abs(upper.Point - lower.Point) > this.XTolerance && iterations < this.MaximumIterations)
            {
                double test_x = lower.Point + (upper.Point - middle.Point);
                var test = objective.Evaluate(test_x);

                if (test.Point < middle.Point)
                {
                    if (test.Value > middle.Value)
                    {
                        lower = test;
                    }
                    else
                    {
                        upper = middle;
                        middle = test;
                    }
                }
                else
                {
                    if (test.Value > middle.Value)
                    {
                        upper = test;
                    }
                    else
                    {
                        lower = middle;
                        middle = test;
                    }
                }

                iterations += 1;
            }

            if (iterations == this.MaximumIterations)
                throw new MaximumIterationsException("Max iterations reached.");
            else
                return new MinimizationOutput1D(middle, iterations, ExitCondition.BoundTolerance);
            
        }

        private void ValueChecker(IEvaluation1D eval)
        {
            if (Double.IsNaN(eval.Value) || Double.IsInfinity(eval.Value))
                throw new EvaluationException("Objective function returned non-finite value.", eval);
        }
        private static double _golden_ratio = (1.0 + Math.Sqrt(5)) / 2.0;
    }
}