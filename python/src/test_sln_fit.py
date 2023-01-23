"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test SLN regression
Author: Gerry Chen
"""

import unittest
import sln_fit

import numpy as np
import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P
import utils
import loader
from fit_types import OptimizationLoggingParams, StrokeSolution, TrajectorySolution, LetterSolution


class TestSlnFit(GtsamTestCase):
    """Test SlnFit."""

    def setUp(self) -> None:
        self.letter_data = loader.load_segments('D', index=None)
        self.trajectory_data = self.letter_data[0]
        self.stroke_data = self.trajectory_data[0]

        self.logging_params = OptimizationLoggingParams(False, False)

    def test_stroke_fit_ideal(self):
        """Test regression using 4 data points that should be exactly fit-able to a SLN curve"""
        #                     t0, D, th1, th2, sigma, mu
        pExpected = np.array([0.0, 1.0, 0.0, 0.0, 0.5, -1.0])
        data = np.array([
            [0.0, 0.0, 0.0],
            [0.2, 0.12104954452446419, 0.0],
            [0.3, 0.35388060275428224, 0.0],
            [0.6, 0.8401353767843259, 0.0],
        ])

        lm_params = utils.create_params(relativeErrorTol=0, absoluteErrorTol=0, errorTol=0)
        sol = sln_fit.fit_stroke(data,
                                 logging_params=self.logging_params,
                                 fit_params=sln_fit.FitStrokeParams(lm_params=lm_params))

        # test fit quality
        self.gtsamAssertEquals(sol['txy'], data, tol=1e-8)

    def test_stroke_fit(self):
        """Test regression of a single stroke using real mocap data"""
        sol = sln_fit.fit_stroke(self.stroke_data, logging_params=self.logging_params)
        for rowi in range(len(self.stroke_data)):  # separate by row to make it easier to read
            self.gtsamAssertEquals(sol['txy'][rowi], self.stroke_data[rowi], tol=3e-2)

    def test_trajectory_fit(self):
        """Test regression of a single trajectory using real mocap data"""
        sol = sln_fit.fit_trajectory(self.trajectory_data, logging_params=self.logging_params)
        for rowi in range(len(sol['data'])):  # separate by row to make it easier to read
            self.gtsamAssertEquals(sol['txy'][rowi], sol['data'][rowi], tol=3e-2)
        # print([stroke.shape for stroke in self.trajectory_data])
        self.assertDictEqual(sol['stroke_indices'], {0: (0, 30), 1: (30, 56), 2: (56, 82)})

    def test_letter_fit(self):
        """Test regression of a single trajectory using real mocap data"""
        sols = sln_fit.fit_letter(self.letter_data, logging_params=self.logging_params)
        for txy, data in zip(sols['txy'], sols['data']):
            for rowi in range(len(data)):  # separate by row to make it easier to read
                self.gtsamAssertEquals(txy[rowi], data[rowi], tol=5e-2)
        self.assertEqual(len(sols['all_stroke_indices']), 2)  # Letter D is 2 trajectories long
        self.assertDictEqual(sols['all_stroke_indices'][0], {0: (0, 30), 1: (30, 56), 2: (56, 82)})
        # print([stroke.shape for stroke in self.letter_data[1]])
        self.assertDictEqual(sols['all_stroke_indices'][1], {0: (0, 20), 1: (20, 36)})

    def test_histories(self):
        logging_params = OptimizationLoggingParams(False, True)
        # run optimizations
        stroke_history = []
        trajectory_history = []
        letter_history = []
        sln_fit.fit_stroke(self.stroke_data,
                           logging_params=logging_params,
                           history_out=stroke_history)
        sln_fit.fit_trajectory(self.trajectory_data,
                               logging_params=logging_params,
                               history_out=trajectory_history)
        sln_fit.fit_letter(self.letter_data,
                           logging_params=logging_params,
                           history_out=letter_history)
        # check that histories are non-empty
        self.assertGreater(len(stroke_history), 1)
        self.assertGreater(len(trajectory_history), 1)
        self.assertGreater(len(letter_history), 1)

        # check that histories are not all the same (i.e. copy-by-reference issue)
        def my_assert_dicts_not_equal(expected, actual):
            self.assertEqual(expected.keys(), actual.keys())
            for key in expected:
                self.assertIn(key, actual)
                if key == 'stroke_indices':
                    self.assertDictEqual(expected[key], actual[key])
                    continue
                for a, e in zip(actual[key], expected[key]):
                    if key == 'all_stroke_indices':
                        self.assertDictEqual(e, a)
                    elif key == 'data':
                        np.testing.assert_array_equal(e, a)
                    else:
                        self.assertTrue(np.any(np.not_equal(a, e)),
                                        f'dict match on {key =}:\n{a}\n{e}')

        my_assert_dicts_not_equal(stroke_history[0], stroke_history[-1])
        my_assert_dicts_not_equal(trajectory_history[0], trajectory_history[-1])
        my_assert_dicts_not_equal(letter_history[0], letter_history[-1])

        # check that the histories at least contain the right keys in the dictionary
        self.assertEqual(stroke_history[0].keys(), {'params', 'txy', 'data'})
        self.assertEqual(trajectory_history[0].keys(), {'params', 'txy', 'data', 'stroke_indices'})
        self.assertEqual(letter_history[0].keys(), {'params', 'txy', 'data', 'all_stroke_indices'})

    def test_params_conversion(self):
        letter_initial_guess = [gtsam.Values(), gtsam.Values()]
        for j, traj_initial_guess in enumerate(letter_initial_guess):
            for i in range(3):
                traj_initial_guess.insert(P(i), np.array([1, 2, 3, 4, 5, 6]) + j * 20 + i * 6)
                traj_initial_guess.insert(X(i), np.array([101, 102]) + j * 20 + i * 2)
        letter_params = sln_fit.FitLetterParams(initial_guess=letter_initial_guess,
                                                strokewise=True,
                                                pos_expression_eps=123)
        traj_params = letter_params.FitTrajectoryParams(0)
        self.assertTrue(traj_params.strokewise)
        self.assertEqual(traj_params.pos_expression_eps, 123)
        self.gtsamAssertEquals(traj_params.initial_guess, letter_initial_guess[0])

        stroke_params = traj_params.FitStrokeParams(1)
        self.assertRaises(AttributeError, lambda: stroke_params.strokewise)
        self.assertEqual(stroke_params.pos_expression_eps, 123)
        stroke_initial_guess_expected = utils.ValuesFromDict({
            P(1): np.array([1, 2, 3, 4, 5, 6]) + 6,
            X(1): np.array([101, 102]) + 2,
        })
        self.gtsamAssertEquals(stroke_params.initial_guess, stroke_initial_guess_expected)


if __name__ == "__main__":
    unittest.main()
