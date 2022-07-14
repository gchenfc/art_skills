import gtsam
import numpy as np
from gtsam.symbol_shorthand import X, P
from fit_types import Stroke, Strokes, FitParams
from sln_stroke_fit import SlnStrokeFit

def create_init_values(fit_params: FitParams, strokes: Strokes, fitter: SlnStrokeFit):
    # Initial values
    initial_values = gtsam.Values()
    if 'default' == fit_params.initialization_strategy_params:
        for i in range(len(strokes)):
            initial_values.insert(P(i), np.array([-0.3, 1., 0., 0., 0.5, -0.5]))
            # initial_values.insert(P(i), np.array([-.3, 1., 1.57, -4.7, 0.5, -0.5]))
            # initial_values.insert(P(i),
            #                       np.array([0.0, 1., (i) * 0.75, (i + 1) * 0.75, 0.5, -0.5]))
    elif isinstance(fit_params.initialization_strategy_params, gtsam.Values):
        initial_values.insert(fit_params.initialization_strategy_params)
    elif ' :D ' in fit_params.initialization_strategy_params:
        for i, stroke in enumerate(strokes):
            # TODO(gerry): clean this up
            v = np.diff(stroke[:, 1:], axis=0) / np.diff(stroke[:, 0]).reshape(-1, 1)
            angles = np.arctan2(v[:, 1], v[:, 0])
            th1 = np.mean(angles[:10])
            if 'old' in fit_params.initialization_strategy_params:
                th2 = np.mean(angles[-10:])
            else:
                angular_displacements = np.diff(angles)
                angular_displacements = np.arctan2(np.sin(angular_displacements),
                                                np.cos(angular_displacements))
                th2 = th1 + np.sum(angular_displacements[9:])
            sigma = 0.4
            speed = np.sqrt(np.sum(np.square(v), axis=1))
            duration = stroke[-1, 0] - stroke[0, 0]
            """
            in order to get our curve to capture ~95% of the velocity profile, we want:
                argument_of_exp = -(1/2) * (ln(t-t0) - mu)^2 / sigma^2
                2*sigma == ln(t-t0) - mu
            assume t0 = 0, then
                2*sigma == ln(duration) - mu
                mu = ln(duration) - 2*sigma
                duration = exp(2*sigma + mu)
            """
            mu = np.log(duration) - 0.9  # for sigma = 0.4

            peak_speed_i = np.argmax(speed)
            tpeak = stroke[peak_speed_i, 0]
            t0 = stroke[0, 0]
            t0alt = tpeak - np.exp(mu - sigma * sigma)

            tpeak_alt = np.exp(mu - sigma * sigma)
            predicted_peak_speed = 1 / (sigma * np.sqrt(2 * np.pi) *
                                        (tpeak_alt)) * np.exp(-0.5 * sigma * sigma)
            D = speed[peak_speed_i] / predicted_peak_speed

            # print('initial values are: ', np.array([t0, D, th1, th2, sigma, mu]))

            initial_values.insert(P(i), np.array([t0, D, th1, th2, sigma, mu]))
    else:
        raise NotImplementedError('The parameter initialization strategy is not yet implemented')

    if 'from params' in fit_params.initialization_strategy_points:
        stroke_indices = fitter.stroke_indices(strokes)
        # "enhanced" this doesn't seem to work as well for some reason
        if 'enhanced' in fit_params.initialization_strategy_points:
            for (begin, _), stroke in zip(stroke_indices.values(), strokes):
                initial_values.insert(X(begin), stroke[0, 1:])
        initial_values = fitter.create_initial_values_from_params(strokes[0][0, 1:], initial_values,
                                                                  stroke_indices)
    elif isinstance(fit_params.initialization_strategy_points, gtsam.Values):
        initial_values.insert_or_assign(fit_params.initialization_strategy_points)
    elif 'from data' in fit_params.initialization_strategy_points:
        raise NotImplementedError('This initialization strategy hasnt been implemented yet.')
    elif fit_params.initialization_strategy_points == 'zero':
        tmax = max(stroke[-1, 0] for stroke in strokes)
        for k in range(fitter.t2k(tmax) + 1):
            initial_values.insert(X(k), np.zeros(2))

    return initial_values


def create_init_values_2(fit_params: FitParams, stroke: Stroke, index: int):
    # Initial values
    initial_values = gtsam.Values()
    if 'default' == fit_params.initialization_strategy_params:
        initial_values.insert(P(index), np.array([-0.3, 1., 0., 0., 0.5, -0.5]))
        # initial_values.insert(P(i), np.array([-.3, 1., 1.57, -4.7, 0.5, -0.5]))
        # initial_values.insert(P(i),
        #                       np.array([0.0, 1., (i) * 0.75, (i + 1) * 0.75, 0.5, -0.5]))
    elif isinstance(fit_params.initialization_strategy_params, gtsam.Values):
        initial_values.insert(fit_params.initialization_strategy_params)
    elif ' :D ' in fit_params.initialization_strategy_params:
        # TODO(gerry): clean this up
        v = np.diff(stroke[:, 1:], axis=0) / np.diff(stroke[:, 0]).reshape(-1, 1)
        angles = np.arctan2(v[:, 1], v[:, 0])
        th1 = np.mean(angles[:10])
        if 'old' in fit_params.initialization_strategy_params:
            th2 = np.mean(angles[-10:])
        else:
            angular_displacements = np.diff(angles)
            angular_displacements = np.arctan2(np.sin(angular_displacements),
                                            np.cos(angular_displacements))
            th2 = th1 + np.sum(angular_displacements[9:])
        sigma = 0.4
        speed = np.sqrt(np.sum(np.square(v), axis=1))
        duration = stroke[-1, 0] - stroke[0, 0]
        """
        in order to get our curve to capture ~95% of the velocity profile, we want:
            argument_of_exp = -(1/2) * (ln(t-t0) - mu)^2 / sigma^2
            2*sigma == ln(t-t0) - mu
        assume t0 = 0, then
            2*sigma == ln(duration) - mu
            mu = ln(duration) - 2*sigma
            duration = exp(2*sigma + mu)
        """
        mu = np.log(duration) - 0.9  # for sigma = 0.4

        peak_speed_i = np.argmax(speed)
        tpeak = stroke[peak_speed_i, 0]
        t0 = stroke[0, 0] - 0.1
        t0alt = tpeak - np.exp(mu - sigma * sigma)

        tpeak_alt = np.exp(mu - sigma * sigma)
        predicted_peak_speed = 1 / (sigma * np.sqrt(2 * np.pi) *
                                    (tpeak_alt)) * np.exp(-0.5 * sigma * sigma)
        D = speed[peak_speed_i] / predicted_peak_speed

        # print('initial values are: ', np.array([t0, D, th1, th2, sigma, mu]))

        initial_values.insert(P(index), np.array([t0, D, th1, th2, sigma, mu]))
    else:
        raise NotImplementedError('The parameter initialization strategy is not yet implemented')

    initial_values.insert(X(index), stroke[0, 1:])

    return initial_values


def create_init_values_3(stroke: Stroke, index: int) -> gtsam.Values:
    values = gtsam.Values()
    # X
    values.insert(X(index), stroke[0, 1:])
    # P
    if True:
        # start time
        t0 = stroke[0, 0] - 0.05
        # thetas
        if True:
            th_mean = stroke[-1, 1:] - stroke[0, 1:]
            th_mean = np.arctan2(th_mean[1], th_mean[0])
            dth = np.diff(stroke[:, 1:], axis=0)
            dth = np.diff(np.arctan2(dth[:, 1], dth[:, 0]))
            dth = np.clip(np.arctan2(np.sin(dth), np.cos(dth)), -0.2, 0.2)
            th_dev = sum(dth)
            th0 = th_mean - th_dev / 2
            th1 = th_mean + th_dev / 2
        else:
            v = np.diff(stroke[:, 1:], axis=0) / np.diff(stroke[:, 0]).reshape(-1, 1)
            angles = np.arctan2(v[:, 1], v[:, 0])
            th1 = np.mean(angles[:10])
            if False:
                th2 = np.mean(angles[-10:])
            else:
                angular_displacements = np.diff(angles)
                angular_displacements = np.arctan2(np.sin(angular_displacements),
                                                np.cos(angular_displacements))
                th2 = th1 + np.sum(angular_displacements[9:])
            th0, th1 = th1, th2
        print(th0, th1)
        # mu, sigma
        sigma = 0.4
        duration = stroke[-1, 0] - stroke[0, 0]
        mu = np.log(duration) - 0.9  # for sigma = 0.4
        # D
        tpeak_alt = np.exp(mu - sigma * sigma)
        predicted_peak_speed = 1 / (sigma * np.sqrt(2 * np.pi) *
                                    (tpeak_alt)) * np.exp(-0.5 * sigma * sigma)
        speed = np.linalg.norm(np.diff(stroke[:, 1:], axis=0) /
                               np.diff(stroke[:, 0]).reshape(-1, 1),
                               axis=1)
        D = max(speed) / predicted_peak_speed * 2.0  # 2.0 fudge factor

    values.insert(P(index), np.array([t0, D, th0, th1, sigma, mu]))
    return values
