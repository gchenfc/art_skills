"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the entire project, end-end
Author: Frank Dellaert, JD Florez-Castillo
"""

import gtsam


def ChebyshevFit(data, time, p_order):
    """
    Fits a Chebyshev Basis and interpolates for x against t using GTSAM

    Args:
        data ([array]): [the x or y coordinates of a stroke]
        time ([array]): [the time for each x or y position]
        porder ([int]): [the order of chebyshev polynomial]
    """
    noise = gtsam.noiseModel.Unit.Create(1)
    fit = gtsam.FitBasisChebyshev1Basis(data, noise, p_order)
    coeff = fit.parameters()
    basis = gtsam.Chebyshev1Basis
    interp = basis.WeightMatrix(p_order, time) @ coeff
    return(interp)