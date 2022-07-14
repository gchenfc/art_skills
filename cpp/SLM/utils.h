/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  utils.h
 *  @brief A collection of utilities, mainly for python wrapper
 *  @author Gerry Chen
 *  @author Juan-Diego Florez
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/expressions.h>

namespace art_skills {

template <typename T>
using DebugExpressionCallback =
    std::function<void(const gtsam::Values&, const T& return_value)>;

template <typename T>
void default_debug_expression_cb(const gtsam::Values& values, const T& ret) {
  gtsam::traits<T>::Print(ret, "Debug: evaluated to:");
  values.print("       called with:");
};

/// Wrapper for ExpressionNode that calls a callback for debugging purposes.
template <typename T>
class DebugExpressionNode : public gtsam::internal::ExpressionNode<T> {
 public:
  DebugExpressionNode(const gtsam::Expression<T>& e,
                      const DebugExpressionCallback<T>& debug_expression_cb)
      : expression_(e.root()), debug_expression_cb_(debug_expression_cb) {}

  void print(const std::string& prefix = "") const override {
    expression_->print(prefix + "Debug");
  }
  /// Calls callback (forwards computation to expression_)
  T value(const gtsam::Values& values) const override {
    T ret = expression_->value(values);
    debug_expression_cb_(values, ret);
    return ret;
  }
  T traceExecution(
      const gtsam::Values& values, gtsam::internal::ExecutionTrace<T>& trace,
      gtsam::internal::ExecutionTraceStorage* traceStorage) const override {
    return expression_->traceExecution(values, trace, traceStorage);
  }

 protected:
  boost::shared_ptr<gtsam::internal::ExpressionNode<T>> expression_;
  DebugExpressionCallback<T> debug_expression_cb_;
};

/// Wrapper for ExpressionNode that calls a callback for debugging purposes.
/// Note: can't use static function because this constructor is protected.
template <typename T>
class DebugExpression : public gtsam::Expression<T> {
 public:
  DebugExpression(const gtsam::Expression<T>& expression,
                  const DebugExpressionCallback<T>& debug_expression_cb =
                      default_debug_expression_cb<T>)
      : Expression<T>(boost::make_shared<DebugExpressionNode<T>>(
            expression, debug_expression_cb)) {}
};

template <typename T>
class DebugExpressionFactor : public gtsam::ExpressionFactor<T> {
 public:
  DebugExpressionFactor(gtsam::SharedNoiseModel noiseModel,  //
                        T measurement, gtsam::Expression<T> expression,
                        DebugExpressionCallback<T> debug_expression_cb =
                            default_debug_expression_cb<T>)
      : gtsam::ExpressionFactor<T>(noiseModel, measurement, expression),
        debug_expression_cb_(debug_expression_cb){};

  gtsam::Vector unwhitenedError(const gtsam::Values& x,
                                boost::optional<std::vector<gtsam::Matrix>&> H =
                                    boost::none) const override {
    gtsam::Vector ret = gtsam::ExpressionFactor<T>::unwhitenedError(x, H);
    debug_expression_cb_(x, ret);
    return ret;
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& x) const override {
    debug_expression_cb_(x, unwhitenedError(x));
    return gtsam::ExpressionFactor<T>::linearize(x);
  }

 private:
  DebugExpressionCallback<T> debug_expression_cb_;
};

}  // namespace art_skills
