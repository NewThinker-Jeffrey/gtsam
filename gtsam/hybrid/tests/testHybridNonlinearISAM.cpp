/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridNonlinearISAM.cpp
 * @brief   Unit tests for nonlinear incremental inference
 * @author  Varun Agrawal, Fan Jiang, Frank Dellaert
 * @date    Jan 2021
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <numeric>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::W;
using symbol_shorthand::X;
using symbol_shorthand::Y;
using symbol_shorthand::Z;

/* ****************************************************************************/
// Test if we can perform elimination incrementally.
TEST(HybridNonlinearISAM, IncrementalElimination) {
  Switching switching(3);
  HybridNonlinearISAM isam;
  HybridNonlinearFactorGraph graph1;
  Values initial;

  // Create initial factor graph
  //  *        *      *
  //  |        |      |
  //  X0  -*-  X1 -*- X2
  //   \*-M0-*/
  graph1.push_back(switching.unaryFactors.at(0));   // P(X0)
  graph1.push_back(switching.binaryFactors.at(0));  // P(X0, X1 | M0)
  graph1.push_back(switching.binaryFactors.at(1));  // P(X1, X2 | M1)
  graph1.push_back(switching.modeChain.at(0));      // P(M0)

  initial.insert<double>(X(0), 1);
  initial.insert<double>(X(1), 2);
  initial.insert<double>(X(2), 3);

  // Run update step
  isam.update(graph1, initial);

  // Check that after update we have 3 hybrid Bayes net nodes:
  // P(X0 | X1, M0) and P(X1, X2 | M0, M1), P(M0, M1)
  HybridGaussianISAM bayesTree = isam.bayesTree();
  EXPECT_LONGS_EQUAL(3, bayesTree.size());
  EXPECT(bayesTree[X(0)]->conditional()->frontals() == KeyVector{X(0)});
  EXPECT(bayesTree[X(0)]->conditional()->parents() == KeyVector({X(1), M(0)}));
  EXPECT(bayesTree[X(1)]->conditional()->frontals() == KeyVector({X(1), X(2)}));
  EXPECT(bayesTree[X(1)]->conditional()->parents() == KeyVector({M(0), M(1)}));

  /********************************************************/
  // New factor graph for incremental update.
  HybridNonlinearFactorGraph graph2;
  initial = Values();

  graph1.push_back(switching.unaryFactors.at(1));  // P(X1)
  graph2.push_back(switching.unaryFactors.at(2));  // P(X2)
  graph2.push_back(switching.modeChain.at(1));     // P(M0, M1)

  isam.update(graph2, initial);

  bayesTree = isam.bayesTree();
  // Check that after the second update we have
  // 1 additional hybrid Bayes net node:
  // P(X1, X2 | M0, M1)
  EXPECT_LONGS_EQUAL(3, bayesTree.size());
  EXPECT(bayesTree[X(2)]->conditional()->frontals() == KeyVector({X(1), X(2)}));
  EXPECT(bayesTree[X(2)]->conditional()->parents() == KeyVector({M(0), M(1)}));
}

/* ****************************************************************************/
// Test if we can incrementally do the inference
TEST(HybridNonlinearISAM, IncrementalInference) {
  Switching switching(3);
  HybridNonlinearISAM isam;
  HybridNonlinearFactorGraph graph1;
  Values initial;

  // Create initial factor graph
  //    *        *        *
  //    |        |        |
  //    X0  -*-  X1  -*-  X2
  //         |        |
  //      *-M0 - * - M1
  graph1.push_back(switching.unaryFactors.at(0));   // P(X0)
  graph1.push_back(switching.binaryFactors.at(0));  // P(X0, X1 | M0)
  graph1.push_back(switching.unaryFactors.at(1));   // P(X1)
  graph1.push_back(switching.modeChain.at(0));      // P(M0)

  initial.insert<double>(X(0), 1);
  initial.insert<double>(X(1), 2);

  // Run update step
  isam.update(graph1, initial);
  HybridGaussianISAM bayesTree = isam.bayesTree();

  auto discreteConditional_m0 = bayesTree[M(0)]->conditional()->asDiscrete();
  EXPECT(discreteConditional_m0->keys() == KeyVector({M(0)}));

  /********************************************************/
  // New factor graph for incremental update.
  HybridNonlinearFactorGraph graph2;
  initial = Values();

  initial.insert<double>(X(2), 3);

  graph2.push_back(switching.binaryFactors.at(1));  // P(X1, X2 | M1)
  graph2.push_back(switching.unaryFactors.at(2));   // P(X2)
  graph2.push_back(switching.modeChain.at(1));      // P(M0, M1)

  isam.update(graph2, initial);
  bayesTree = isam.bayesTree();

  /********************************************************/
  // Run batch elimination so we can compare results.
  const Ordering ordering{X(0), X(1), X(2)};

  // Now we calculate the actual factors using full elimination
  const auto [expectedHybridBayesTree, expectedRemainingGraph] =
      switching.linearizedFactorGraph
          .BaseEliminateable::eliminatePartialMultifrontal(ordering);

  // The densities on X(1) should be the same
  auto x0_conditional = dynamic_pointer_cast<HybridGaussianConditional>(
      bayesTree[X(0)]->conditional()->inner());
  auto expected_x0_conditional =
      dynamic_pointer_cast<HybridGaussianConditional>(
          (*expectedHybridBayesTree)[X(0)]->conditional()->inner());
  EXPECT(assert_equal(*x0_conditional, *expected_x0_conditional));

  // The densities on X(1) should be the same
  auto x1_conditional = dynamic_pointer_cast<HybridGaussianConditional>(
      bayesTree[X(1)]->conditional()->inner());
  auto expected_x1_conditional =
      dynamic_pointer_cast<HybridGaussianConditional>(
          (*expectedHybridBayesTree)[X(1)]->conditional()->inner());
  EXPECT(assert_equal(*x1_conditional, *expected_x1_conditional));

  // The densities on X(2) should be the same
  auto x2_conditional = dynamic_pointer_cast<HybridGaussianConditional>(
      bayesTree[X(2)]->conditional()->inner());
  auto expected_x2_conditional =
      dynamic_pointer_cast<HybridGaussianConditional>(
          (*expectedHybridBayesTree)[X(2)]->conditional()->inner());
  EXPECT(assert_equal(*x2_conditional, *expected_x2_conditional));

  // We only perform manual continuous elimination for 0,0.
  // The other discrete probabilities on M(2) are calculated the same way
  const Ordering discreteOrdering{M(0), M(1)};
  HybridBayesTree::shared_ptr discreteBayesTree =
      expectedRemainingGraph->eliminateMultifrontal(discreteOrdering);

  // Test the probability values with regression tests.
  auto discrete = bayesTree[M(1)]->conditional()->asDiscrete();
  EXPECT(assert_equal(0.095292, (*discrete)({{M(0), 0}, {M(1), 0}}), 1e-5));
  EXPECT(assert_equal(0.282758, (*discrete)({{M(0), 1}, {M(1), 0}}), 1e-5));
  EXPECT(assert_equal(0.314175, (*discrete)({{M(0), 0}, {M(1), 1}}), 1e-5));
  EXPECT(assert_equal(0.307775, (*discrete)({{M(0), 1}, {M(1), 1}}), 1e-5));

  // Check that the clique conditional generated from incremental elimination
  // matches that of batch elimination.
  auto expectedConditional = (*discreteBayesTree)[M(1)]->conditional();
  auto actualConditional = bayesTree[M(1)]->conditional();
  EXPECT(assert_equal(*expectedConditional, *actualConditional, 1e-6));
}

/* ****************************************************************************/
// Test if we can approximately do the inference
TEST(HybridNonlinearISAM, Approx_inference) {
  Switching switching(4);
  HybridNonlinearISAM incrementalHybrid;
  HybridNonlinearFactorGraph graph1;
  Values initial;

  // Add the 3 hybrid factors, x0-x1, x1-x2, x2-x3
  for (size_t i = 0; i < 3; i++) {
    graph1.push_back(switching.binaryFactors.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(1), X(2), X(3)
  for (size_t i = 0; i < 4; i++) {
    graph1.push_back(switching.unaryFactors.at(i));
    initial.insert<double>(X(i), i + 1);
  }

  // TODO(Frank): no mode chain?

  // Create ordering.
  Ordering ordering;
  for (size_t j = 0; j < 4; j++) {
    ordering.push_back(X(j));
  }

  // Now we calculate the actual factors using full elimination
  const auto [unPrunedHybridBayesTree, unPrunedRemainingGraph] =
      switching.linearizedFactorGraph
          .BaseEliminateable::eliminatePartialMultifrontal(ordering);

  size_t maxNrLeaves = 5;
  incrementalHybrid.update(graph1, initial);
  HybridGaussianISAM bayesTree = incrementalHybrid.bayesTree();

  bayesTree.prune(maxNrLeaves);

  /*
  unPruned factor is:
    Choice(m3)
    0 Choice(m2)
    0 0 Choice(m1)
    0 0 0 Leaf 0.11267528
    0 0 1 Leaf 0.18576102
    0 1 Choice(m1)
    0 1 0 Leaf 0.18754662
    0 1 1 Leaf 0.30623871
    1 Choice(m2)
    1 0 Choice(m1)
    1 0 0 Leaf 0.18576102
    1 0 1 Leaf 0.30622428
    1 1 Choice(m1)
    1 1 0 Leaf 0.30623871
    1 1 1 Leaf  0.5

  pruned factors is:
    Choice(m3)
    0 Choice(m2)
    0 0 Leaf    0
    0 1 Choice(m1)
    0 1 0 Leaf 0.18754662
    0 1 1 Leaf 0.30623871
    1 Choice(m2)
    1 0 Choice(m1)
    1 0 0 Leaf    0
    1 0 1 Leaf 0.30622428
    1 1 Choice(m1)
    1 1 0 Leaf 0.30623871
    1 1 1 Leaf  0.5
  */

  auto discreteConditional_m0 = *dynamic_pointer_cast<DiscreteConditional>(
      bayesTree[M(0)]->conditional()->inner());
  EXPECT(discreteConditional_m0.keys() == KeyVector({M(0), M(1), M(2)}));

  // Get the number of elements which are greater than 0.
  auto count = [](const double &value, int count) {
    return value > 0 ? count + 1 : count;
  };
  // Check that the number of leaves after pruning is 5.
  EXPECT_LONGS_EQUAL(5, discreteConditional_m0.fold(count, 0));

  // Check that the hybrid nodes of the bayes net match those of the pre-pruning
  // bayes net, at the same positions.
  auto &unPrunedLastDensity = *dynamic_pointer_cast<HybridGaussianConditional>(
      unPrunedHybridBayesTree->clique(X(3))->conditional()->inner());
  auto &lastDensity = *dynamic_pointer_cast<HybridGaussianConditional>(
      bayesTree[X(3)]->conditional()->inner());

  std::vector<std::pair<DiscreteValues, double>> assignments =
      discreteConditional_m0.enumerate();
  // Loop over all assignments and check the pruned components
  for (auto &&av : assignments) {
    const DiscreteValues &assignment = av.first;
    const double value = av.second;

    if (value == 0.0) {
      EXPECT(lastDensity(assignment) == nullptr);
    } else {
      CHECK(lastDensity(assignment));
      EXPECT(assert_equal(*unPrunedLastDensity(assignment),
                          *lastDensity(assignment)));
    }
  }
}

/* ****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(HybridNonlinearISAM, Incremental_approximate) {
  Switching switching(5);
  HybridNonlinearISAM incrementalHybrid;
  HybridNonlinearFactorGraph graph1;
  Values initial;

  /***** Run Round 1 *****/
  // Add the 3 hybrid factors, x0-x1, x1-x2, x2-x3
  for (size_t i = 0; i < 3; i++) {
    graph1.push_back(switching.binaryFactors.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(1), X(2), X(3)
  for (size_t i = 0; i < 4; i++) {
    graph1.push_back(switching.unaryFactors.at(i));
    initial.insert<double>(X(i), i + 1);
  }

  // TODO(Frank): no mode chain?


  // Run update with pruning
  size_t maxComponents = 5;
  incrementalHybrid.update(graph1, initial);
  incrementalHybrid.prune(maxComponents);
  HybridGaussianISAM bayesTree = incrementalHybrid.bayesTree();

  // Check if we have a bayes tree with 4 hybrid nodes,
  // each with 2, 4, 8, and 5 (pruned) leaves respetively.
  EXPECT_LONGS_EQUAL(4, bayesTree.size());
  EXPECT_LONGS_EQUAL(
      2, bayesTree[X(0)]->conditional()->asHybrid()->nrComponents());
  EXPECT_LONGS_EQUAL(
      3, bayesTree[X(1)]->conditional()->asHybrid()->nrComponents());
  EXPECT_LONGS_EQUAL(
      5, bayesTree[X(2)]->conditional()->asHybrid()->nrComponents());
  EXPECT_LONGS_EQUAL(
      5, bayesTree[X(3)]->conditional()->asHybrid()->nrComponents());

  /***** Run Round 2 *****/
  HybridGaussianFactorGraph graph2;
  graph2.push_back(switching.binaryFactors.at(3));  // x3-x4
  graph2.push_back(switching.unaryFactors.at(4));  // x4 measurement
  initial = Values();
  initial.insert<double>(X(4), 5);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph2, initial);
  incrementalHybrid.prune(maxComponents);
  bayesTree = incrementalHybrid.bayesTree();

  // Check if we have a bayes tree with pruned hybrid nodes,
  // with 5 (pruned) leaves.
  CHECK_EQUAL(5, bayesTree.size());
  EXPECT_LONGS_EQUAL(
      5, bayesTree[X(3)]->conditional()->asHybrid()->nrComponents());
  EXPECT_LONGS_EQUAL(
      5, bayesTree[X(4)]->conditional()->asHybrid()->nrComponents());
}

/* ************************************************************************/
// A GTSAM-only test for running inference on a single-legged robot.
// The leg links are represented by the chain X-Y-Z-W, where X is the base and
// W is the foot.
// We use BetweenFactor<Pose2> as constraints between each of the poses.
TEST(HybridNonlinearISAM, NonTrivial) {
  /*************** Run Round 1 ***************/
  HybridNonlinearFactorGraph fg;
  HybridNonlinearISAM inc;

  // Add a prior on pose x0 at the origin.
  // A prior factor consists of a mean  and
  // a noise model (covariance matrix)
  Pose2 prior(0.0, 0.0, 0.0);  // prior mean is at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector3(0.3, 0.3, 0.1));  // 30cm std on x,y, 0.1 rad on theta
  fg.emplace_shared<PriorFactor<Pose2>>(X(0), prior, priorNoise);

  // create a noise model for the landmark measurements
  auto poseNoise = noiseModel::Isotropic::Sigma(3, 0.1);

  // We model a robot's single leg as X - Y - Z - W
  // where X is the base link and W is the foot link.

  // Add connecting poses similar to PoseFactors in GTD
  fg.emplace_shared<BetweenFactor<Pose2>>(X(0), Y(0), Pose2(0, 1.0, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Y(0), Z(0), Pose2(0, 1.0, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Z(0), W(0), Pose2(0, 1.0, 0),
                                          poseNoise);

  // Create initial estimate
  Values initial;
  initial.insert(X(0), Pose2(0.0, 0.0, 0.0));
  initial.insert(Y(0), Pose2(0.0, 1.0, 0.0));
  initial.insert(Z(0), Pose2(0.0, 2.0, 0.0));
  initial.insert(W(0), Pose2(0.0, 3.0, 0.0));

  // Don't run update now since we don't have discrete variables involved.

  using PlanarMotionModel = BetweenFactor<Pose2>;

  /*************** Run Round 2 ***************/
  // Add odometry factor with discrete modes.
  Pose2 odometry(1.0, 0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(3, 1.0);
  auto still = std::make_shared<PlanarMotionModel>(W(0), W(1), Pose2(0, 0, 0),
                                                   noise_model),
       moving = std::make_shared<PlanarMotionModel>(W(0), W(1), odometry,
                                                    noise_model);
  std::vector<NoiseModelFactor::shared_ptr> components{moving, still};
  fg.emplace_shared<HybridNonlinearFactor>(DiscreteKey(M(1), 2), components);

  // Add equivalent of ImuFactor
  fg.emplace_shared<BetweenFactor<Pose2>>(X(0), X(1), Pose2(1.0, 0.0, 0),
                                          poseNoise);
  // PoseFactors-like at k=1
  fg.emplace_shared<BetweenFactor<Pose2>>(X(1), Y(1), Pose2(0, 1, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Y(1), Z(1), Pose2(0, 1, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Z(1), W(1), Pose2(-1, 1, 0),
                                          poseNoise);

  initial.insert(X(1), Pose2(1.0, 0.0, 0.0));
  initial.insert(Y(1), Pose2(1.0, 1.0, 0.0));
  initial.insert(Z(1), Pose2(1.0, 2.0, 0.0));
  // The leg link did not move so we set the expected pose accordingly.
  initial.insert(W(1), Pose2(0.0, 3.0, 0.0));

  // Update without pruning
  // The result is a HybridBayesNet with 1 discrete variable M(1).
  // P(X | measurements) = P(W0|Z0, W1, M1) P(Z0|Y0, W1, M1) P(Y0|X0, W1, M1)
  //                       P(X0 | X1, W1, M1) P(W1|Z1, X1, M1) P(Z1|Y1, X1, M1)
  //                       P(Y1 | X1, M1)P(X1 | M1)P(M1)
  // The MHS tree is a 1 level tree for time indices (1,) with 2 leaves.
  inc.update(fg, initial);

  fg = HybridNonlinearFactorGraph();
  initial = Values();

  /*************** Run Round 3 ***************/
  // Add odometry factor with discrete modes.
  still = std::make_shared<PlanarMotionModel>(W(1), W(2), Pose2(0, 0, 0),
                                              noise_model);
  moving =
      std::make_shared<PlanarMotionModel>(W(1), W(2), odometry, noise_model);
  components = {moving, still};
  fg.emplace_shared<HybridNonlinearFactor>(DiscreteKey(M(2), 2), components);

  // Add equivalent of ImuFactor
  fg.emplace_shared<BetweenFactor<Pose2>>(X(1), X(2), Pose2(1.0, 0.0, 0),
                                          poseNoise);
  // PoseFactors-like at k=1
  fg.emplace_shared<BetweenFactor<Pose2>>(X(2), Y(2), Pose2(0, 1, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Y(2), Z(2), Pose2(0, 1, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Z(2), W(2), Pose2(-2, 1, 0),
                                          poseNoise);

  initial.insert(X(2), Pose2(2.0, 0.0, 0.0));
  initial.insert(Y(2), Pose2(2.0, 1.0, 0.0));
  initial.insert(Z(2), Pose2(2.0, 2.0, 0.0));
  initial.insert(W(2), Pose2(0.0, 3.0, 0.0));

  // Now we prune!
  // P(X | measurements) = P(W0|Z0, W1, M1) P(Z0|Y0, W1, M1) P(Y0|X0, W1, M1)
  //                       P(X0 | X1, W1, M1) P(W1|W2, Z1, X1, M1, M2)
  //                       P(Z1| W2, Y1, X1, M1, M2) P(Y1 | W2, X1, M1, M2)
  //                       P(X1 | W2, X2, M1, M2) P(W2|Z2, X2, M1, M2)
  //                       P(Z2|Y2, X2, M1, M2) P(Y2 | X2, M1, M2)
  //                       P(X2 | M1, M2) P(M1, M2)
  // The MHS at this point should be a 2 level tree on (1, 2).
  // 1 has 2 choices, and 2 has 4 choices.
  inc.update(fg, initial);
  inc.prune(2);

  fg = HybridNonlinearFactorGraph();
  initial = Values();

  /*************** Run Round 4 ***************/
  // Add odometry factor with discrete modes.
  still = std::make_shared<PlanarMotionModel>(W(2), W(3), Pose2(0, 0, 0),
                                              noise_model);
  moving =
      std::make_shared<PlanarMotionModel>(W(2), W(3), odometry, noise_model);
  components = {moving, still};
  fg.emplace_shared<HybridNonlinearFactor>(DiscreteKey(M(3), 2), components);

  // Add equivalent of ImuFactor
  fg.emplace_shared<BetweenFactor<Pose2>>(X(2), X(3), Pose2(1.0, 0.0, 0),
                                          poseNoise);
  // PoseFactors-like at k=3
  fg.emplace_shared<BetweenFactor<Pose2>>(X(3), Y(3), Pose2(0, 1, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Y(3), Z(3), Pose2(0, 1, 0),
                                          poseNoise);
  fg.emplace_shared<BetweenFactor<Pose2>>(Z(3), W(3), Pose2(-3, 1, 0),
                                          poseNoise);

  initial.insert(X(3), Pose2(3.0, 0.0, 0.0));
  initial.insert(Y(3), Pose2(3.0, 1.0, 0.0));
  initial.insert(Z(3), Pose2(3.0, 2.0, 0.0));
  initial.insert(W(3), Pose2(0.0, 3.0, 0.0));

  // Keep pruning!
  inc.update(fg, initial);
  inc.prune(3);

  fg = HybridNonlinearFactorGraph();
  initial = Values();

  HybridGaussianISAM bayesTree = inc.bayesTree();

  // The final discrete graph should not be empty since we have eliminated
  // all continuous variables.
  auto discreteTree = bayesTree[M(3)]->conditional()->asDiscrete();
  EXPECT_LONGS_EQUAL(3, discreteTree->size());

  // Test if the optimal discrete mode assignment is (1, 1, 1).
  DiscreteFactorGraph discreteGraph;
  discreteGraph.push_back(discreteTree);
  DiscreteValues optimal_assignment = discreteGraph.optimize();

  DiscreteValues expected_assignment;
  expected_assignment[M(1)] = 1;
  expected_assignment[M(2)] = 1;
  expected_assignment[M(3)] = 1;

  EXPECT(assert_equal(expected_assignment, optimal_assignment));

  // Test if pruning worked correctly by checking that
  // we only have 3 leaves in the last node.
  auto lastConditional = bayesTree[X(3)]->conditional()->asHybrid();
  EXPECT_LONGS_EQUAL(3, lastConditional->nrComponents());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
