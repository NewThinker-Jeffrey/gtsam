/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeMarginalizationHelper.h
 * @brief   Helper functions for marginalizing variables from a Bayes Tree.
 *
 * @author  Jeffrey (Zhiwei Wang)
 * @date    Oct 28, 2024
 */

// \callgraph
#pragma once

#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/base/debug.h>
#include "gtsam_unstable/dllexport.h"

namespace gtsam {

/**
 * This class provides helper functions for marginalizing variables from a Bayes Tree.
 */
template <typename BayesTree>
class GTSAM_UNSTABLE_EXPORT BayesTreeMarginalizationHelper {

public:
  using Clique = typename BayesTree::Clique;
  using sharedClique = typename BayesTree::sharedClique;

  /** Get the additional keys that need to be re-eliminated when marginalizing
   * the variables in @p marginalizableKeys from the Bayes tree @p bayesTree.
   * 
   * @param[in] bayesTree The Bayes tree to be marginalized.
   * @param[in] marginalizableKeys  The keys to be marginalized.
   * 
   * 
   * When marginalizing a variable @f$ \theta @f$ in a Bayes tree, some nodes
   * may need to be re-eliminated. The variable to be marginalized should be
   * eliminated first.
   * 
   * 1. If @f$ \theta @f$ is already in a leaf node @f$ L @f$, and all other
   *    frontal variables within @f$ L @f$ are to be marginalized, then this
   *    node does not need to be re-eliminated; the entire node can be directly
   *    marginalized.
   * 
   * 2. If @f$ \theta @f$ is in a leaf node @f$ L @f$, but @f$ L @f$ contains
   *    other frontal variables that do not need to be marginalized:
   *
   *    a. If all other non-marginalized frontal variables are listed after
   *       @f$ \theta @f$ (each node contains a frontal list, with variables to
   *       be eliminated earlier in the list), then node @f$ L @f$ does not
   *       need to be re-eliminated.
   *
   *    b. Otherwise, if there are non-marginalized nodes listed before
   *       @f$ \theta @f$, then node @f$ L @f$ needs to be re-eliminated, and
   *       correspondingly, all nodes between @f$ L @f$ and the root need to be
   *       re-eliminated.
   * 
   * 3. If @f$ \theta @f$ is in an intermediate node @f$ M @f$ (non-leaf node),
   *    but none of @f$ M @f$'s child nodes depend on variable @f$ \theta @f$
   *    (they only depend on other variables within @f$ M @f$), then during the
   *    process of marginalizing @f$ \theta @f$, @f$ M @f$ can be treated as a
   *    leaf node, and @f$ M @f$ should be processed following the same
   *    approach as for leaf nodes.
   * 
   *    In this case, the original elimination of @f$ \theta @f$ does not
   *    depend on the elimination results of variables in the child nodes.
   * 
   * 4. If @f$ \theta @f$ is in an intermediate node @f$ M @f$ (non-leaf node),
   *    and there exist child nodes that depend on variable @f$ \theta @f$,
   *    then not only does node @f$ M @f$ need to be re-eliminated, but all
   *    child nodes dependent on @f$ \theta @f$, including descendant nodes
   *    recursively dependent on @f$ \theta @f$, also need to be re-eliminated.
   *
   *    The frontal variables in child nodes were originally eliminated before
   *    @f$ \theta @f$ and their elimination results are relied upon by
   *    @f$ \theta @f$'s elimination. When re-eliminating, they should be
   *    eliminated after @f$ \theta @f$.
   */
  static void gatherAdditionalKeysToReEliminate(
      const BayesTree& bayesTree,
      const KeyVector& marginalizableKeys,
      std::set<Key>& additionalKeys) {
    const bool debug = ISDEBUG("BayesTreeMarginalizationHelper");

    std::set<Key> marginalizableKeySet(marginalizableKeys.begin(), marginalizableKeys.end());
    std::set<sharedClique> checkedCliques;

    std::set<sharedClique> dependentCliques;
    for (const Key& key : marginalizableKeySet) {
      sharedClique clique = bayesTree[key];
      if (checkedCliques.count(clique)) {
        continue;
      }
      checkedCliques.insert(clique);

      bool is_leaf = clique->children.empty();
      bool need_reeliminate = false;
      bool has_non_marginalizable_ahead = false;
      for (Key i: clique->conditional()->frontals()) {
        if (marginalizableKeySet.count(i)) {
          if (has_non_marginalizable_ahead) {
            need_reeliminate = true;
            break;
          } else {
            // Check whether there're child nodes dependent on this key.
            for(const sharedClique& child: clique->children) {
              if (std::find(child->conditional()->beginParents(),
                  child->conditional()->endParents(), i)
                  != child->conditional()->endParents()) {
                need_reeliminate = true;
                break;
              }
            }
          }
        } else {
          has_non_marginalizable_ahead = true;
        }
      }

      if (!need_reeliminate) {
        continue;
      } else {
        // need to re-eliminate this clique and all its children that depend on
        // a marginalizable key
        for (Key i: clique->conditional()->frontals()) {
          additionalKeys.insert(i);
          for (const sharedClique& child: clique->children) {
            if (!dependentCliques.count(child) &&
                std::find(child->conditional()->beginParents(),
                child->conditional()->endParents(), i)
                != child->conditional()->endParents()) {
              dependentCliques.insert(child);
            }
          }
        }
      }
    }

    // Recursively add the dependent keys
    while (!dependentCliques.empty()) {
      auto begin = dependentCliques.begin();
      sharedClique clique = *begin;
      dependentCliques.erase(begin);

      for (Key key : clique->conditional()->frontals()) {
        additionalKeys.insert(key);
      }

      for (const sharedClique& child: clique->children) {
        dependentCliques.insert(child);
      }
    }

    if (debug) {
      std::cout << "BayesTreeMarginalizationHelper: Additional keys to re-eliminate: ";
      for (const Key& key : additionalKeys) {
        std::cout << DefaultKeyFormatter(key) << " ";
      }
      std::cout << std::endl;
    }
  }
};
// BayesTreeMarginalizationHelper

}/// namespace gtsam
