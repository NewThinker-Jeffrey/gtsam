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

  /** Get the additional keys that need reelimination when marginalizing
   * the variables in @p marginalizableKeys from the Bayes tree @p bayesTree.
   * 
   * @param[in] bayesTree The Bayes tree.
   * @param[in] marginalizableKeys  The keys to be marginalized.
   * 
   * 
   * When marginalizing a variable @f$ \theta @f$ from a Bayes tree, some
   * nodes may need reelimination to ensure the variables to marginalize
   * be eliminated first.
   * 
   * We should consider two cases:
   * 
   * 1. If a child node relies on @f$ \theta @f$ (i.e., @f$ \theta @f$
   *    is a parent / separator of the node), then the frontal
   *    variables of the child node need to be reeliminated. In
   *    addition, all the descendants of the child node also need to
   *    be reeliminated.
   * 
   * 2. If other frontal variables in the same node with @f$ \theta @f$
   *    are in front of @f$ \theta @f$ but not to be marginalized, then
   *    these variables also need to be reeliminated.
   * 
   * These variables were eliminated before @f$ \theta @f$ in the original
   * Bayes tree, and after reelimination they will be eliminated after
   * @f$ \theta @f$ so that @f$ \theta @f$ can be marginalized safely.
   * 
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

      bool need_reeliminate = false;
      bool has_non_marginalizable_ahead = false;
      for (Key i: clique->conditional()->frontals()) {
        if (marginalizableKeySet.count(i)) {
          if (has_non_marginalizable_ahead) {
            // Case 2 in the docstring
            need_reeliminate = true;
            break;
          } else {
            // Check whether there's a child node dependent on this key.
            for(const sharedClique& child: clique->children) {
              if (std::find(child->conditional()->beginParents(),
                  child->conditional()->endParents(), i)
                  != child->conditional()->endParents()) {
                // Case 1 in the docstring
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
        // No variable needs to be reeliminated
        continue;
      } else {
        // Need to reeliminate the current clique and all its children
        // that rely on a marginalizable key.
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
