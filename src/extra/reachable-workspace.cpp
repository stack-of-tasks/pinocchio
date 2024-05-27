//
// Copyright (c) 2016-2023 CNRS INRIA
//

#include "pinocchio/extra/reachable-workspace.hpp"

#include <libqhullcpp/QhullError.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/Qhull.h>

namespace pinocchio
{
  namespace internal
  {
    void buildConvexHull(ReachableSetResults & res)
    {
      using namespace orgQhull;

      Qhull qh;
      qh.runQhull(
        "", static_cast<int>(res.vertex.rows()), static_cast<int>(res.vertex.cols()),
        res.vertex.data(), "QJ");
      if (qh.qhullStatus() != qh_ERRnone)
        throw(qh.qhullMessage());

      QhullFacetList facets = qh.facetList();
      res.faces.resize(static_cast<int>(facets.count()), 3);
      QhullFacetListIterator j(facets);
      QhullFacet f;
      int count = 0;

      while (j.hasNext())
      {
        f = j.next();
        if (!f.isGood())
        {
          // ignore facet
        }
        else
        {
          QhullVertexSet vs = f.vertices();
          for (int i = 0; i < static_cast<int>(vs.size()); i++)
            res.faces(count, i) = static_cast<int>(vs[i].point().id());
          count++;
        }
      }
    }
  } // namespace internal
} // namespace pinocchio
