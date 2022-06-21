// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_EDGE_SWITCHING_PROX_H_
#define G2O_EDGE_SWITCHING_PROX_H_

//#define NUMERIC_JACOBIAN_TWO_D_TYPES

#include "g2o/core/base_ternary_edge.h"
#include "vertex_prox.h"
#include "vertex_switch_pp.h"
#include "parameter_spp_weight.h"

//#include "vertex_se2.h"
#include "g2o_types_slam2d_api.h"

#ifndef prox_maxN
#define prox_maxN 10
#endif

namespace g2o {
    class G2O_TYPES_SLAM2D_API EdgeSwitchProx : public BaseTernaryEdge < 3 * prox_maxN, std::array<int, 2 * prox_maxN>, VertexProx, VertexProx, VertexSwitchingProxPair > {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          EdgeSwitchProx();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      virtual void linearizeOplus();
#endif

      virtual void setMeasurement(const std::array<int, 2 * prox_maxN> m) {
          for (int i = 0; i < 2 * prox_maxN; i++)
              _measurement[i] = m[i];
      }

      virtual void setMeasurement(const int m[2 * prox_maxN]) {
          for (int i = 0; i < 2 * prox_maxN; i++)
              _measurement[i] = m[i];
      }

      virtual bool setMeasurementData(const int* d) {
          for(int i = 0; i < 2 * prox_maxN; i++)
              _measurement[i] = d[i];
          return true;
      }

      virtual bool setMeasurementData(const int d) {
          for (int i = 0; i < 2 * prox_maxN; i++)
              _measurement[i] = d;
          return true;
      }

      virtual bool getMeasurementData(int d[2 * prox_maxN]) const {
          for (int i = 0; i < 2 * prox_maxN; i++)
              d[i] = _measurement[i];
          return true;
      }

      virtual int measurementDimension() const { return 3; }

      virtual bool setMeasurementFromState();

      virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
          (void)to;
          return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      virtual inline Vector2 from_prox(int i) const;
      virtual inline Vector2 to_prox(int i) const;
      virtual inline Vector2 from_gr_prox(int i) const;
      virtual inline Vector2 to_gr_prox(int i) const;
      virtual inline int num_prox_pairs() const { return _num_prox_pairs; };

  private:
      int _prox_pairs[2][prox_maxN];
      int _num_prox_pairs;
      ParameterSPPWeight* _ProxPairWeightInfo;
      number_t info;
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeSwitchProxDrawAction : public DrawAction {
  public:
      EdgeSwitchProxDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
          HyperGraphElementAction::Parameters* params_);
  protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleX, * _triangleY, * _triangleZ;
  };
#endif

}
#endif
