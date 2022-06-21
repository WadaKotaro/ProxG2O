// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_VERTEX_SE2_PROXIMITY_H
#define G2O_VERTEX_SE2_PROXIMITY_H

#include "g2o/core/base_binary_edge.h"

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_types_slam2d_api.h"

#ifndef prox_maxN
#define prox_maxN 10
#endif

//#define Param

namespace g2o {

  /**
   * \brief 2D pose Vertex, (x,y,theta)
   */
  class ParameterPointXYProximity;
  class CachePointXYProximity;

  class G2O_TYPES_SLAM2D_API VertexProx : public BaseVertex<3, SE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          VertexProx();

      virtual void setToOriginImpl() {
        _estimate = SE2();
      }

      virtual void oplusImpl(const number_t* update);

      virtual bool setEstimateDataImpl(const number_t* est){
        _estimate=SE2(est[0], est[1], est[2]);
        return true;
      }

      virtual bool getEstimateData(number_t* est) const {
        Eigen::Map<Vector3> v(est);
        v = _estimate.toVector();
        return true;
      }
      
      virtual int estimateDimension() const { return 3; }

      virtual bool setMinimalEstimateDataImpl(const number_t* est){
        return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(number_t* est) const {
        return getEstimateData(est);
      }
      
      virtual int minimalEstimateDimension() const { return 3; }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;


      virtual inline Vector2 proximity(int i) const {
          if(0 <= i && i < _num_prox)
            return *_proximityParam[i];
          std::cerr << "out of range" << std::endl;
          return Vector2();
      };
      virtual inline int num_prox() const { return _num_prox; };

  private:
      Vector2* _proximityParam[prox_maxN];
      int _num_prox;
  };

  class G2O_TYPES_SLAM2D_API VertexProxWriteGnuplotAction: public WriteGnuplotAction {
  public:
      VertexProxWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API VertexProxDrawAction: public DrawAction{
  public:
    VertexProxDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  protected:
    HyperGraphElementAction* _drawActions;
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _arrow_length, * _arrow_width, * _point_size, * _line_width;

  };
#endif

} // end namespace

#endif
