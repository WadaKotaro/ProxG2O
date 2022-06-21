// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_EDGE_SE2_PROXIMITY_H_
#define G2O_EDGE_SE2_PROXIMITY_H_

//#define NUMERIC_JACOBIAN_TWO_D_TYPES

#include "g2o/core/base_binary_edge.h"
#include "vertex_prox.h"

//#include "vertex_se2.h"
#include "g2o_types_slam2d_api.h"

#ifndef prox_maxN
#define prox_maxN 10
#endif

namespace g2o {


  /*! \class EdgeSE2PointXYOffset
   * \brief g2o edge from a track to a point node
   */
  // first two args are the measurement type, second two the connection classes
  /*
  class G2O_TYPES_SLAM2D_API EdgeSE2Proximity : public BaseBinaryEdge<2 * prox_maxN, SE2, VertexSE2Proximity, VertexSE2Proximity> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2Proximity();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError();
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
    virtual void linearizeOplus();
#endif

    virtual void setMeasurement(const SE2& m){
      _measurement = m;
    }

    virtual bool setMeasurementData(const number_t* d){
        _measurement = SE2(d[0], d[1], d[2]);
        return true;
    }

    virtual bool getMeasurementData(number_t* d) const{
        Vector3 v = _measurement.toVector();
        d[0] = v[0];
        d[1] = v[1];
        d[2] = v[2];
        return true;
    }

    virtual int measurementDimension() const {return 3;}

    virtual bool setMeasurementFromState() ;

    virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
      (void)to;
      return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);
    }

    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

#ifndef Param
    virtual inline Vector2 from_prox(int i) const;
    virtual inline Vector2 to_prox(int i) const;
    virtual inline Vector2 from_gr_prox(int i) const;
    virtual inline Vector2 to_gr_prox(int i) const;
    virtual inline int num_prox_pairs() const { return _num_prox_pairs; };
#else
    const virtual inline CachePointXYProximity* f() const { return dynamic_cast<VertexSE2Proximity*>(_vertices[0])->proximityCache(); };
    const virtual inline CachePointXYProximity* t() const { return dynamic_cast<VertexSE2Proximity*>(_vertices[1])->proximityCache(); };
#endif

  private:
      int _prox_pairs[2][prox_maxN];
      int _num_prox_pairs;

  };
  */
    class G2O_TYPES_SLAM2D_API EdgeProx : public BaseBinaryEdge < 2 * prox_maxN, std::array<int, 2 * prox_maxN>, VertexProx, VertexProx > {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          EdgeProx();
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

#ifndef Param
      virtual inline Vector2 from_prox(int i) const;
      virtual inline Vector2 to_prox(int i) const;
      virtual inline Vector2 from_gr_prox(int i) const;
      virtual inline Vector2 to_gr_prox(int i) const;
      virtual inline int num_prox_pairs() const { return _num_prox_pairs; };
#else
      const virtual inline CachePointXYProximity* f() const { return dynamic_cast<VertexSE2Proximity*>(_vertices[0])->proximityCache(); };
      const virtual inline CachePointXYProximity* t() const { return dynamic_cast<VertexSE2Proximity*>(_vertices[1])->proximityCache(); };
#endif

  private:
      int _prox_pairs[2][prox_maxN];
      int _num_prox_pairs;

  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeProxDrawAction : public DrawAction {
  public:
      EdgeProxDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
          HyperGraphElementAction::Parameters* params_);
  protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleX, * _triangleY, * _triangleZ;
  };
#endif

}
#endif
