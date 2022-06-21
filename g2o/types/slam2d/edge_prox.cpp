// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#include "edge_prox.h"
#include <iostream>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeProx::EdgeProx()
      : BaseBinaryEdge<20, array<int, 2 * prox_maxN>, VertexProx, VertexProx>(), _num_prox_pairs(0), _prox_pairs() {
      memset(_prox_pairs[0], 0, prox_maxN);
      memset(_prox_pairs[1], 0, prox_maxN);
    information().setZero();
  }

  bool EdgeProx::read(std::istream& is) {
    // measured keypoint
    setMeasurementData(0);
    if (is.bad()) return false;
    //readInformationMatrix(is);
    //  we overwrite the information matrix in case of read errors
    //if (is.bad()) information().setIdentity();
    information().setZero();
    _num_prox_pairs = 0;
    int n;
    is >> n;
    for (int i = 0; i < n && i < prox_maxN; i++) {
        is >> _prox_pairs[0][i] >> _prox_pairs[1][i];
        double info;
        is >> info;
        information()(2*i, 2*i) = info;
        information()(2*i+1, 2*i+1) = info;
        if (is.bad()) return false;
        _num_prox_pairs = i + 1;
    }
    return true;
  }

  bool EdgeProx::write(std::ostream& os) const {
      int n = _num_prox_pairs;
      os << _num_prox_pairs << " ";
      for (int i = 0; i < _num_prox_pairs && i < prox_maxN; i++) {
          double info;
          os << _prox_pairs[0][i] << " " << _prox_pairs[1][i] << " " << information()(2*i, 2*i) << " ";
      }
      return true;
  }

  void EdgeProx::computeError() {
#ifndef Param
      const SE2 vi = dynamic_cast<VertexProx*>(_vertices[0])->estimate();
      const SE2 vj = dynamic_cast<VertexProx*>(_vertices[1])->estimate();
      for (int k = 0; k < _num_prox_pairs && k < prox_maxN; k++) {
          const Vector2 ri = from_prox(k),  rj = to_prox(k), qi = from_gr_prox(k), qj = to_gr_prox(k);
          if (ri == Vector2() || rj == Vector2() || qi == Vector2() || qj == Vector2()) continue;
          const Vector2 ei = vi.rotation() * ri, ej = vj.rotation() * rj;
          const number_t li = cst((ei / ri.norm()).dot(qj - qi)), lj = cst((ej / rj.norm()).dot(qi - qj));
          _error.block<2, 1>(2*k, 0) << li, lj;
      }
#else
      for (int i = 0; i < _num_prox_pairs && i < prox_maxN; i++) {
          const int fi = _prox_pairs[0][i], ti = _prox_pairs[1][i];
          const number_t fl = cst(f()->e(fi).dot(t()->q(ti) - f()->q(fi))), tl = cst(t()->e(ti).dot(f()->q(fi) - t()->q(ti)));
          _error.block<2, 1>(2 * i, 0) << fl, tl;
      }
#endif
  }

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeProx::linearizeOplus() {
      _jacobianOplusXi.setZero();
      _jacobianOplusXj.setZero();
      const SE2 vi = dynamic_cast<VertexProx*>(_vertices[0])->estimate();
      const SE2 vj = dynamic_cast<VertexProx*>(_vertices[1])->estimate();
      const Vector2 dt = vj.translation() - vi.translation();
      const Matrix2 Ri = vi.rotation().toRotationMatrix(),
          Rj = vj.rotation().toRotationMatrix(),
          dR = Ri.transpose() * Rj;
      for (int k = 0; k < _num_prox_pairs && k < prox_maxN; k++) {
          const Vector2 ri = from_prox(k), rj = to_prox(k);
          if (ri == Vector2() || rj == Vector2()) continue;
          const number_t ni = 1. / ri.norm(), nj = 1. / rj.norm();
          Vector2 z;

          _jacobianOplusXi.block<1, 2>(2*k, 0) = -ri;
          _jacobianOplusXj.block<1, 2>(2*k, 0) = dR.transpose() * ri;
          z = dR * rj + Ri.transpose() * dt;
          _jacobianOplusXi(2*k, 2) = ri.x() * z.y() - ri.y() * z.x();
          z = dR * rj;
          _jacobianOplusXj(2*k, 2) = -ri.x() * z.y() + ri.y() * z.x();
          _jacobianOplusXi.block<1, 3>(2*k, 0) = _jacobianOplusXi.block<1, 3>(2*k, 0) * ni;
          _jacobianOplusXj.block<1, 3>(2*k, 0) = _jacobianOplusXj.block<1, 3>(2*k, 0) * ni;
          _jacobianOplusXj.block<1, 2>(2*k+1, 0) = -rj;
          _jacobianOplusXi.block<1, 2>(2*k+1, 0) = dR * rj;
          z = dR.transpose() * ri - Rj.transpose() * dt;
          _jacobianOplusXj(2*k+1, 2) = rj.x() * z.y() - rj.y() * z.x();
          z = dR.transpose() * ri;
          _jacobianOplusXi(2*k+1, 2) = -rj.x() * z.y() + rj.y() * z.x();
          _jacobianOplusXj.block<1, 3>(2*k+1, 0) = _jacobianOplusXj.block<1, 3>(2*k+1, 0) * nj;
          _jacobianOplusXi.block<1, 3>(2*k+1, 0) = _jacobianOplusXi.block<1, 3>(2*k+1, 0) * nj;
      }

      /*
      cout << this->id() << " : " << _vertices[0]->id() << endl;
      for (int i = 0; i < 20; i++) {
          if (_jacobianOplusXi.col(0)[i] == 0.) break;
          cout << _jacobianOplusXi.col(0)[i] << ", ";
      }
      cout << endl;
      for (int i = 0; i < 20; i++) {
          if (_jacobianOplusXi.col(1)[i] == 0.) break;
          cout << _jacobianOplusXi.col(1)[i] << ", ";
      }
      cout << endl;
      for (int i = 0; i < 20; i++) {
          if (_jacobianOplusXi.col(2)[i] == 0.) break;
          cout << _jacobianOplusXi.col(2)[i] << ", ";
      }
      cout << endl << endl;
      cout << this->id() << " : " << _vertices[1]->id() << endl;
      for (int i = 0; i < 20; i++) {
          if (_jacobianOplusXj.col(0)[i] == 0.) break;
          cout << _jacobianOplusXj.col(0)[i] << ", ";
      }
      cout << endl;
      for (int i = 0; i < 20; i++) {
          if (_jacobianOplusXj.col(1)[i] == 0.) break;
          cout << _jacobianOplusXj.col(1)[i] << ", ";
      }
      cout << endl;
      for (int i = 0; i < 20; i++) {
          if (_jacobianOplusXj.col(2)[i] == 0.) break;
          cout << _jacobianOplusXj.col(2)[i] << ", ";
      }
      cout << endl << endl;
      */
  }
#endif

  bool EdgeProx::setMeasurementFromState(){
      /*
      VertexSE2Proximity* from_node = static_cast<VertexSE2Proximity*>(_vertices[0]);
      VertexSE2Proximity* to_node = static_cast<VertexSE2Proximity*>(_vertices[1]);
      
      _measurement = from_node->estimate().inverse() * to_node->estimate();
      */
    return true;
  }


  void EdgeProx::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
    /*
    VertexSE2Proximity* from_node = dynamic_cast<VertexSE2Proximity*>(_vertices[0]);
    VertexSE2Proximity* to_node = dynamic_cast<VertexSE2Proximity*>(_vertices[1]);
    if (from.count(from_node) > 0)
        to_node->setEstimate(from_node->estimate() * _measurement);
    else
        from_node->setEstimate(to_node->estimate() * _measurement.inverse());
    */
  }

  inline Vector2 EdgeProx::from_prox(int i) const {
      if (0 <= i && i < _num_prox_pairs)
          return dynamic_cast<VertexProx*>(_vertices[0])->proximity(_prox_pairs[0][i]);
      cerr << "out of range" << endl;
      return Vector2();
  };
  inline Vector2 EdgeProx::to_prox(int i) const {
      if (0 <= i && i < _num_prox_pairs)
          return dynamic_cast<VertexProx*>(_vertices[1])->proximity(_prox_pairs[1][i]);
      cerr << "out of range" << endl;
      return Vector2();
  };
  inline Vector2 EdgeProx::from_gr_prox(int i) const {
      if (0 <= i && i < _num_prox_pairs) {
          Vector2 fp = from_prox(i);
          if (fp == Vector2()) return Vector2();
          return (dynamic_cast<VertexProx*>(_vertices[0])->estimate() * fp);
      }
      cerr << "out of range" << endl;
      return Vector2();
  };
  inline Vector2 EdgeProx::to_gr_prox(int i) const {
      if (0 <= i && i < _num_prox_pairs) {
          Vector2 tp = to_prox(i);
          if (tp == Vector2()) return Vector2();
          return (dynamic_cast<VertexProx*>(_vertices[1])->estimate() * tp);
      }
      cerr << "out of range" << endl;
      return Vector2();
  };

#ifdef G2O_HAVE_OPENGL
  EdgeProxDrawAction::EdgeProxDrawAction()
      : DrawAction(typeid(EdgeProx).name()), _triangleX(nullptr), _triangleY(nullptr), _triangleZ(nullptr) {}

  bool EdgeProxDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_) {
      if (!DrawAction::refreshPropertyPtrs(params_))
          return false;
      if (_previousParams) {
          _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::N_Edge_Arrow_Length", .0f);
          _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::N_Edge_Width", 3.0f);
          _triangleZ = _previousParams->makeProperty<FloatProperty>(_typeName + "::P_Edge_Width", 2.0f);
      }
      else {
          _triangleX = 0;
          _triangleY = 0;
          _triangleZ = 0;
      }
      return true;
  }

  HyperGraphElementAction* EdgeProxDrawAction::operator()(HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_) {
      if (typeid(*element).name() != _typeName)
          return nullptr;

      refreshPropertyPtrs(params_);
      if (!_previousParams)
          return this;

      if (_show && !_show->value())
          return this;

      EdgeProx* e = static_cast<EdgeProx*>(element);
      VertexProx* from = static_cast<VertexProx*>(e->vertex(0));
      VertexProx* to = static_cast<VertexProx*>(e->vertex(1));
      if (!from && !to)
          return this;
      SE2 fromTransform;
      SE2 toTransform;
      glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
      glDisable(GL_LIGHTING);
      /*
      if (!from) {
          glColor3f(POSE_EDGE_GHOST_COLOR);
          toTransform = to->estimate();
          fromTransform = to->estimate() * e->measurement().inverse();
          // DRAW THE FROM EDGE AS AN ARROW
          glPushMatrix();
          glTranslatef((float)fromTransform.translation().x(), (float)fromTransform.translation().y(), 0.f);
          glRotatef((float)RAD2DEG(fromTransform.rotation().angle()), 0.f, 0.f, 1.f);
          opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value() * .3f);
          glPopMatrix();
      }
      else if (!to) {
          glColor3f(POSE_EDGE_GHOST_COLOR);
          fromTransform = from->estimate();
          toTransform = from->estimate() * e->measurement();
          // DRAW THE TO EDGE AS AN ARROW
          glPushMatrix();
          glTranslatef(toTransform.translation().x(), toTransform.translation().y(), 0.f);
          glRotatef((float)RAD2DEG(toTransform.rotation().angle()), 0.f, 0.f, 1.f);
          opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value() * .3f);
          glPopMatrix();
      }
      */
      { //else {
          glColor3f(POSE_EDGE_COLOR);
          fromTransform = from->estimate();
          toTransform = to->estimate();
      }
      //glGetFloatv(GL_LINE_WIDTH, &(float)_triangleY->value());
      if (_triangleY->value() != 0) {
          glLineWidth((float)_triangleY->value());
          glBegin(GL_LINES);
          glVertex3f((float)fromTransform.translation().x(), (float)fromTransform.translation().y(), 0.f);
          glVertex3f((float)toTransform.translation().x(), (float)toTransform.translation().y(), 0.f);
          glEnd();
      }
      //glGetFloatv(GL_LINE_WIDTH, &(float)_triangleZ->value());
      if (_triangleZ->value() != 0) {
          glLineWidth((float)_triangleZ->value());
          glColor3f(LANDMARK_EDGE_COLOR);
          glBegin(GL_LINES);
          for (int i = 0; i < e->num_prox_pairs(); i++) {
              glVertex3f((float)e->from_gr_prox(i).x(), (float)e->from_gr_prox(i).y(), 0.f);
              glVertex3f((float)e->to_gr_prox(i).x(), (float)e->to_gr_prox(i).y(), 0.f);
          }
      }
      glEnd();
      glPopAttrib();
      return this;
  }
#endif
}
