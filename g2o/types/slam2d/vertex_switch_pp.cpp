// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#include <string>
#include "vertex_switch_pp.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

    VertexSwitchingProxPair::VertexSwitchingProxPair() :
        BaseVertex<prox_maxN, std::array<number_t, prox_maxN>>(), _num_prox_pairs(0) { 
        setToOriginImpl();
    }
  void VertexSwitchingProxPair::oplusImpl(const number_t* update) {
      for (int i = 0; i < _num_prox_pairs; i++) {
          _estimate[i] += update[i];
          //_estimate[i] += update[i];
          //if(_estimate[i] + update[i] < 0.5)
          //    _estimate[i] = 0.;
          //else
          //    _estimate[i] = 1.;
          //_estimate[i] = std::max(std::min(_estimate[i], cst(30.)), cst(-4.5));
          //_estimate[i] = std::min(std::max(_estimate[i] + update[i], 1.), 1.e-2);
          //_estimate[i] = (_estimate[i] + update[i]) / (1. + _estimate[i] + update[i]);
      }
    for (int i = _num_prox_pairs; i < prox_maxN; i++)
        _estimate[i] = 1.;
  }

  bool VertexSwitchingProxPair::read(std::istream& is) {
    if (is.bad()) return false;
    std::array<number_t, prox_maxN> p;
    int n;
    is >> n;
    setToOriginImpl();
    _num_prox_pairs = std::min(n, prox_maxN);
    for (int i = 0; i < _num_prox_pairs; i++) {
        if (is.bad()) return true;
        is >> _estimate[i];
    }
    return true;
  }

  bool VertexSwitchingProxPair::write(std::ostream& os) const {
    os << _num_prox_pairs << " ";
    for (int i = 0; i < _num_prox_pairs && i < prox_maxN; i++)
      os << _estimate[i] << " " ;
    return true;
  }

} // end namespace
