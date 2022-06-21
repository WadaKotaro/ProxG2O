// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_SWITCHING_PROX_PAIR_H
#define G2O_SWITCHING_PROX_PAIR_H

#include "g2o/core/base_ternary_edge.h"
#include "parameter_spp_weight.h"

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam2d_api.h"

#ifndef prox_maxN
#define prox_maxN 10
#endif

namespace g2o {

    class G2O_TYPES_SLAM2D_API VertexSwitchingProxPair : public BaseVertex<prox_maxN, std::array<number_t, prox_maxN>>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexSwitchingProxPair();

        virtual void setToOriginImpl() {
            //for (int i = 0; i < prox_maxN; i++)
            //    _estimate[i] = 1.;
            _estimate.fill(1.);
        }

        virtual void oplusImpl(const number_t* update);

        virtual bool setEstimateDataImpl(const number_t* est) {
            for (int i = 0; i < prox_maxN; i++)
                _estimate[i] = est[i];
            return true;
        }

        virtual bool getEstimateData(number_t* est) const {
            for (int i = 0; i < prox_maxN; i++)
                est[i] = _estimate[i];
            return true;
        }

        virtual int estimateDimension() const { return prox_maxN; }

        virtual bool setMinimalEstimateDataImpl(const number_t* est) {
            return setEstimateData(est);
        }

        virtual bool getMinimalEstimateData(number_t* est) const {
            return getEstimateData(est);
        }

        virtual int minimalEstimateDimension() const { return prox_maxN; }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    private:
        int _num_prox_pairs;
    };

} // end namespace

#endif
