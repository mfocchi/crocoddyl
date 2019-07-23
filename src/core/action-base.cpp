#include "crocoddyl/core/action-base.hpp"

namespace crocoddyl {

ActionModelAbstract::ActionModelAbstract(StateAbstract* const state, const unsigned int& nu, const unsigned int& nr)
    : nx_(state->get_nx()),
      ndx_(state->get_ndx()),
      nu_(nu),
      nr_(nr),
      state_(state),
      unone_(Eigen::VectorXd::Zero(nu)) {
  assert(nx_ != 0);
  assert(ndx_ != 0);
  assert(nu_ != 0);
}

ActionModelAbstract::~ActionModelAbstract() {}

void ActionModelAbstract::calc(boost::shared_ptr<ActionDataAbstract>& data,
                               const Eigen::Ref<const Eigen::VectorXd>& x) {
  calc(data, x, unone_);
}

void ActionModelAbstract::calcDiff(boost::shared_ptr<ActionDataAbstract>& data,
                                   const Eigen::Ref<const Eigen::VectorXd>& x) {
  calcDiff(data, x, unone_);
}

const unsigned int& ActionModelAbstract::get_nx() const { return nx_; }

const unsigned int& ActionModelAbstract::get_ndx() const { return ndx_; }

const unsigned int& ActionModelAbstract::get_nu() const { return nu_; }

const unsigned int& ActionModelAbstract::get_nr() const { return nr_; }

StateAbstract* ActionModelAbstract::get_state() const { return state_; }

}  // namespace crocoddyl