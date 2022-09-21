///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/contact-friction-cone.hpp"

namespace crocoddyl {

template <typename Scalar>
ResidualModelContactFrictionConeTpl<Scalar>::ResidualModelContactFrictionConeTpl(
    boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id, const FrictionCone& fref,
    const std::size_t nu)
    : Base(state, fref.get_nf() + 1, nu, true, true, true),
      id_(id),
      fref_(fref),
      pin_model_(state->get_pinocchio())
{
    pinocchio::Model m = *pin_model_.get();
    parent_id_ = m.frames[id].parent;
}

template <typename Scalar>
ResidualModelContactFrictionConeTpl<Scalar>::ResidualModelContactFrictionConeTpl(
    boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id, const FrictionCone& fref)
    : Base(state, fref.get_nf() + 1),
      id_(id),
      fref_(fref)
{
    pinocchio::Model m = *pin_model_.get();
    parent_id_ = m.frames[id].parent;
}



template <typename Scalar>
ResidualModelContactFrictionConeTpl<Scalar>::~ResidualModelContactFrictionConeTpl() {}

template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                       const Eigen::Ref<const VectorXs>&,
                                                       const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // Compute the residual of the friction cone. Note that we need to transform the force
  // to the world frame
  pinocchio::updateGlobalPlacements(*pin_model_.get(), *d->pinocchio);
  d->oF = d->pinocchio->oMi[parent_id_].actInv(d->contact->f);

  // new:
  // data->r.noalias() = fref_.get_A() * d->oF.linear();

  // original:
  data->r.noalias() = fref_.get_A() * d->contact->jMf.actInv(d->contact->f).linear();
}

template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                       const Eigen::Ref<const VectorXs>&) {
  data->r.setZero();
}

template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                           const Eigen::Ref<const VectorXs>& x,
                                                           const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  // original:
  // const MatrixXs& df_dx = d->contact->df_dx;
  // new:
  const MatrixXs& df_dq = d->contact->df_dx.leftCols(state_->get_nv());
  const MatrixXs& df_dv = d->contact->df_dx.rightCols(state_->get_nv());
  const MatrixXs& df_du = d->contact->df_du;
  const MatrixX3s& A = fref_.get_A();

  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());

  d->oJi.setZero();
  pinocchio::computeJointJacobians(*pin_model_.get(), *d->pinocchio, q);
  pinocchio::getJointJacobian(*pin_model_.get(), *d->pinocchio, parent_id_, pinocchio::WORLD, d->oJi);

  d->fskew = pinocchio::skew(d->oF.linear());
  d->oMi = d->pinocchio->oMi[parent_id_];


  switch (d->contact_type) {
    case Contact2D: {
      // Valid for xz plane (TODO)
      data->Rx.setZero();
      data->Ru.setZero();
      // original:
      // data->Rx.noalias() = A.col(0) * df_dx.row(0);
      // data->Ru.noalias() = A.col(0) * df_du.row(0);
      break;
    }
    case Contact3D:
      // new:
      d->Rq.noalias() = A * ( d->oMi.rotation().transpose()*df_dq - d->fskew * d->oJi.bottomRows(3));
      d->Rv.noalias() = A * d->oMi.rotation().transpose() * df_dv;
      data->Rx.leftCols(state_->get_nv())  = d->Rq;
      data->Rx.rightCols(state_->get_nv()) = d->Rv;
      data->Ru.noalias() = A * d->oMi.rotation().transpose() * df_du;

      // original:
      // data->Rx.noalias() = A * df_dx;
      // data->Ru.noalias() = A * df_du;
      break;
    case Contact6D:
      //d->fskew = pinocchio::skew(d->contact->f.linear());
      //data->Rx.noalias() = A * (d->pinocchio->oMi[parent_id_].rotation() * df_dx.template topRows<3>() - d->fskew * d->oJi.bottomRows(3) );
      //data->Ru.noalias() = A * d->pinocchio->oMi[parent_id_].rotation() * df_du.template topRows<3>();
      data->Rx.setZero();
      data->Ru.setZero();
      // original:
      // data->Rx.noalias() = A * df_dx.template topRows<3>();
      // data->Ru.noalias() = A * df_du.template topRows<3>();

      break;
    default:
      break;
  }
}

template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                           const Eigen::Ref<const VectorXs>&) {
  data->Rx.setZero();
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelContactFrictionConeTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::print(std::ostream& os) const {
  boost::shared_ptr<StateMultibody> s = boost::static_pointer_cast<StateMultibody>(state_);
  os << "ResidualModelContactFrictionCone {frame=" << s->get_pinocchio()->frames[id_].name << ", mu=" << fref_.get_mu()
     << "}";
}

template <typename Scalar>
pinocchio::FrameIndex ResidualModelContactFrictionConeTpl<Scalar>::get_id() const {
  return id_;
}

template <typename Scalar>
const FrictionConeTpl<Scalar>& ResidualModelContactFrictionConeTpl<Scalar>::get_reference() const {
  return fref_;
}

template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::set_id(const pinocchio::FrameIndex id) {
  id_ = id;
}


template <typename Scalar>
pinocchio::FrameIndex ResidualModelContactFrictionConeTpl<Scalar>::get_parent_id() const {
  return parent_id_;
}


template <typename Scalar>
void ResidualModelContactFrictionConeTpl<Scalar>::set_reference(const FrictionCone& reference) {
  fref_ = reference;
}

}  // namespace crocoddyl
