//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_cartesian_product_variant_hxx__
#define __pinocchio_cartesian_product_variant_hxx__

namespace pinocchio
{

template<typename _Scalar, int _Options, template<typename,int> class LieGroupCollectionTpl>
CartesianProductOperationVariantTpl<_Scalar,_Options,LieGroupCollectionTpl>
CartesianProductOperationVariantTpl<_Scalar,_Options,LieGroupCollectionTpl>::
operator* (const CartesianProductOperationVariantTpl& other) const
{
  CartesianProductOperationVariantTpl res;

  res.liegroups.reserve(liegroups.size() + other.liegroups.size());
  res.liegroups.insert(res.liegroups.end(), liegroups.begin(), liegroups.end());
  res.liegroups.insert(res.liegroups.end(), other.liegroups.begin(), other.liegroups.end());

  res.lg_nqs.reserve(lg_nqs.size() + other.lg_nqs.size());
  res.lg_nqs.insert(res.lg_nqs.end(), lg_nqs.begin(), lg_nqs.end());
  res.lg_nqs.insert(res.lg_nqs.end(), other.lg_nqs.begin(), other.lg_nqs.end());

  res.lg_nvs.reserve(lg_nvs.size() + other.lg_nvs.size());
  res.lg_nvs.insert(res.lg_nvs.end(), lg_nvs.begin(), lg_nvs.end());
  res.lg_nvs.insert(res.lg_nvs.end(), other.lg_nvs.begin(), other.lg_nvs.end());

  res.m_nq = m_nq + other.m_nq;
  res.m_nv = m_nv + other.m_nv;

  if(liegroups.size() > 0)
    res.m_name = m_name;
  if(other.liegroups.size() > 0) {
    if (liegroups.size() > 0)
      res.m_name += " x ";
    res.m_name += other.m_name;
  }

  res.m_neutral.resize(res.m_nq);
  res.m_neutral.head(m_nq) = m_neutral;
  res.m_neutral.tail(other.m_nq) = other.m_neutral;

  return res;
}

template<typename _Scalar, int _Options, template<typename,int> class LieGroupCollectionTpl>
CartesianProductOperationVariantTpl<_Scalar,_Options,LieGroupCollectionTpl>&
CartesianProductOperationVariantTpl<_Scalar,_Options,LieGroupCollectionTpl>::
operator*= (const CartesianProductOperationVariantTpl& other)
{
  liegroups.insert(liegroups.end(), other.liegroups.begin(), other.liegroups.end());

  lg_nqs.insert(lg_nqs.end(), other.lg_nqs.begin(), other.lg_nqs.end());
  lg_nvs.insert(lg_nvs.end(), other.lg_nvs.begin(), other.lg_nvs.end());

  m_nq += other.m_nq;
  m_nv += other.m_nv;

  if(other.liegroups.size() > 0) {
    if (liegroups.size())
      m_name += " x ";
    m_name += other.m_name;
  }

  m_neutral.conservativeResize(m_nq);
  m_neutral.tail(other.m_nq) = other.m_neutral;

  return *this;
}

template<typename _Scalar, int _Options, template<typename,int> class LieGroupCollectionTpl>
bool CartesianProductOperationVariantTpl<_Scalar,_Options,LieGroupCollectionTpl>::
isEqual_impl (const CartesianProductOperationVariantTpl& other) const
{
  if (liegroups.size() != other.liegroups.size())
    return false;
  for(size_t k = 0; k < liegroups.size(); ++k)
    if (liegroups[k].isDifferent_impl(other.liegroups[k]))
      return false;
  return true;
}

template<typename _Scalar, int _Options, template<typename,int> class LieGroupCollectionTpl>
template <typename LieGroup1, typename LieGroup2>
bool CartesianProductOperationVariantTpl<_Scalar,_Options,LieGroupCollectionTpl>::
isEqual(const CartesianProductOperation<LieGroup1, LieGroup2> & other) const
{
  if (liegroups.size() != 2)
    return false;
  if (liegroups[0].isDifferent_impl(LieGroupGeneric(other.lg1)))
    return false;
  if (liegroups[1].isDifferent_impl(LieGroupGeneric(other.lg2)))
    return false;
  return true;
}

} // namespace pinocchio

#endif // ifndef __pinocchio_cartesian_product_variant_hxx__
