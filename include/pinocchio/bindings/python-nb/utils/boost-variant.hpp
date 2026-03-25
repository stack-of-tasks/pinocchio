// Copyright (c) 2026 INRIA
//
// Type caster for boost::variant, adapted from nanobind's std::variant caster
// (nanobind/stl/variant.h, Copyright (c) 2022 Yoshiki Matsuda and Wenzel Jakob).
//

#pragma once

#include "../fwd.hpp"

#include <boost/variant.hpp>

namespace nanobind::detail
{

  template<typename... Ts>
  struct type_caster<boost::variant<Ts...>>
  {
    using Value = boost::variant<Ts...>;
    static constexpr auto Name = union_name(make_caster<Ts>::Name...);

    template<typename T>
    using Cast = movable_cast_t<T>;
    template<typename T>
    static constexpr bool can_cast()
    {
      return true;
    }

    Value value;
    explicit operator Value *()
    {
      return &value;
    }
    explicit operator Value &()
    {
      return value;
    }
    explicit operator Value &&()
    {
      return std::move(value);
    }

    template<typename T>
    bool try_variant(const handle & src, uint8_t flags, cleanup_list * cleanup)
    {
      using CasterT = make_caster<T>;
      CasterT caster;
      if (
        !caster.from_python(src, flags_for_local_caster<T>(flags), cleanup)
        || !caster.template can_cast<T>())
        return false;
      value = caster.operator cast_t<T>();
      return true;
    }

    bool from_python(handle src, uint8_t flags, cleanup_list * cleanup) noexcept
    {
      if (flags & (uint8_t)cast_flags::convert)
      {
        if ((try_variant<Ts>(src, flags & ~(uint8_t)cast_flags::convert, cleanup) || ...))
          return true;
      }
      return (try_variant<Ts>(src, flags, cleanup) || ...);
    }

    template<typename T>
    static handle from_cpp(T * value, rv_policy policy, cleanup_list * cleanup)
    {
      if (!value)
        return none().release();
      return from_cpp(*value, policy, cleanup);
    }

    struct Visitor : public boost::static_visitor<handle>
    {
      rv_policy policy;
      cleanup_list * cleanup;
      Visitor(rv_policy p, cleanup_list * c)
      : policy(p)
      , cleanup(c)
      {
      }
      template<typename U>
      handle operator()(U && v) const
      {
        return make_caster<std::decay_t<U>>::from_cpp(std::forward<U>(v), policy, cleanup);
      }
    };

    template<typename T>
    static handle from_cpp(T && value, rv_policy policy, cleanup_list * cleanup) noexcept
    {
      return boost::apply_visitor(Visitor{policy, cleanup}, std::forward<T>(value));
    }
  };

} // namespace nanobind::detail
