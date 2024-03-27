//
// Created by bene on 22.03.24.
//

#pragma once
#include "core/detail/traits.hpp"
#include <core/field.hpp>

namespace visukf {
class Velocity;
}
namespace visukf {

/**
 * @class Position
 *  Position represents the current position in ECEF coordinates
 *
 */
class Position
    : public ukf::core::Field<3,
                              ukf::core::StateDependencies<Position, Velocity>,
                              ukf::core::Inputs<>> {
public:
  Noising noising() const override;
  Data timeUpdate(float, const Position &,
                                     const Velocity &) const override;
  using Field::Field;
};

} // namespace visukf
