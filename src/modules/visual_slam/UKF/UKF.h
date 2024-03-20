//
// Created by bene on 20.03.24.
//

#pragma once
#include <core/ukf.hpp>

namespace visukf {

class UKF : public ukf::core::Ukf<ukf::core::StateFields<>> {
  UKF() = default;
};
} // namespace visukf
