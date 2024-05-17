//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_serialization_csv_hpp__
#define __pinocchio_serialization_csv_hpp__

#include "pinocchio/serialization/fwd.hpp"

#include <Eigen/Core>
#include <fstream>

namespace pinocchio
{

  template<typename Derived>
  void toCSVfile(const std::string & filename, const Eigen::MatrixBase<Derived> & matrix)
  {
    const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    std::ofstream file(filename.c_str());
    file << matrix.format(CSVFormat);
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_serialization_csv_hpp__
