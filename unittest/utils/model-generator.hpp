//
// Copyright (c) 2015-2020 CNRS
// Copyright (c) 2018-2025 INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include <iostream>

namespace pinocchio
{

  template<typename D>
  void addJointAndBody(
    Model & model,
    const JointModelBase<D> & jmodel,
    const Model::JointIndex parent_id,
    const SE3 & joint_placement,
    const std::string & name,
    const Inertia & Y)
  {
    Model::JointIndex idx;
    typedef typename D::TangentVector_t TV;
    typedef typename D::ConfigVector_t CV;

    idx = model.addJoint(
      parent_id, jmodel, joint_placement, name + "_joint", TV::Zero(),
      1e3 * (TV::Random() + TV::Constant(1)), 1e3 * (CV::Random() - CV::Constant(1)),
      1e3 * (CV::Random() + CV::Constant(1)));

    model.appendBodyToJoint(idx, Y, SE3::Identity());
  }

  void buildAllJointsModel(Model & model)
  {
    addJointAndBody(
      model, JointModelFreeFlyer(), model.getJointId("universe"), SE3::Identity(), "freeflyer",
      Inertia::Random());
    addJointAndBody(
      model, JointModelSpherical(), model.getJointId("freeflyer_joint"), SE3::Identity(),
      "spherical", Inertia::Random());
    addJointAndBody(
      model, JointModelPlanar(), model.getJointId("spherical_joint"), SE3::Identity(), "planar",
      Inertia::Random());
    addJointAndBody(
      model, JointModelRX(), model.getJointId("planar_joint"), SE3::Identity(), "rx",
      Inertia::Random());
    addJointAndBody(
      model, JointModelPX(), model.getJointId("rx_joint"), SE3::Identity(), "px",
      Inertia::Random());
    addJointAndBody(
      model, JointModelHX(1.0), model.getJointId("px_joint"), SE3::Identity(), "hx",
      Inertia::Random());
    addJointAndBody(
      model, JointModelPrismaticUnaligned(SE3::Vector3(1, 0, 0)), model.getJointId("hx_joint"),
      SE3::Identity(), "pu", Inertia::Random());
    addJointAndBody(
      model, JointModelRevoluteUnaligned(SE3::Vector3(0, 0, 1)), model.getJointId("pu_joint"),
      SE3::Identity(), "ru", Inertia::Random());
    addJointAndBody(
      model, JointModelHelicalUnaligned(SE3::Vector3(0, 0, 1), 1.0), model.getJointId("ru_joint"),
      SE3::Identity(), "hu", Inertia::Random());
    addJointAndBody(
      model, JointModelSphericalZYX(), model.getJointId("hu_joint"), SE3::Identity(),
      "sphericalZYX", Inertia::Random());
    addJointAndBody(
      model, JointModelTranslation(), model.getJointId("sphericalZYX_joint"), SE3::Identity(),
      "translation", Inertia::Random());
  }

  void toFull(
    const Model & model_full,
    const Model & model_mimic,
    const std::vector<pinocchio::JointIndex> & mimicking_ids,
    const std::vector<double> & ratio,
    const std::vector<double> & offset,
    const Eigen::VectorXd & q,
    Eigen::VectorXd & q_full)
  {
    for (int n = 1; n < model_full.njoints; n++)
    {
      double joint_ratio = 1.0;
      double joint_offset = 0.0;
      auto it = std::find(mimicking_ids.begin(), mimicking_ids.end(), n);
      if (it != mimicking_ids.end()) // If n was found
      {
        joint_ratio = ratio[size_t(std::distance(mimicking_ids.begin(), it))];
        joint_offset = offset[size_t(std::distance(mimicking_ids.begin(), it))];
      }
      model_full.joints[size_t(n)].JointMappedConfigSelector(q_full) =
        joint_ratio * model_mimic.joints[size_t(n)].JointMappedConfigSelector(q)
        + joint_offset * Eigen::VectorXd::Ones(model_full.joints[size_t(n)].nq());
    }
  }

  void mimicTransformMatrix(
    const Model & model_full,
    const Model & model_mimic,
    const std::vector<pinocchio::JointIndex> & /*mimicked_ids*/,
    const std::vector<pinocchio::JointIndex> & mimicking_ids,
    const std::vector<double> & ratios,
    Eigen::MatrixXd & G)
  {
    // Initialize G as a zero matrix
    G.resize(model_full.nv, model_mimic.nv);
    G.setZero();
    // Set ones on the pseudo-diagonal
    for (int j = 1; j < model_full.njoints; ++j)
    {
      if (std::find(mimicking_ids.begin(), mimicking_ids.end(), j) == mimicking_ids.end())
        G.block(
           model_full.joints[size_t(j)].idx_v(), model_mimic.joints[size_t(j)].idx_v(),
           model_full.joints[size_t(j)].nv(), model_mimic.joints[size_t(j)].nv())
          .setIdentity();
    }

    // Set specific values for mimicked-mimicking joint pairs
    for (size_t i = 0; i < mimicking_ids.size(); ++i)
    {
      G(model_full.idx_vs[mimicking_ids[i]], model_mimic.idx_vs[mimicking_ids[i]]) = ratios[i];
    }
  }

  class MimicTestCases
  {
  public:
    static const int N_CASES = 6;

    pinocchio::Model model_mimic;
    pinocchio::Model model_full;
    std::vector<pinocchio::JointIndex> mimicked_ids;
    std::vector<pinocchio::JointIndex> mimicking_ids;
    Eigen::MatrixXd G;
    std::vector<double> ratios;
    std::vector<double> offsets;

    MimicTestCases(int case_i, bool verbose = false)
    : mimicked_ids()
    , mimicking_ids()
    , ratios()
    , offsets()
    {
      pinocchio::buildModels::humanoidRandom(model_full);
      model_full.lowerPositionLimit.head<3>().fill(-1.);
      model_full.upperPositionLimit.head<3>().fill(1.);

      // Select mimic joints based on test case
      if (verbose)
      {
        std::cout << "Mimic test case : ";
      }
      switch (case_i)
      {
      case 0:
        if (verbose)
        {
          std::cout << "Mimicked/mimicking parent/child";
        }
        mimicked_ids.push_back(model_full.getJointId("rleg1_joint"));
        mimicking_ids.push_back(model_full.getJointId("rleg2_joint"));
        ratios.push_back(2.5);
        offsets.push_back(0.75);
        break;
      case 1:
        if (verbose)
        {
          std::cout << "Spaced mimicked/mimicking";
        }
        mimicked_ids.push_back(model_full.getJointId("rleg1_joint"));
        mimicking_ids.push_back(model_full.getJointId("rleg4_joint"));
        ratios.push_back(2.5);
        offsets.push_back(0.75);
        break;
      case 2:
        if (verbose)
        {
          std::cout << "Parallel mimic";
        }
        mimicked_ids.push_back(model_full.getJointId("lleg1_joint"));
        mimicking_ids.push_back(model_full.getJointId("rleg1_joint"));
        ratios.push_back(2.5);
        offsets.push_back(0.75);
        break;
      case 3:
        if (verbose)
        {
          std::cout << "Double mimic, not same mimicked";
        }
        mimicked_ids.push_back(model_full.getJointId("lleg1_joint"));
        mimicking_ids.push_back(model_full.getJointId("rleg1_joint"));
        ratios.push_back(2.5);
        offsets.push_back(0.75);

        mimicked_ids.push_back(model_full.getJointId("rarm1_joint"));
        mimicking_ids.push_back(model_full.getJointId("larm1_joint"));
        ratios.push_back(3.2);
        offsets.push_back(8);
        break;
      case 4:
        if (verbose)
        {
          std::cout << "Double mimic, same mimicked";
        }
        mimicked_ids.push_back(model_full.getJointId("lleg1_joint"));
        mimicking_ids.push_back(model_full.getJointId("rleg1_joint"));
        ratios.push_back(2.5);
        offsets.push_back(0.75);

        mimicked_ids.push_back(model_full.getJointId("lleg1_joint"));
        mimicking_ids.push_back(model_full.getJointId("lleg2_joint"));
        ratios.push_back(3.2);
        offsets.push_back(8);
        break;
      case 5:
        if (verbose)
        {
          std::cout << "Mimicking terminal joint";
        }
        mimicked_ids.push_back(model_full.getJointId("larm5_joint"));
        mimicking_ids.push_back(model_full.getJointId("larm6_joint"));
        ratios.push_back(2.5);
        offsets.push_back(0.75);

        break;
      default:
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "No mimic test case number " << case_i << ". Max number " << N_CASES - 1);
      }

      if (verbose)
      {
        std::cout << std::endl;
      }

      buildMimicModel(model_full, mimicked_ids, mimicking_ids, ratios, offsets, model_mimic);
      mimicTransformMatrix(model_full, model_mimic, mimicked_ids, mimicking_ids, ratios, G);
    }

    void toFull(const Eigen::VectorXd & q, Eigen::VectorXd & q_full) const
    {
      pinocchio::toFull(model_full, model_mimic, mimicking_ids, ratios, offsets, q, q_full);
    }
  };
} // namespace pinocchio
