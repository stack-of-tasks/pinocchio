// Create the default CMakeOptions
function CMakeOptions() {
  this.BUILD_WITH_COLLISION_SUPPORT = "OFF";
  this.BUILD_WITH_CASADI_SUPPORT = "OFF";
  this.BUILD_WITH_AUTODIFF_SUPPORT = "OFF";
  this.BUILD_WITH_EXTRA_SUPPORT = "OFF";
  this.BUILD_WITH_CODEGEN_SUPPORT = "OFF";
  this.BUILD_WITH_OPENMP_SUPPORT = "OFF";
  this.BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT = "OFF";
  this.INSTALL_DOCUMENTATION = "ON";
  this.GENERATE_PYTHON_STUBS = "ON";
  this.BUILD_WITH_ACCELERATE_SUPPORT = "OFF";
  this.BUILD_WITH_SDF_SUPPORT = "OFF";
}

// Return an object with an association between labels and
// options that should be set to ON
function makeLabelToOptions() {
  const os = process.env.RUNNER_OS;
  if(os == "Windows") {
    return {
      build_all: [
        'BUILD_WITH_COLLISION_SUPPORT',
        'BUILD_WITH_CASADI_SUPPORT',
        'BUILD_WITH_AUTODIFF_SUPPORT',
        'BUILD_WITH_EXTRA_SUPPORT',
        'BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT',
        'BUILD_WITH_SDF_SUPPORT'
      ],
      build_collision: ['BUILD_WITH_COLLISION_SUPPORT'],
      build_casadi: ['BUILD_WITH_CASADI_SUPPORT'],
      build_autodiff: ['BUILD_WITH_AUTODIFF_SUPPORT'],
      build_extra: ['BUILD_WITH_EXTRA_SUPPORT'],
      build_codegen: [],
      build_openmp: [],
      build_mpfr: ['BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT'],
      build_sdf: ['BUILD_WITH_SDF_SUPPORT'],
      build_accelerate: ['BUILD_WITH_ACCELERATE_SUPPORT']
    };
  } else {
    return {
      build_all: [
        'BUILD_WITH_COLLISION_SUPPORT',
        'BUILD_WITH_CASADI_SUPPORT',
        'BUILD_WITH_AUTODIFF_SUPPORT',
        'BUILD_WITH_EXTRA_SUPPORT',
        'BUILD_WITH_CODEGEN_SUPPORT',
        'BUILD_WITH_OPENMP_SUPPORT',
        'BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT',
        'BUILD_WITH_SDF_SUPPORT'
      ],
      build_collision: ['BUILD_WITH_COLLISION_SUPPORT'],
      build_casadi: ['BUILD_WITH_CASADI_SUPPORT'],
      build_autodiff: ['BUILD_WITH_AUTODIFF_SUPPORT'],
      build_extra: ['BUILD_WITH_EXTRA_SUPPORT'],
      build_codegen: ['BUILD_WITH_CODEGEN_SUPPORT'],
      build_openmp: ['BUILD_WITH_OPENMP_SUPPORT'],
      build_mpfr: ['BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT'],
      build_sdf: ['BUILD_WITH_SDF_SUPPORT'],
      build_accelerate: ['BUILD_WITH_ACCELERATE_SUPPORT']
    };
  }
}

function getPullRequestNumber(ref) {
  // This assumes that the ref is in the form of `refs/pull/:prNumber/merge`
  const prNumber = ref.replace(/refs\/pull\/(\d+)\/merge/, '$1');
  return parseInt(prNumber, 10);
}

// Get activated labels
async function getLabelNames(github, context) {
  const {LABELS} = process.env;

  if(LABELS) {
    return [LABELS];
  } else {
    const prNumber = context.issue.number || getPullRequestNumber(context.ref);

    if(isNaN(prNumber)) {
      return [];
    }

    const { data } = await github.rest.pulls.get({
        owner: context.repo.owner,
        repo: context.repo.repo,
        pull_number: prNumber,
    });
    return data.labels.map(label => label.name);
  }
}

// Inspired by https://github.com/emmenko/action-verify-pr-labels/tree/master
module.exports = async ({github, context, core}) => {
  const labelToOptions = makeLabelToOptions();
  const labelNames = await getLabelNames(github, context);
  let options = new CMakeOptions();

  // Turn CMake options ON
  for (let label of labelNames) {
    if (labelToOptions[label]) {
      for (let x of labelToOptions[label]) {
        options[x] = "ON";
      }
    }
  }

  // Transform options object into command line output
  let cmakeFlags = '';
  for (let [k, v] of Object.entries(options)) {
    cmakeFlags += `-D${k}=${v} `;
  }

  console.log(cmakeFlags);
  core.setOutput("cmakeFlags", cmakeFlags);
  return;
}
