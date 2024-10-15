// Inspired by https://github.com/emmenko/action-verify-pr-labels/tree/master
module.exports = async ({github, context, core}) => {
    const getPullRequestNumber = (ref) => {
        core.debug(`Parsing ref: ${ref}`);
        // This assumes that the ref is in the form of `refs/pull/:prNumber/merge`
        const prNumber = ref.replace(/refs\/pull\/(\d+)\/merge/, '$1');
        return parseInt(prNumber, 10);
    };

    const prNumber = context.issue.number || getPullRequestNumber(context.ref);

    let cmakeFlags = '';
    // get os process is run on
    const os = process.env.RUNNER_OS;
    var labelFlags;
    if(os == "Windows")
    {
        labelFlags = {
            build_all: [
                ' -DBUILD_WITH_COLLISION_SUPPORT=ON',
                ' -DBUILD_WITH_CASADI_SUPPORT=ON',
                ' -DBUILD_WITH_AUTODIFF_SUPPORT=ON',
                ' -DBUILD_WITH_EXTRA_SUPPORT=ON',
                ' -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=ON',
                ' -DINSTALL_DOCUMENTATION=ON',
                ' -DGENERATE_PYTHON_STUBS=ON',
                ' -DBUILD_WITH_ACCELERATE_SUPPORT=OFF',
                ' -DBUILD_WITH_SDF_SUPPORT=ON'
            ],
            build_collision: ' -DBUILD_WITH_COLLISION_SUPPORT=ON',
            build_casadi: ' -DBUILD_WITH_CASADI_SUPPORT=ON',
            build_autodiff: ' -DBUILD_WITH_AUTODIFF_SUPPORT=ON',
            build_extra: ' -DBUILD_WITH_EXTRA_SUPPORT=ON',
            build_mpfr: ' -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=ON',
            build_sdf: ' -DBUILD_WITH_SDF_SUPPORT=ON',
            build_accelerate: ' -DBUILD_WITH_ACCELERATE_SUPPORT=ON'
        };
    }
    else
    {
        labelFlags = {
            build_all: [
                ' -DBUILD_WITH_COLLISION_SUPPORT=ON',
                ' -DBUILD_WITH_CASADI_SUPPORT=ON',
                ' -DBUILD_WITH_AUTODIFF_SUPPORT=ON',
                ' -DBUILD_WITH_EXTRA_SUPPORT=ON',
                ' -DBUILD_WITH_OPENMP_SUPPORT=ON',
                ' -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=ON',
                ' -DINSTALL_DOCUMENTATION=ON',
                ' -DBUILD_WITH_CODEGEN_SUPPORT=ON',
                ' -DGENERATE_PYTHON_STUBS=ON',
                ' -DBUILD_WITH_ACCELERATE_SUPPORT=OFF',
                ' -DBUILD_WITH_SDF_SUPPORT=ON'
            ],
            build_collision: ' -DBUILD_WITH_COLLISION_SUPPORT=ON',
            build_casadi: ' -DBUILD_WITH_CASADI_SUPPORT=ON',
            build_autodiff: ' -DBUILD_WITH_AUTODIFF_SUPPORT=ON',
            build_codegen: ' -DBUILD_WITH_CODEGEN_SUPPORT=ON',
            build_extra: ' -DBUILD_WITH_EXTRA_SUPPORT=ON',
            build_openmp: ' -DBUILD_WITH_OPENMP_SUPPORT=ON',
            build_mpfr: ' -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=ON',
            build_sdf: ' -DBUILD_WITH_SDF_SUPPORT=ON',
            build_accelerate: ' -DBUILD_WITH_ACCELERATE_SUPPORT=ON'
        };
    }

    // Get the GitHub event name that triggered the workflow
    const eventName = process.env.GITHUB_EVENT_NAME;
    if (eventName == "schedule")
    {
        cmakeFlags += labelFlags['build_all'].join(' ');
        console.log(cmakeFlags);
        core.setOutput("cmakeFlags", cmakeFlags);
        return;
    }

    if(isNaN(prNumber))
    {
        core.setOutput("cmakeFlags", cmakeFlags);
        return;
    }

    const { data } = await github.rest.pulls.get({
        owner: context.repo.owner,
        repo: context.repo.repo,
        pull_number: prNumber,
    });
    const labelNames = data.labels.map(label => label.name);

    labelNames.forEach(label => {
        if (labelFlags[label]) {
            if (Array.isArray(labelFlags[label])) {
                cmakeFlags += labelFlags[label].join(' ');
            } else {
                cmakeFlags += labelFlags[label];
            }
        }
    });
    console.log(cmakeFlags);
    core.setOutput("cmakeFlags", cmakeFlags);
    return;
}
