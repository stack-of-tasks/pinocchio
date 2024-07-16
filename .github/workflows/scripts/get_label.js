module.exports = async ({github, context, core}) => {
    const getPullRequestNumber = (ref) => {
        core.debug(`Parsing ref: ${ref}`);
        // This assumes that the ref is in the form of `refs/pull/:prNumber/merge`
        const prNumber = ref.replace(/refs\/pull\/(\d+)\/merge/, '$1');
        return parseInt(prNumber, 10);
    };
  
    const prNumber = context.issue.number || getPullRequestNumber(context.ref);

    let cmakeFlags = '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=${GITHUB_WORKSPACE}/.github/workflows/cmake/linux-debug-toolchain.cmake -DPYTHON_EXECUTABLE=$(which python3)';
    console.log(cmakeFlags);
    if(isNaN(prNumber))
    {
        core.setOutput("cmakeFlags", cmakeFlags);
        return cmakeFlags;
    }

    const { data } = await github.rest.pulls.get({
        owner: context.repo.owner,
        repo: context.repo.repo,
        pull_number: prNumber,
    });
    const labelNames = data.labels.map(label => label.name);

    labelNames.forEach((label, index) => {
        // Collision
        if(label == 'build_collision')
        {
            cmakeFlags += ' -DBUILD_WITH_COLLISION_SUPPORT=ON';
        }
        // URDF
        if(label == "build_with_urdf")
        {
            cmakeFlags += ' -DBUILD_WITH_URDF_SUPPORT=ON';
        }
        else
        {
            cmakeFlags += ' -DBUILD_WITH_URDF_SUPPORT=OFF';
        }

    });

    core.setOutput("cmakeFlags", cmakeFlags);
    return cmakeFlags;
}