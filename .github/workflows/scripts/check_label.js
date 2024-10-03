
// Inspired by https://github.com/emmenko/action-verify-pr-labels/tree/master
module.exports = async ({github, context, core}) => {
    const getPullRequestNumber = (ref) => {
        core.debug(`Parsing ref: ${ref}`);
        // This assumes that the ref is in the form of `refs/pull/:prNumber/merge`
        const prNumber = ref.replace(/refs\/pull\/(\d+)\/merge/, '$1');
        return parseInt(prNumber, 10);
    };

    const prNumber = context.issue.number || getPullRequestNumber(context.ref);

    const { data } = await github.rest.pulls.get({
        owner: context.repo.owner,
        repo: context.repo.repo,
        pull_number: prNumber,
    });
    const labelNames = data.labels.map(label => label.name);

    const labelFlags= [
        'build_all',
        'build_collision',
        'build_casadi',
        'build_autodiff',
        'build_codegen',
        'build_extra',
        'build_openmp',
        'build_mpfr',
        'build_sdf',
        'build_accelerate'
      ];

    var hasLabel = false;
    labelNames.forEach(label => {
        if (labelFlags.includes(label)) {
          hasLabel = true;
        }
      });

    try
    {
        const reviews = await github.rest.pulls.listReviews({
            owner: context.repo.owner,
            repo: context.repo.repo,
            pull_number: prNumber,
        });
        const allReviewsFromActionsBot = reviews.data.filter(
            (review) => review.user.login === 'github-actions[bot]'
        );

        if(allReviewsFromActionsBot.length > 0)
        {
            return;
        }

        if (labelNames.length > 0 && hasLabel) {
                return;
        }
        else
        {
            const reviewMessage = `👋 Hi,
This is a reminder message to assign an extra build label to this Pull Request if needed.
By default, this PR will be build with minimal build options (URDF support and Python bindings)
The possible extra labels are:

- **build_collision** (build Pinocchio with coal support)
- **build_casadi** (build Pinocchio with CasADi support)
- **build_autodiff** (build Pinocchio with CppAD support)
- **build_codegen** (build Pinocchio with CppADCodeGen support)
- **build_extra** (build Pinocchio with extra algorithms)
- **build_mpfr** (build Pinocchio with Boost.Multiprecision support)
- **build_sdf** (build Pinocchio with SDF parser)
- **build_accelerate** (build Pinocchio with APPLE Accelerate framework support)
- **build_all** (build Pinocchio with ALL the options stated above)

Thanks.
The Pinocchio development team.`;
            await github.rest.pulls.createReview({
                owner:context.repo.owner,
                repo: context.repo.repo,
                pull_number: prNumber,
                body: reviewMessage,
                event: 'COMMENT'
            });
        }
    } catch (error) {
    await core.setFailed(error.stack || error.message);
    }
    return;
}
