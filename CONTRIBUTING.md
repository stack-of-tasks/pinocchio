# Contributing Guidelines
Thank you for your interest in contributing to `pinocchio`.
Whether it's a bug report, new feature, correction, or additional
documentation, we greatly value feedback and contributions from our community.

Please read through this document before submitting any issues or pull requests to ensure we have all the necessary
information to effectively respond to your bug report or contribution.

## Reporting Bugs/Feature Requests
We welcome you to use the GitHub issue tracker to report bugs or suggest features.

When filing an issue, please check [existing open][issues], or [recently closed][closed-issues], issues to make sure
 somebody else hasn't already reported the issue.
Please try to include as much information as you can. Details like these are incredibly useful:

  * A reproducible test case or series of steps
  * The version of our code being used
  * Any modifications you've made relevant to the bug
  * Anything unusual about your environment or deployment

## Contributing via Pull Requests
The following guidance should be up-to-date, but the documentation as found [here](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/) should prove as the final say.

Contributions via pull requests are much appreciated.
Before sending us a pull request, please ensure that:

 1. Limited scope. Your PR should do one thing or one set of things. Avoid adding “random fixes” to PRs. Put those on separate PRs.
 2. Give your PR a descriptive title. Add a short summary, if required.
 3. Make sure the pipeline is green.
 4. Don’t be afraid to request reviews from maintainers.
 5. New code = new tests. If you are adding new functionality, always make sure to add some tests exercising the code and serving as live documentation of your original intention.

To send us a pull request, please:

 1. Fork the repository.
 2. Modify the source; please focus on the specific change you are contributing. If you also reformat all the code, it will be hard for us to focus on your change.
 3. Ensure local tests pass. (`make test`)
 4. Commit to your fork using clear commit messages.
 5. Send a pull request, answering any default questions in the pull request interface.
 6. Pay attention to any automated CI failures reported in the pull request, and stay involved in the conversation.

GitHub provides additional documentation on [forking a repository](https://help.github.com/articles/fork-a-repo/) and [creating a pull request](https://help.github.com/articles/creating-a-pull-request/).

## Finding contributions to work on
Looking at the existing issues is a great way to find something to contribute on.
As this project, by default, uses the default GitHub issue labels (enhancement/bug/duplicate/help wanted/invalid/question/wontfix), looking at any ['help wanted'][help-wanted] issues is a great place to start.

## Licensing
Any contribution that you make to this repository will be under the BSD Clause 2 License, as dictated by that [license]:

~~~
BSD 2-Clause License

Copyright (c) 2014-2021, CNRS 
Copyright (c) 2018-2021, INRIA
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Pinocchio project.
~~~

[issues](https://github.com/stack-of-tasks/pinocchio/issues)
[closed-issues](https://github.com/stack-of-tasks/pinocchio/issues?utf8=%E2%9C%93&q=is%3Aissue%20is%3Aclosed%20)
[help-wanted](https://github.com/stack-of-tasks/pinocchio/issues?q=is%3Aopen+is%3Aissue+label%3A%22help+wanted%22)
[license](https://opensource.org/licenses/BSD-2-Clause)
