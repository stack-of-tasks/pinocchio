# Contributing Guidelines

Thank you for your interest in contributing to `pinocchio`.
Whether it's a bug report, a new feature, a fix, or documentation, we value every contribution.

Read this document before opening an issue or a pull request.

All communication on this project must follow the [Code of Conduct](../CODE_OF_CONDUCT.md).

## Table of contents

- [Contributing Guidelines](#contributing-guidelines)
   * [Reporting bugs and feature requests](#reporting-bugs-and-feature-requests)
   * [Asking questions](#asking-questions)
   * [Contributing via pull requests](#contributing-via-pull-requests)
      + [Choosing an issue](#choosing-an-issue)
      + [Set up the development environment](#set-up-the-development-environment)
      + [Pull request content](#pull-request-content)
      + [Running tests](#running-tests)
      + [Code style](#code-style)
      + [Changelog](#changelog)
   * [AI-assisted contributions](#ai-assisted-contributions)
      + [Responsibility](#responsibility)
      + [Disclosure](#disclosure)
      + [Communication](#communication)
      + [Translation](#translation)
      + [AI agent](#ai-agent)
   * [Licensing](#licensing)

## Reporting bugs and feature requests

Use the GitHub [issue tracker](https://github.com/stack-of-tasks/pinocchio/issues) to report bugs or suggest features.

Before opening an issue, check existing open and closed issues to avoid duplicates.

Use the appropriate template and give as much detail as possible.
If you don't use the template, maintainers may close your issue without explanation.

## Asking questions

Ask questions in the [discussions section](https://github.com/stack-of-tasks/pinocchio/discussions).
It separates development topics from community questions.
Questions posted in the issue tracker will be moved to the discussions section.

## Contributing via pull requests

### Choosing an issue

Every external contributor pull request needs an associated issue.
Open an issue first. Core developers will review it.

An issue is ready for a pull request when:

- It has the **ready** label.
- It is not assigned.
- It does not have the **core developers** label.

Issues with the **core developers** label are reserved for core developers.

If an issue meets these criteria, claim it with a short comment.

### Set up the development environment

The easiest way to set up a development environment is to use pixi, as described in the [build documentation](build.md).

See the CI workflows for examples with other package managers.

### Pull request content

To create a pull request, follow the GitHub guides on
[forking a repository](https://help.github.com/articles/fork-a-repo/) and
[creating a pull request](https://help.github.com/articles/creating-a-pull-request/).

In your pull request:

- Use a descriptive title and follow the pull request template.
- If the pull request is not ready for review, keep it as a draft.
- Keep it to a single self-contained change. Don't mix unrelated fixes.
- Follow the [code convention](./convention.md).
- Keep backward compatibility. Don't break the API.
- Write tests that cover your changes.
- Add an entry to the [changelog](../CHANGELOG.md).
- Make sure code style checks pass (`pixi run lint` or `pre-commit run --all-files`).
- Make sure the CI is green. Ask for help if you're stuck on a CI issue.
- Check all the appropriate items in the pull request template checklist.

### Running tests

To run the full test suite:

```bash
pixi run test
```

You can also run tests manually with `ctest`. Use the `-R` option to run a single test:

```bash
ctest --test-dir build --output-on-failure -R <test-name>
```

### Code style

Code style is enforced with [pre-commit](https://pre-commit.com/) hooks configured in [.pre-commit-config.yaml](../.pre-commit-config.yaml).
Before pushing your changes, run:

```bash
pixi run lint
```

or directly:

```bash
pre-commit run --all-files
```

### Changelog

Add changelog entries under the `## [Unreleased]` section,
in the matching [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
category (`Added`, `Changed`, `Fixed`, `Removed`).

Each entry is a short message followed by the pull request link, e.g.:

```
- Add spline joint to default joint collection ([#2784](https://github.com/stack-of-tasks/pinocchio/pull/2784))
```

CI and infra-only changes should not be listed. Use the **no changelog** label in this case.

## AI-assisted contributions

AI-assisted contributions are more and more common.
To avoid wasting maintainers time, follow the rules below.

AI-assisted contributions (issues, pull requests and discussions) are allowed under these rules.
Any AI-assisted contribution that doesn't follow them can be closed without explanation.

### Responsibility

You are responsible for the whole contribution.
Review it, understand it and be able to explain everything the AI assistant produced.
Don't shift this work to the reviewers.

The standard pull request guidelines apply.
If some AI-proposed code doesn't look necessary or doesn't address the issue, remove it.
Some AI assistants write too many tests. Keep only the tests you need, unit test maintenance is costly.

### Disclosure

Disclose that you used an AI assistant.
Describing what it did is optional.

### Communication

Human-to-human communication matters.
Never copy-paste AI-generated text into an issue, pull request description or comment.
Write it yourself, it shows you understand the topic.
If you want to quote the AI assistant, use a code or quote block.

### Translation

You can use an AI assistant for translation and grammar fixes.
You are still responsible for everything it produces.

### AI agent

Don't submit contributions automatically with an AI agent.
It breaks the responsibility and communication rules.

## Licensing

All contributions to this repository are under the BSD 2-Clause License, as stated in [LICENSE](../LICENSE).
