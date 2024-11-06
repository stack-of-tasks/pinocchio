# Build and install from source with Pixi

To build **Pinocchio** from source the easiest way is to use [Pixi](https://pixi.sh/latest/#installation).

[Pixi](https://pixi.sh/latest/) is a cross-platform package management tool for developers that
will install all required dependencies in `.pixi` directory.
It's used by our CI agent so you have the guarantee to get the right dependencies.

Run the following command to install dependencies, configure, build and test the project:

```bash
pixi run test
```

The project will be built in the `build` directory.
You can run `pixi shell` and build the project with `cmake` and `ninja` manually.

