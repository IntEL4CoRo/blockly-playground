# blockly-playground

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/IntEL4CoRo/blockly-playground.git/stretch?urlpath=lab/tree/examples/stretch.jpblockly)

Block-based programming for household robots.


Examples on Binderhub:
- https://binder.intel4coro.de/v2/gh/IntEL4CoRo/blockly-playground.git/stretch?urlpath=lab/tree/examples/stretch.jpblockly

## Software Components

### Blockly

Blockly is a library from Google for building beginner-friendly block-based programming languages.

Docs: https://developers.google.com/blockly/guides/overview

Repo: https://github.com/google/blockly

### Giskardpy

Giskard is an open source motion planning framework for ROS, which uses constraint and optimization based task space control to generate trajectories for the whole body of mobile manipulators.

Repo: https://github.com/SemRoCo/giskardpy/tree/devel


## Development

### Build Docker Image Locally (Under repo directory)

- Run Docker image with X-forwarding

  ```bash
  docker compose -f ./binder/docker-compose.yml up  --build
  ```

- Open Web browser and go to http://localhost:8888/lab/tree/examples/stretch.jpblockly

- To stop and remove container:

  ```bash
  docker compose -f ./binder/docker-compose.yml down
  ```
