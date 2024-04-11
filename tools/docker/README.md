# docker tools

This directory provides tools to build moteus using docker for environment and virtualisation management.

Moteus manages its own dependencies well via Bazel, but it can still only be run on x86_64 Ubuntu.

This container + script (./env) provides a way to run the build and test tools on other platforms (eg. my M1 Macbook).

| OS | State |
| --- | ----------- |
| Linux | Untested - expected to work |
| OSx | Working 👌 |
| Windows | Untested - expected to work |

## Setup Notes

This approach has been developed using [docker desktop](https://www.docker.com/products/docker-desktop/)

For OSx specifically, enabling the option "Use Rosetta for x86/amd64 emulation on Apple Silicon" should vastly improve performance.


## Usage

