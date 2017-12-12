#!/bin/bash

set -e

for tag in 14.04 16.04 zesty
do
    (
    docker pull ubuntu:$tag
    docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/robotpkg:$tag -f Dockerfile.robotpkg.$tag .
    docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/robotpkg:$tag &
    docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/minimal:$tag -f Dockerfile.minimal.$tag .
    docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/minimal:$tag &
    docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/full:$tag -f Dockerfile.full.$tag .
    docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/full:$tag
    ) &
done

wait
