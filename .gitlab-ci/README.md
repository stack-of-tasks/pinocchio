```
docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/robotpkg:16.04 -f Dockerfile.robotpkg.16 .
docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/robotpkg:14.04 -f Dockerfile.robotpkg.14 .
docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/minimal:16.04 -f Dockerfile.minimal.16 .
docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/minimal:14.04 -f Dockerfile.minimal.14 .
docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/full:16.04 -f Dockerfile.full.16 .
docker build -t eur0c.laas.fr:4567/stack-of-tasks/pinocchio/full:14.04 -f Dockerfile.full.14 .
docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/robotpkg:16.04
docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/robotpkg:14.04
docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/minimal:16.04
docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/minimal:14.04
docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/full:16.04
docker push eur0c.laas.fr:4567/stack-of-tasks/pinocchio/full:14.04
```
