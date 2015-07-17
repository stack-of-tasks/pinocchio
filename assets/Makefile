#
# Copyright (c) 2014 CNRS-LAAS
# Author: Florent Lamiraux
#

ROS_REPO=https://github.com/ros
SOT_REPO=https://github.com/stack-of-tasks
LAAS_REPO=https://github.com/laas
HPP_REPO=https://github.com/humanoid-path-planner
# TINYXML2_REPO=https://github.com/leethomason

SRC_DIR=${DEVEL_DIR}/src
BUILD_TYPE=Debug

console-bridge_branch=master
console-bridge_repository=$(ROS_REPO)

urdfdom_headers_branch=master
urdfdom_headers_repository=$(ROS_REPO)

urdfdom_branch=master
urdfdom_repository=$(ROS_REPO)

# May be change the branch of eigenpy ? v1.2.0
eigenpy_branch=master
eigenpy_repository=$(SOT_REPO)

pinocchio_branch=master
pinocchio_repository=$(SOT_REPO)

collada-dom_branch=master
collada-dom_repository=${HPP_REPO}

gepetto-viewer_branch=devel
gepetto-viewer_repository=${HPP_REPO}

gepetto-viewer-corba_branch=devel
gepetto-viewer-corba_repository=${HPP_REPO}

viewer_branch=devel
viewer_repository=${HPP_REPO}

OpenSceneGraph-3.2.1_extra_flags= -DCOLLADA_DYNAMIC_LIBRARY=${DEVEL_DIR}/install/lib/libcollada14dom.so -DCOLLADA_INCLUDE_DIR=${DEVEL_DIR}/install/include/collada-dom -DLIB_POSTFIX=""

all: gepetto-viewer-corba.install \
	# pinocchio.install

console-bridge.configure.dep: console-bridge.checkout
urdfdom_headers.configure.dep: urdfdom_headers.checkout
urdfdom.configure.dep: urdfdom_headers.install console-bridge.install \
	urdfdom.checkout
eigenpy.configure.dep: eigenpy.checkout
pinocchio.configure.dep: eigenpy.install urdfdom.install \
	pinocchio.checkout
collada-dom.configure.dep: collada-dom.checkout
OpenSceneGraph-3.2.1.configure.dep: collada-dom.install \
	OpenSceneGraph-3.2.1.checkout
gepetto-viewer.configure.dep: OpenSceneGraph-3.2.1.install \
	gepetto-viewer.checkout
gepetto-viewer-corba.configure.dep: gepetto-viewer.install \
	gepetto-viewer-corba.checkout

status:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		cd "$$child_dir";\
		echo \
		"\033[1;36m------- Folder $$child_dir ---------------\033[0m"; \
		git --no-pager -c status.showUntrackedFiles=no status --short --branch; \
		cd ..; \
	done

log:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".log; \
	done

update:
	for d in hpp-util hpp-model hpp-model-urdf hpp-core hpp-template-corba hpp-corbaserver hpp-constraints hpp-wholebody-step hpp-wholebody-step-corba hpp-doc ; do \
	echo "Updating $$d";\
	make $$d.update; done

%.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		git clone --recursive -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
	fi \

%.update:
	cd ${SRC_DIR}/$(@:.update=);\
	git remote rm origin;\
	git remote add origin ${$(@:.update=)_repository}/$(@:.update=);\
	git fetch origin;\
	git checkout -b bce46g origin/${$(@:.update=)_branch};\
	git branch -D ${$(@:.update=)_branch};\
	git checkout -b ${$(@:.update=)_branch} bce46g;\
	git branch -D bce46g;\
	git submodule update

%.configure:%.configure.dep
	cd ${SRC_DIR}/$(@:.configure=);\
	mkdir -p build; \
	cd ${SRC_DIR}/$(@:.configure=)/build; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" ${$(@:.configure=)_extra_flags} ..

%.install:%.configure
	cd ${SRC_DIR}/$(@:.install=)/build;\
	make install

%.uninstall:
	cd ${SRC_DIR}/$(@:.uninstall=)/build;\
	make uninstall

%.clean:
	cd ${SRC_DIR}/$(@:.clean=)/build;\
	make clean

%.very-clean:
	rm -rf ${SRC_DIR}/$(@:.very-clean=)/build/*

%.status:
	cd ${SRC_DIR}/$(@:.status=); git status

%.log:
	@cd ${SRC_DIR}/$(@:.log=); \
	if [ -f .git/refs/heads/${$(@:.log=)_branch} ]; then \
		echo -n "$(@:.log=): "; \
		cat .git/refs/heads/${$(@:.log=)_branch}; \
	fi

OpenSceneGraph-3.2.1.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		wget http://www.openscenegraph.org/downloads/developer_releases/OpenSceneGraph-3.2.1.zip;\
		cd ${SRC_DIR}; unzip OpenSceneGraph-3.2.1.zip;\
		rm -f OpenSceneGraph-3.2.1.zip;\
	fi
