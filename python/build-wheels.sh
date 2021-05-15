#!/bin/bash

CURRDIR=$(pwd)

# Set the build directory 
BUILDDIR="/io/build"
mkdir -p ${BUILDDIR}

# FIX auditwheel
# https://github.com/pypa/auditwheel/issues/136
# echo /opt/_internal/*/*/*/*/auditwheel
# cd /opt/_internal/tools/lib64/python3.7/site-packages/auditwheel
# patch -p2 < python/auditwheel.txt
# cd $BUILDDIR

# Get python information
PYBIN="/opt/python/$PYTHON_VERSION/bin"
PYVER_NUM=$($PYBIN/python -c "import sys;print(sys.version.split(\" \")[0])")
export PATH=$PYBIN:$PATH

# Install python dependencies
${PYBIN}/pip install -r python/requirements.txt

# run cmake & make
cd ${BUILDDIR}
cmake ${CURRDIR} -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON=ON \
    -DTESTS=OFF \
    -DEXAMPLES=OFF \
    -DCOL=1 \
    -DAUG=1 \
    -DVEC=1 \
    -DPYBIND11_PYTHON_VERSION=$PYVER_NUM; ec=$?
# if [ $ec -ne 0 ]; then
#     echo "Error:"
#     cat ./CMakeCache.txt
#     exit $ec
# fi
set -e -x
make -j2

# make wheels
mkdir -p /io/wheelhouse
cd python
"${PYBIN}/python" setup.py bdist_wheel --python-tag=$PYTHONVER --plat-name=$PLAT

# Bundle external shared libraries into the wheels
for whl in ./dist/*.whl; do
    auditwheel repair "$whl" -w /io/wheelhouse/
done

for whl in /io/wheelhouse/*.whl; do
    new_filename=$(echo $whl | sed "s#\.none-manylinux2014_x86_64\.#.#g")
    new_filename=$(echo $new_filename | sed "s#\.manylinux2014_x86_64\.#.#g") # For 37 and 38
    new_filename=$(echo $new_filename | sed "s#-none-#-#g")
    mv $whl $new_filename
done