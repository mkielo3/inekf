#!/bin/bash

SRCDIR=$(pwd)

# Get python information
PYBIN="/opt/python/$PYTHON_VERSION/bin"
PYVER_NUM=$($PYBIN/python -c "import sys;print(sys.version.split(\" \")[0])")
PYTHONVER="$(basename $(dirname $PYBIN))"
export PATH=$PYBIN:$PATH

# Set the build directory 
BUILDDIR="$SRCDIR/build-$PYVER_NUM"
mkdir -p ${BUILDDIR}

# Install python dependencies
# ${PYBIN}/pip install -r python/requirements.txt

# run cmake & make
cd ${BUILDDIR}
cmake ${SRCDIR} -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON=ON \
    -DTESTS=OFF \
    -DEXAMPLES=OFF \
    -DCOL=2 \
    -DAUG=6 \
    -DVEC=6 \
    -DPYBIND11_PYTHON_VERSION=$PYVER_NUM; ec=$?
make -j2

# make wheel
# outputs them in BUILDDIR/python/dist
cd python
"${PYBIN}/python" setup.py bdist_wheel --python-tag=$PYTHONVER --plat-name=$PLAT

# Bundle external shared libraries into the wheel
# outputs them in BUILDDIR/python/wheelhouse
for whl in ./dist/*.whl; do
    echo $(auditwheel show $whl)
    auditwheel repair "$whl"
done

# rename wheel
# move it to SRCDIR/dist
mkdir -p $SRCDIR/dist
for whl in ./wheelhouse/*.whl; do
    new_filename=$(echo $whl | sed "s#\.none-manylinux2014_x86_64\.#.#g")
    new_filename=$(echo $new_filename | sed "s#\.manylinux2014_x86_64\.#.#g") # For 37 and 38
    new_filename=$(echo $new_filename | sed "s#-none-#-#g")
    mv $whl $new_filename
    mv $new_filename $SRCDIR/dist/
done
