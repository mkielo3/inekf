FROM gcc:11

RUN apt-get update
RUN apt-get install -qy cmake python3-dev python3-pip

RUN git clone https://gitlab.com/libeigen/eigen.git
RUN cd eigen && mkdir build && cd build && cmake .. && make install

RUN pip3 install numpy matplotlib