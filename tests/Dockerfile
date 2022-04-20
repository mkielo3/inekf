FROM gcc:11

# Install packages required for build
RUN apt-get update
RUN apt-get install -qy cmake python3-dev python3-pip gnuplot

# Install python libraries for testing
RUN pip3 install numpy matplotlib pytest scipy