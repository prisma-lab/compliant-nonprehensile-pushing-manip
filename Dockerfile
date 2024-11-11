FROM ubuntu:22.04

#Instal essential, cmake and eigen
RUN apt-get update && apt-get install -y build-essential libssl-dev wget sudo && apt-get -y install cmake protobuf-compiler && apt-get install -y libeigen3-dev  && apt-get clean 

#Install RAISIM dependencies
RUN apt update && apt -y install minizip ffmpeg libvulkan1 mesa-vulkan-drivers libx11-dev libxext-dev
RUN ln -s /lib/x86_64-linux-gnu/libdl.so.2 /lib/x86_64-linux-gnu/libdl.so

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0

#ENV HOME /root
#Add non root user using UID and GID passed as argument
ENV HOME /home/user
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers
USER user

#Install ACADOS
COPY --chown=user dependencies/acados ${HOME}/acados
#COPY dependencies/acados ${HOME}/acados
RUN mkdir ${HOME}/acados/build
WORKDIR ${HOME}/acados/build
RUN cmake -DACADOS_WITH_QPOASES=ON ..
RUN make install -j4
RUN echo "source ${HOME}/acados/acados_env.sh" >> ${HOME}/.bashrc


#Install RAISIM
COPY --chown=user dependencies/raisimLib ${HOME}/raisimLib
COPY --chown=user dependencies/.raisim ${HOME}/.raisim
COPY --chown=user pushing_reduced ${HOME}/raisimLib/pushing_reduced

#COPY dependencies/raisimLib ${HOME}/raisimLib
RUN mkdir ${HOME}/raisimLib/build
WORKDIR ${HOME}/raisimLib/build
RUN echo "source ${HOME}/raisimLib/raisim_env.sh" >> ${HOME}/.bashrc
RUN cmake ..
RUN make
#SHELL ["/bin/bash", "-c"] 
#RUN source ${HOME}/.bashrc; cmake .. -DCMAKE_PREFIX_PATH=${HOME}/raisimLib/raisim/linux; make

#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user

#Set non root user and home work directory

WORKDIR ${HOME}




