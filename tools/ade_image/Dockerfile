FROM ros:dashing
ARG CODENAME=bionic


# Disable non-free repositories
RUN if [ "$(uname -m)" = "x86_64" ]; then \
        echo "\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME} main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-backports main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-updates main universe\n\
deb http://security.ubuntu.com/ubuntu/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    else \
        echo "\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME} main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-backports main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-updates main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    fi


RUN apt-get update && \
    apt-get install -y \
        locales \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN echo deb http://packages.ros.org/ros/ubuntu ${CODENAME} main | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 5523BAEEB01FA116

RUN echo deb https://deb.nodesource.com/node_10.x ${CODENAME} main | tee /etc/apt/sources.list.d/nodesource.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 1655A0AB68576280

COPY apt-packages /tmp/
RUN apt-get update && \
    apt-get install -y \
        $(cut -d# -f1 </tmp/apt-packages) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers


#COPY pip3-packages /tmp/
#RUN pip3 install -U \
#        $(cut -d# -f1 </tmp/pip3-packages) \
#    && rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages


#RUN mkdir -p /opt/aw-atom-config/bin
#COPY atom-install-our-plugins /etc/atom/atom-install-our-plugins


#RUN git clone https://github.com/rigtorp/udpreplay && mkdir -p udpreplay/build \
#      && cd udpreplay/build && cmake .. && make && make install \
#      && cd - && rm -rf udpreplay/


COPY bashrc-git-prompt /
RUN cat /bashrc-git-prompt >> /etc/skel/.bashrc && \
    rm /bashrc-git-prompt
#COPY gdbinit /etc/gdb/


# ===================== CLEAN UP ZONE ===================== #
# Commands in the cleanup zone will be cleaned up before every release
# and put into the correct place.
RUN apt-get update \
  && apt-get install -y \
    unzip \
    ca-certificates \
    libx11-6 \
    libxau6 \
    libxcb1 \
    libxdmcp6 \
    libxext6 \
    libvulkan1 \
    libgl1 \
    libgtk2.0-0 \
    vulkan-utils \
  && apt-get clean

ADD "https://gitlab.com/nvidia/container-images/vulkan/raw/master/nvidia_icd.json" /etc/vulkan/icd.d/nvidia_icd.json
RUN chmod 644 /etc/vulkan/icd.d/nvidia_icd.json

#COPY npm-packages /tmp/
#RUN . /opt/ros/dashing/setup.sh && \
#    npm install -g --unsafe \
#        $(cut -d# -f1 </tmp/npm-packages) && \
#    rm -rf /tmp/npm-packages

# ===================== END OF CLEAN UP ZONE ===================== #

COPY env.sh /etc/profile.d/ade_env.sh
COPY gitconfig /etc/gitconfig
COPY entrypoint /ade_entrypoint
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
