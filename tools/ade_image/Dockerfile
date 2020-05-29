FROM ubuntu:bionic
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
        gnupg2 \
        locales \
        wget \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN echo deb http://packages.ros.org/ros/ubuntu ${CODENAME} main | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 5523BAEEB01FA116

RUN wget https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb \
    -O /tmp/packages-microsoft-prod.deb && \
    dpkg -i /tmp/packages-microsoft-prod.deb && \
    rm -f /tmp/packages-microsoft-prod.deb

COPY apt-packages /tmp/
RUN apt-get update && \
    apt-get install -y \
        $(cut -d# -f1 </tmp/apt-packages) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

COPY env.sh /etc/profile.d/ade_env.sh
COPY gitconfig /etc/gitconfig
COPY entrypoint /ade_entrypoint
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
