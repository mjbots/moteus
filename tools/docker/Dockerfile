FROM ubuntu

ARG USER_ID
ARG GROUP_ID

RUN apt-get update
RUN apt-get install -y python3-pep517 python3-can python3-serial python3-setuptools python3-pyelftools python3-qtpy python3-wheel libtinfo5
RUN apt-get install -y curl patch git

# RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user

USER user

# ignore git error "detected dubious ownership in repository at '/src'""
RUN git config --global --add safe.directory /src
