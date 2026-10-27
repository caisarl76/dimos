# 223's greeter stack (docker/g1-demo/compose.yaml): the unitree-g1-sonic-arm blueprint and
# greeter.py, CPU only, no ROS. Built from the checkout, deps from uv.lock with just the extras
# that blueprint imports.
FROM ubuntu:24.04

ARG UID=1000
ENV DEBIAN_FRONTEND=noninteractive
# scripts/install.sh's UBUNTU_PACKAGES minus pre-commit, plus ssh for the pc2 tunnel and voice.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git g++ pkg-config portaudio19-dev libturbojpeg libgl1 libegl1 \
    libglib2.0-0 ffmpeg libsndfile1 openssh-client \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:0.11.14 /uv /usr/local/bin/

# Same uid as the host user so the bind-mounted ~/.ssh is usable (ssh refuses a config file
# owned by someone else) and the state volume stays writable.
RUN userdel -r ubuntu && useradd -m -u ${UID} dimos && mkdir /app && chown dimos /app
USER dimos
WORKDIR /app
ENV UV_LINK_MODE=copy
ARG EXTRAS="--extra agents --extra web --extra sonic-feedback --extra cpu --extra sim"

COPY --chown=dimos pyproject.toml uv.lock .python-version ./
RUN --mount=type=cache,target=/home/dimos/.cache/uv,uid=${UID} \
    uv sync --frozen --no-default-groups --no-install-project ${EXTRAS}
COPY --chown=dimos . .
RUN --mount=type=cache,target=/home/dimos/.cache/uv,uid=${UID} \
    CIBUILDWHEEL=1 uv sync --frozen --no-default-groups ${EXTRAS} && mkdir -p /home/dimos/.local/state

ENV PATH=/app/.venv/bin:$PATH VIRTUAL_ENV=/app/.venv PYTHONUNBUFFERED=1
