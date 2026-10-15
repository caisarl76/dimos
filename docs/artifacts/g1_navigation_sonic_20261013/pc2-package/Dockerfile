# pc2 SONIC navigation graph image: read-only sensor sender, relay and execution owner.
# Base: the existing task image (verified by ID in provision.sh) already holds an ARM64
# CycloneDDS 0.10.5 build, uv and its managed Python. A separate virtual environment
# is created from this branch's lockfile; no host path or existing venv is used.
ARG BASE_IMAGE=dimos-g1-navigation-verify:20261012-pointlio
FROM ${BASE_IMAGE}

USER root
RUN mkdir -p /opt/sonic-navigation && chown nav:nav /opt/sonic-navigation
USER nav
ENV CYCLONEDDS_HOME=/opt/cyclonedds \
    LD_LIBRARY_PATH=/opt/cyclonedds/lib \
    UV_PROJECT_ENVIRONMENT=/opt/sonic-navigation/.venv \
    UV_LINK_MODE=copy \
    PYTHONDONTWRITEBYTECODE=1
WORKDIR /opt/sonic-navigation
COPY --chown=nav:nav dimos-source.tar /tmp/dimos-source.tar
RUN mkdir source && tar -xf /tmp/dimos-source.tar -C source && rm /tmp/dimos-source.tar && \
    cd source && uv sync --frozen --no-dev --extra unitree-dds --extra sonic-feedback
COPY --chown=nav:nav package/ /opt/sonic-navigation/package/
ENV PATH=/opt/sonic-navigation/.venv/bin:$PATH \
    PYTHONPATH=/opt/sonic-navigation/source:/opt/sonic-navigation/package
WORKDIR /opt/sonic-navigation/source
CMD ["python", "/opt/sonic-navigation/package/smoke.py"]
