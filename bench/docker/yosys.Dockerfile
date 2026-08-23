# Yosys baseline image for the Struo QoR suite.
#
# Pinned to ubuntu:24.04 so that the Yosys baseline shares provenance with the
# locally installed nextpnr-ecp5 0.6-3build5 package (also Ubuntu 24.04).
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends yosys \
    && rm -rf /var/lib/apt/lists/*

RUN yosys -V
ENTRYPOINT ["yosys"]
