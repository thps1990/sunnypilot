#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.7"
fi

export STAGING_ROOT="/data/safe_staging"

# Konik Stable environment configuration
if [ -f "/data/konik-stable.env" ]; then
  source "/data/konik-stable.env"
fi

export API_HOST="${API_HOST:-https://api.konik.ai}"
export ATHENA_HOST="${ATHENA_HOST:-wss://athena.konik.ai}"
export MAPS_HOST="${MAPS_HOST:-https://maps.konik.ai}"
