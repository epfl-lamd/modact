FROM python:3.12-stretch as builder

RUN apt-get update && apt-get install -y liboctomap-dev libfcl-dev libgeos-dev libspatialindex-dev

RUN cd /app && \
    pip install --no-cache-dir cython dill numpy && \
    pip install --no-cache-dir bokeh distributed dask[bag] pymoo==0.6.1.3

COPY ./ /app
RUN cd /app && pip install .

FROM python:3.12-slim-stretch

RUN apt-get update && apt-get install -y liboctomap1.8 libfcl0.7 libgeos-c1v5 libspatialindex-c4v5 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin
COPY ./ /app

RUN ldconfig
