FROM python:3.7-stretch as builder

RUN apt-get update && apt-get install -y liboctomap-dev libfcl-dev libgeos-dev libspatialindex-dev

COPY ./requirements.txt /app/requirements.txt
RUN cd /app && \
    pip install --no-cache-dir cython dill numpy && \
    pip install --no-cache-dir bokeh distributed dask[bag] pymoo==0.3.2 && \
    pip install --no-cache-dir -r requirements.txt

COPY ./ /app
RUN cd /app && python setup.py install

FROM python:3.7-slim-stretch

RUN apt-get update && apt-get install -y liboctomap1.8 libfcl0.5 libgeos-c1v5 libspatialindex-c4v5 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/lib/python3.7/site-packages /usr/local/lib/python3.7/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin
COPY ./ /app

RUN ldconfig
