FROM emscripten/emsdk:3.1.44 as builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    ninja-build \
    build-essential \
    git \
    curl \
    unzip \
    tar \
    zip \
    pkg-config \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


RUN cd /opt && \
    git clone https://github.com/Microsoft/vcpkg.git -n && \
    cd vcpkg && \
    git checkout d5b03c125afee1d9cef38f4cfa77e229400fb48a && \
    ./bootstrap-vcpkg.sh && \
    ./vcpkg integrate install

WORKDIR /app

ENV VCPKG_BUILD_TYPE=release
ENV VCPKG_MAX_CONCURRENCY=1

COPY vcpkg.json .
RUN /opt/vcpkg/vcpkg install --triplet wasm32-emscripten

COPY . ./
RUN /opt/vcpkg/downloads/tools/cmake-3.27.1-linux/cmake-3.27.1-linux-x86_64/bin/cmake . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release "-DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=${EMSDK}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" "-DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake" "-DVCPKG_TARGET_TRIPLET=wasm32-emscripten" && cmake --build build

FROM nginx as server
COPY --from=builder /app/build/main.* /usr/share/nginx/html/
RUN cp /usr/share/nginx/html/main.html /usr/share/nginx/html/index.html
