# A static container for the simulator page: build with Vite, serve with nginx on 8080.
#
#   docker build -t vector-simulacrum-web .                             robot drawn as physics shapes
#   docker build --build-arg WITH_SDK_MODEL=1 -t vector-simulacrum-web .   with the SDK's robot model
#   docker run --rm -p 8080:8080 vector-simulacrum-web
#
# WITH_SDK_MODEL=1 / WITH_RIG_MODEL=1 / WITH_DOCK_MODEL=1 place proprietary 3D meshes
# under public/robot/sdk/ (not open-source licences; read public/robot/sdk/LICENSE.txt
# and public/robot/sdk/DDL-ASSET-LICENSE.txt before distributing an image).

FROM python:3.12-slim AS sdkmodel
ARG WITH_SDK_MODEL=0
ARG WITH_PRINT_PARTS=0
ARG WITH_RIG_MODEL=0
ARG WITH_DOCK_MODEL=0
ARG WITH_CUBE_MODEL=0
WORKDIR /w
COPY tools/fetch_sdk_model.py tools/fetch_print_parts.py tools/fetch_rig_model.py tools/fetch_dock_model.py tools/fetch_cube_model.py tools/
COPY public/robot/model.json public/robot/model.json
RUN mkdir -p public/robot/sdk && \
    if [ "$WITH_SDK_MODEL" = "1" ]; then python3 tools/fetch_sdk_model.py --download; fi
RUN if [ "$WITH_PRINT_PARTS" = "1" ]; then test "$WITH_SDK_MODEL" = "1" && python3 tools/fetch_print_parts.py; fi
RUN if [ "$WITH_DOCK_MODEL" = "1" ]; then python3 tools/fetch_dock_model.py; fi
RUN if [ "$WITH_CUBE_MODEL" = "1" ]; then python3 tools/fetch_cube_model.py --download; fi
RUN if [ "$WITH_RIG_MODEL" = "1" ]; then pip install --no-cache-dir numpy && python3 tools/fetch_rig_model.py --download; fi

FROM node:22-alpine AS build
WORKDIR /build
COPY package.json package-lock.json ./
RUN npm ci --no-audit --no-fund
COPY . ./
COPY --from=sdkmodel /w/public/robot/sdk ./public/robot/sdk
RUN npm run build

FROM nginxinc/nginx-unprivileged:alpine
COPY --from=build /build/dist /usr/share/nginx/html
EXPOSE 8080
