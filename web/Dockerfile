# A static container for the simulator page: build with Vite, serve with nginx on 8080.
#
#   docker build -t vector-simulacrum-web .                             robot drawn as physics shapes
#   docker build --build-arg WITH_SDK_MODEL=1 -t vector-simulacrum-web .   with the SDK's robot model
#   docker run --rm -p 8080:8080 vector-simulacrum-web
#
# WITH_SDK_MODEL=1 downloads the Vector SDK wheel and puts its 3D model in the
# image. That model is under the Anki Image and 3D Model License, not an
# open-source licence: read public/robot/sdk/LICENSE.txt before giving such an
# image to anyone else.

FROM python:3.12-slim AS sdkmodel
ARG WITH_SDK_MODEL=0
WORKDIR /w
COPY tools/fetch_sdk_model.py tools/fetch_sdk_model.py
RUN mkdir -p public/robot/sdk && \
    if [ "$WITH_SDK_MODEL" = "1" ]; then python3 tools/fetch_sdk_model.py --download; fi

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
