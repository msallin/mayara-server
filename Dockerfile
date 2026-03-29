FROM rust:1.94-alpine AS builder
RUN apk add --no-cache build-base perl curl

WORKDIR /usr/src/mayara-server

# Cache dependencies in a separate layer
COPY Cargo.lock Cargo.toml build.rs ./
RUN mkdir src && echo "fn main() {}" > src/main.rs \
    && cargo build --release 2>/dev/null || true \
    && rm -rf src

COPY src src/
COPY web web/
RUN cargo build --release

FROM alpine:3.21
LABEL org.opencontainers.image.source="https://github.com/MarineYachtRadar/mayara-server"
LABEL org.opencontainers.image.title="mayara-server"
LABEL org.opencontainers.image.description="Marine radar server with REST API and WebSocket support"
LABEL org.opencontainers.image.license="Apache-2.0"
RUN apk add --no-cache tini \
    && adduser -D -h /home/mayara mayara
COPY --from=builder /usr/src/mayara-server/target/release/mayara-server /usr/local/bin/mayara-server
USER mayara
EXPOSE 6502
ENTRYPOINT ["tini", "--"]
CMD ["mayara-server"]
