set -euxo pipefail

main() {
    if [ $TARGET = rustfmt ]; then
        cargo fmt -- --check
        return
    fi

    cargo check --target $TARGET
}

main
