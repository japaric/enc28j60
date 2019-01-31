set -euxo pipefail

main() {
    if [ $TARGET = rustfmt ]; then
        cargo fmt -- --check
        return
    fi

    cargo check --target $TARGET
}

# fake Travis variables to be able to run this on a local machine
if [ -z ${TARGET-} ]; then
    TARGET=$(rustc -Vv | grep host | cut -d ' ' -f2)
fi

main
