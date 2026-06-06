# Dev container base = the exact ESPHome release end-users get via
#   external_components: { source: github://slimcdk/esphome-custom-components }
#
# Pin/track by changing the tag: :beta (the upcoming release), :stable
# (currently 2026.4.x), a specific release like :2026.4.5, or :dev (nightly).
FROM ghcr.io/esphome/esphome:dev

# The runtime image's entrypoint auto-runs the ESPHome dashboard and exec's
# `esphome`; reset it so the dev container can idle for VS Code to attach.
ENTRYPOINT []

# git (+ git-lfs — the repo stores images via LFS, see .gitattributes) and
# Node.js 22 for development.
#
# The claude / gemini CLIs are NOT installed in the image: they're bind-
# mounted from the host (see devcontainer.json `mounts`) so the container
# runs the host's exact versions and auth. Node is still required because
# the Gemini CLI is a Node bundle (the Claude CLI is a self-contained native
# binary). The VS Code extensions (anthropic.claude-code,
# google.gemini-cli-vscode-ide-companion) run server-side here and spawn
# those CLIs.
RUN apt-get update \
 && apt-get install -y --no-install-recommends git git-lfs ca-certificates curl gnupg \
 && git lfs install --system \
 && curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
 && apt-get install -y --no-install-recommends nodejs \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
