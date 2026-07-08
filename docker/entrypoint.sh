#!/usr/bin/env bash
# Runs when the ROS container starts. Sets things up, then hands off to the CMD
# (which is `sleep infinity` -- the container stays alive so you can open
# terminals into it and launch Part 1 or Part 2 yourself).
set -e

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

# Wait for the Ollama service, then make sure the model is present (cached after
# the first pull). Only the LLM-driven launches need this, but doing it once here
# means it's ready whichever part you run.
echo "[entrypoint] waiting for Ollama at ${OLLAMA_HOST} ..."
until curl -sf "${OLLAMA_HOST}/api/tags" >/dev/null 2>&1; do sleep 2; done
echo "[entrypoint] Ollama up. Ensuring model '${LLM_MODEL}' ..."
curl -s "${OLLAMA_HOST}/api/pull" -d "{\"name\":\"${LLM_MODEL}\"}" \
  | grep -o '"status":"[^"]*"' | tail -n 1 || true

echo "[entrypoint] ready. Open a terminal with:  docker compose exec ros bash"
exec "$@"
