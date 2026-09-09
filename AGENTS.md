# Repository Guidelines

## Project Structure & Module Organization
- `esphome/config/` holds ESPHome YAML configs and `custom_components/` C++/Python extensions.
- `home_assistant/config/` contains Home Assistant configuration (`configuration.yaml`, `automations.yaml`, `secrets.yaml`).
- `docker/` has Dockerfiles and helper compose files for ESPHome and Home Assistant.
- Root scripts like `print_audio.py` and `config_codex.sh` are utility helpers.

## Build, Test, and Development Commands
- `docker compose build` builds the ESPHome and Home Assistant images defined in `docker-compose.yml`.
- `docker compose up -d esphome` starts the ESPHome service (UI on `http://localhost:6052`).
- `docker compose up -d home_assistant` starts Home Assistant (UI on `http://localhost:8123`).
- From inside the ESPHome container, use `esphome run /config/<device>.yaml` to build and flash.

## Coding Style & Naming Conventions
- YAML uses 2-space indentation; keep `id` and component names consistent with existing files in `esphome/config/*.yaml`.
- C++ in `custom_components/` follows ESPHome/ESP-IDF style: 2-space indents, braces on same line, `snake_case_` for private members.
- Python helpers are lightweight; prefer simple, readable scripts without heavy dependencies.

## Testing Guidelines
- No automated test framework is configured. Validation is device- and UI-driven.
- For audio features, verify via ESPHome logs and HA UI actions; document manual steps in PRs.

## Commit & Pull Request Guidelines
- Commit history uses short, informal subjects; no strict convention enforced. Keep messages concise and specific.
- PRs should describe the device/config touched, include repro steps, and link any related issues.
- If behavior changes, add a short manual test checklist and any relevant screenshots/log excerpts.

## Security & Configuration Tips
- Do not commit secrets: keep credentials in `home_assistant/config/secrets.yaml`.
- Proxy settings are defined in `docker-compose.yml`; update them only if your local network requires it.
